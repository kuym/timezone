//! Binary serializer — full-artifact compact format.
//!
//! The **quadtree section is implemented**, reusing the `quad` format's encoding
//! primitives: the `q` variable-length integer ([`super::qvarint`]) and a
//! recursive, length-prefixed node structure so a reader can skip whole subtrees
//! in O(depth).  Unlike `quad` (one tzid per leaf), this encodes the real
//! cost-model quadtree — each node carries its `eref` list (zones that fully
//! cover the cell) and, at leaves, the `ref` candidates with their localized
//! point-in-polygon data (winding, edge runs, holes).
//!
//! The **polygon / zone geometry section is now implemented**, so the file is a
//! complete standalone artifact (`is_complete()` is true).  Each polygon uses the
//! same packed delta encoding as the `json` export (`polycodec`), but the byte
//! stream is stored raw (no base64) behind a `q`-length prefix.
//!
//! Zones are referenced by **rank**, not original id: they are renumbered by
//! descending reference count (P5), so the busiest zones get 1-byte ids, and the
//! zones section is written in that rank order (rank = index).
//!
//! ## Layout
//!
//! ```text
//!   header:
//!     magic "TZQT", version:u16
//!     quant: xMin,xMax,yMin,yMax : i32 ; xScale,yScale : f64 ; maxDepth : u16
//!     counts: zones:u32, tz:u32, nodes:u32, arcs:u32
//!   section 1 — quadtree (one recursive node):
//!     node := bodyLen:q                         (byte length of the body; skip = jump bodyLen)
//!             hdr:q                             (bit0 = internal; bit1 = has eref;
//!                                                leaf: bits2+ = refCount)                  [P2]
//!             [ erefCount:q, erefCount × rankDelta:q ]   if bit1  (sorted, delta-coded)    [P5]
//!             if internal: child0 child1 child2 child3   (each a node)
//!             else (leaf): refCount × candidate           (sorted by rank, z delta-coded)  [P4/P5]
//!     candidate := zDelta:q                     (rank delta from the previous candidate)   [P4/P5]
//!                  packed:q                     ((outerArcCount<<3)|(windCode<<1)|hasHoles) [P1]
//!                  [ w:zigzag-q ]  if windCode == 3   (escape for |w| > 1)
//!                  outerArcCount × arcRuns            (the outer arcs crossing this cell)
//!                  [ holeCount:q, holeCount × ( i:q, w:zigzag-q, arcCount:q, arcCount × arcRuns ) ]
//!     arcRuns := (arcIdxDelta << 1 | hasMoreRuns):q, gap0:q, len0:q,
//!                [ if hasMoreRuns: extraCount:q, extraCount × ( gap:q, len:q ) ]
//!         arcIdxDelta indexes the ring's arc-ref list; a reader decodes ONLY these
//!         crossing arcs (not the candidate's whole ring) to run the localized test.
//!         The first edge run is inline (~93% of arcs have just one), so the old
//!         always-1 runCount byte is gone.
//!   section 2 — arcs (`arcs` count entries — the shared polygon boundaries):
//!     arc := ox:svar, oy:svar, pLen:uvar, p:bytes   (polycodec packed, no base64)
//!         Origins are delta-coded against the previous arc (consecutive arcs lie
//!         close together), so an origin is usually a few bytes, not a full coord.
//!   section 3 — zones (`zones` count entries, in rank order):
//!     zone := tzid:uvar                          (alphabetical rank — see tz names)
//!             area:  uvar
//!             aabb:  xLo:svar, yLo:svar, (xHi-xLo):uvar, (yHi-yLo):uvar
//!             outer: refCount:uvar, refCount × arcRef
//!             holeCount:uvar, holeCount × ( refCount:uvar, refCount × arcRef )
//!     arcRef := uvar( zigzag(arcIndex - prevArcIndex) << 1 | reversed )
//!         Arc indices are delta-coded (~50% are consecutive), `prev` running
//!         across the zone's rings — so most refs are a single byte.
//!   section 4 — tz names (`tz` count entries, indexed by tzid) — front-coded:
//!     tzCount:uvar, tzCount × ( sharedPrefixLen:uvar, suffixLen:uvar, suffix:bytes )
//!         Names are alphabetized and each shares its longest prefix with the
//!         previous, so `America/` etc. is stored once (a linearized prefix tree).
//!         `zone.tzid` is remapped to this alphabetical order (index = tzid).
//! ```
//!
//! A zone's ring is rebuilt by concatenating its referenced arcs (reversed when
//! the low bit is set), dropping the junction shared with the previous arc and
//! the final closure — see `topology::reconstruct`.  `uvar`/`svar` are LEB128
//! (svar = zigzag): exact and 1 byte for small values, with no per-value count
//! byte — used by the arcs, zones and tz-names sections; the quadtree keeps the
//! `q` varint (paddable node lengths).  `windCode`: 0→w=0, 1→w=−1, 2→w=+1,
//! 3→escape (full ZigZag w follows).  Runs are sorted, non-overlapping
//! `[first, last]` edge ranges stored as gap-from-previous plus length.

use super::qvarint::{read_q, representable, unzigzag, write_q, zigzag, Q_MAX};
use super::Serializer;
use crate::build::{ArcRun, Cand, Output};
use crate::quant;
use crate::topology::ArcRef;

const MAGIC: &[u8; 4] = b"TZQT";
const VERSION: u16 = 0; // 0 = pre-release (geometry sections not yet implemented)

const HDR_INTERNAL: u64 = 1;
const HDR_EREF: u64 = 2;

pub struct BinarySerializer;

impl Serializer for BinarySerializer {
    fn serialize(&self, out: &Output) -> Result<Vec<u8>, String> {
        // P5: renumber zones by descending reference count so the busiest get the
        // lowest (1-byte) ranks.  The tree references ranks; the zones section is
        // written in this rank order (order[rank] = original zone id).
        let order = compute_order(out);
        let mut remap = vec![0u32; out.zones.len()];
        for (rank, &old) in order.iter().enumerate() {
            remap[old] = rank as u32;
        }

        // Renumber timezones into alphabetical order so the front-coded name table
        // shares maximal prefixes; the zones' `tzid` field references this order.
        let mut tz_order: Vec<usize> = (0..out.tz.len()).collect();
        tz_order.sort_by(|&a, &b| out.tz[a].n.cmp(&out.tz[b].n));
        let mut tz_remap = vec![0u32; out.tz.len()];
        for (rank, &old) in tz_order.iter().enumerate() {
            tz_remap[old] = rank as u32;
        }

        let mut buf = Vec::new();

        // --- header ---
        buf.extend_from_slice(MAGIC);
        buf.extend_from_slice(&VERSION.to_le_bytes());
        for v in [quant::X_MIN, quant::X_MAX, quant::Y_MIN, quant::Y_MAX] {
            buf.extend_from_slice(&(v as i32).to_le_bytes());
        }
        buf.extend_from_slice(&quant::X_SCALE.to_le_bytes());
        buf.extend_from_slice(&quant::Y_SCALE.to_le_bytes());
        buf.extend_from_slice(&(quant::MAX_DEPTH as u16).to_le_bytes());
        buf.extend_from_slice(&(out.zones.len() as u32).to_le_bytes());
        buf.extend_from_slice(&(out.tz.len() as u32).to_le_bytes());
        buf.extend_from_slice(&(out.arena.len() as u32).to_le_bytes());
        buf.extend_from_slice(&(out.arc_packed.len() as u32).to_le_bytes());

        // --- section 1: quadtree ---
        let tree_start = buf.len();
        encode_node(out, out.root, &mut buf, &remap)?;
        let mut pos = tree_start;
        let decoded = decode_node(&buf, &mut pos);
        if pos != buf.len() || decoded != arena_to_dec(out, out.root, &remap) {
            return Err("binary quadtree round-trip mismatch".to_string());
        }
        eprintln!("binary: quadtree section {} bytes ({} nodes)", pos - tree_start, out.arena.len());

        // --- section 2: arcs (shared polygon boundaries) ---
        let arcs_start = buf.len();
        encode_arcs(out, &mut buf);
        let mut apos = arcs_start;
        if decode_arcs(&buf, &mut apos, out.arc_packed.len()) != source_arcs(out) {
            return Err("binary arcs round-trip mismatch".to_string());
        }
        eprintln!("binary: arcs section {} bytes ({} arcs)", apos - arcs_start, out.arc_packed.len());

        // --- section 3: zones (arc references + metadata) ---
        let zones_start = buf.len();
        encode_zones(out, &order, &tz_remap, &mut buf);
        let mut zpos = zones_start;
        if decode_zones(&buf, &mut zpos, out.zones.len()) != source_zones(out, &order, &tz_remap) {
            return Err("binary zones round-trip mismatch".to_string());
        }
        eprintln!("binary: zones section {} bytes ({} polygons)", zpos - zones_start, out.zones.len());

        // --- section 4: tz names (front-coded, in `tz_order`) ---
        let tz_start = buf.len();
        encode_tz_names(out, &tz_order, &mut buf);
        let mut tpos = tz_start;
        let src_names: Vec<String> = tz_order.iter().map(|&i| out.tz[i].n.clone()).collect();
        if decode_tz_names(&buf, &mut tpos) != src_names {
            return Err("binary tz-names round-trip mismatch".to_string());
        }
        if tpos != buf.len() {
            return Err("binary tz-names section did not consume to end".to_string());
        }
        eprintln!("binary: tz-names section {} bytes ({} names)", tpos - tz_start, out.tz.len());

        Ok(buf)
    }

    fn format_name(&self) -> &'static str {
        "binary"
    }

    fn is_complete(&self) -> bool {
        true
    }
}

// P5: reference count = candidate appearances + eref appearances, per zone.
// Returns order[rank] = original zone id.
fn compute_order(out: &Output) -> Vec<usize> {
    let mut refs = vec![0u64; out.zones.len()];
    for node in &out.arena {
        for &z in &node.eref {
            refs[z] += 1;
        }
        for cand in &node.cands {
            refs[cand.z] += 1;
        }
    }
    let mut order: Vec<usize> = (0..out.zones.len()).collect();
    order.sort_by(|&a, &b| refs[b].cmp(&refs[a]).then(a.cmp(&b)));
    order
}

// --- encoding ---

fn encode_node(out: &Output, idx: usize, buf: &mut Vec<u8>, remap: &[u32]) -> Result<(), String> {
    let node = &out.arena[idx];
    let is_internal = node.q.is_some();
    let has_eref = !node.eref.is_empty();

    let mut body: Vec<u8> = Vec::new();

    // P2: one header value carries internal/eref flags and (for leaves) refCount.
    let header = if is_internal {
        (has_eref as u64) << 1 | HDR_INTERNAL
    } else {
        (node.cands.len() as u64) << 2 | (has_eref as u64) << 1
    };
    write_q(&mut body, header);

    // P5: erefs as sorted, delta-coded ranks.
    if has_eref {
        let mut ranks: Vec<u32> = node.eref.iter().map(|&z| remap[z]).collect();
        ranks.sort_unstable();
        write_q(&mut body, ranks.len() as u64);
        let mut prev = 0u32;
        for r in ranks {
            write_q(&mut body, (r - prev) as u64);
            prev = r;
        }
    }

    if let Some(children) = node.q {
        for &c in children.iter() {
            encode_node(out, c, &mut body, remap)?;
        }
    } else {
        // P4/P5: candidates sorted by rank, z delta-coded.
        let mut cands: Vec<&Cand> = node.cands.iter().collect();
        cands.sort_by_key(|c| remap[c.z]);
        let mut prev = 0u32;
        for c in cands {
            let rank = remap[c.z];
            write_q(&mut body, (rank - prev) as u64);
            prev = rank;
            encode_candidate_meta(c, &mut body);
        }
    }

    let len = representable(body.len() as u64);
    if len > Q_MAX {
        return Err(format!("binary: node body {} bytes exceeds the {}-byte limit", body.len(), Q_MAX));
    }
    write_q(buf, len);
    buf.extend_from_slice(&body);
    buf.resize(buf.len() + (len as usize - body.len()), 0); // pad to representable length
    Ok(())
}

// P1: winding, outer arc count and hole presence share one `q`.
fn encode_candidate_meta(cand: &Cand, buf: &mut Vec<u8>) {
    let wind_code: u64 = match cand.w {
        0 => 0,
        -1 => 1,
        1 => 2,
        _ => 3, // escape
    };
    let has_holes = !cand.h.is_empty();
    let packed = (cand.e.len() as u64) << 3 | wind_code << 1 | (has_holes as u64);
    write_q(buf, packed);
    if wind_code == 3 {
        write_q(buf, zigzag(cand.w));
    }
    encode_arc_runs(&cand.e, buf); // outer crossing arcs (count carried by `packed`)
    if has_holes {
        write_q(buf, cand.h.len() as u64);
        for h in &cand.h {
            write_q(buf, h.i as u64);
            write_q(buf, zigzag(h.w));
            write_q(buf, h.e.len() as u64); // hole crossing-arc count
            encode_arc_runs(&h.e, buf);
        }
    }
}

// Each crossing arc: (arcIdxDelta << 1 | hasMoreRuns), then the first edge run
// inline (gap, len); only when hasMoreRuns is the remaining run count + runs
// stored.  ~93% of crossing arcs have exactly one run, so this drops a per-arc
// run-count byte for almost all of them.
fn encode_arc_runs(e: &[ArcRun], buf: &mut Vec<u8>) {
    let mut prev = 0u32;
    for ar in e {
        let has_more = ar.runs.len() > 1;
        write_q(buf, (((ar.a - prev) as u64) << 1) | has_more as u64);
        prev = ar.a;
        let (f0, l0) = ar.runs[0];
        write_q(buf, f0 as u64);
        write_q(buf, (l0 - f0) as u64);
        if has_more {
            write_q(buf, (ar.runs.len() - 1) as u64);
            let mut pr = l0 as u64 + 1;
            for &(f, l) in &ar.runs[1..] {
                write_q(buf, f as u64 - pr);
                write_q(buf, (l - f) as u64);
                pr = l as u64 + 1;
            }
        }
    }
}

// --- decoding (round-trip verification / reader reference) ---

#[derive(PartialEq, Debug)]
struct DecNode {
    eref: Vec<u64>,
    kind: DecKind,
}

#[derive(PartialEq, Debug)]
enum DecKind {
    Internal(Vec<DecNode>),
    Leaf(Vec<DecCand>),
}

// e / hole-e are lists of (arcIndex, localRuns) — the crossing arcs.
type DecArcRuns = Vec<(u64, Vec<(u64, u64)>)>;

#[derive(PartialEq, Debug)]
struct DecCand {
    z: u64,
    w: i64,
    e: DecArcRuns,
    h: Vec<(u64, i64, DecArcRuns)>,
}

fn decode_node(bytes: &[u8], pos: &mut usize) -> DecNode {
    let body_len = read_q(bytes, pos);
    let end = *pos + body_len as usize;

    let header = read_q(bytes, pos);
    let is_internal = header & HDR_INTERNAL != 0;
    let has_eref = header & HDR_EREF != 0;

    let mut eref = Vec::new();
    if has_eref {
        let n = read_q(bytes, pos);
        let mut cum = 0u64;
        for _ in 0..n {
            cum += read_q(bytes, pos);
            eref.push(cum);
        }
    }

    let kind = if is_internal {
        let mut children = Vec::with_capacity(4);
        for _ in 0..4 {
            children.push(decode_node(bytes, pos));
        }
        DecKind::Internal(children)
    } else {
        let ref_count = header >> 2;
        let mut cands = Vec::with_capacity(ref_count as usize);
        let mut cum = 0u64;
        for _ in 0..ref_count {
            cum += read_q(bytes, pos); // rank delta
            let z = cum;
            let (w, e, h) = decode_candidate_meta(bytes, pos);
            cands.push(DecCand { z, w, e, h });
        }
        DecKind::Leaf(cands)
    };

    *pos = end; // skip any padding
    DecNode { eref, kind }
}

fn decode_candidate_meta(bytes: &[u8], pos: &mut usize) -> (i64, DecArcRuns, Vec<(u64, i64, DecArcRuns)>) {
    let packed = read_q(bytes, pos);
    let arc_count = packed >> 3;
    let wind_code = (packed >> 1) & 3;
    let has_holes = packed & 1 != 0;

    let w = match wind_code {
        0 => 0,
        1 => -1,
        2 => 1,
        _ => unzigzag(read_q(bytes, pos)),
    };
    let e = decode_arc_runs(bytes, pos, arc_count);

    let mut h = Vec::new();
    if has_holes {
        let hn = read_q(bytes, pos);
        for _ in 0..hn {
            let i = read_q(bytes, pos);
            let hw = unzigzag(read_q(bytes, pos));
            let hac = read_q(bytes, pos);
            h.push((i, hw, decode_arc_runs(bytes, pos, hac)));
        }
    }
    (w, e, h)
}

// arcCount crossing arcs: arcIdxDelta<<1|hasMore, first run inline, then extras.
fn decode_arc_runs(bytes: &[u8], pos: &mut usize, arc_count: u64) -> DecArcRuns {
    let mut out = Vec::with_capacity(arc_count as usize);
    let mut cum = 0u64;
    for _ in 0..arc_count {
        let v = read_q(bytes, pos);
        cum += v >> 1;
        let f0 = read_q(bytes, pos);
        let l0 = f0 + read_q(bytes, pos);
        let mut runs = vec![(f0, l0)];
        if v & 1 == 1 {
            let extra = read_q(bytes, pos);
            let mut pr = l0 + 1;
            for _ in 0..extra {
                let f = pr + read_q(bytes, pos);
                let l = f + read_q(bytes, pos);
                runs.push((f, l));
                pr = l + 1;
            }
        }
        out.push((cum, runs));
    }
    out
}

fn arc_runs_to_dec(e: &[ArcRun]) -> DecArcRuns {
    e.iter()
        .map(|ar| (ar.a as u64, ar.runs.iter().map(|&(a, b)| (a as u64, b as u64)).collect()))
        .collect()
}

fn arena_to_dec(out: &Output, idx: usize, remap: &[u32]) -> DecNode {
    let node = &out.arena[idx];
    let mut eref: Vec<u64> = node.eref.iter().map(|&z| remap[z] as u64).collect();
    eref.sort_unstable();
    let kind = if let Some(children) = node.q {
        DecKind::Internal(children.iter().map(|&c| arena_to_dec(out, c, remap)).collect())
    } else {
        let mut cands: Vec<DecCand> = node
            .cands
            .iter()
            .map(|c| DecCand {
                z: remap[c.z] as u64,
                w: c.w,
                e: arc_runs_to_dec(&c.e),
                h: c.h.iter().map(|h| (h.i as u64, h.w, arc_runs_to_dec(&h.e))).collect(),
            })
            .collect();
        cands.sort_by_key(|c| c.z);
        DecKind::Leaf(cands)
    };
    DecNode { eref, kind }
}

// --- section 2 (arcs) & 3 (zones) varints ---

// LEB128 varints — exact for any magnitude, 1 byte for values < 128, and no
// per-value count byte.  Used throughout the arcs, zones and tz-names sections,
// where most values are tiny (delta-coded arc origins/refs, front-coded lengths).
fn write_uvar(buf: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7F) as u8;
        v >>= 7;
        if v == 0 {
            buf.push(b);
            break;
        }
        buf.push(b | 0x80);
    }
}
fn read_uvar(bytes: &[u8], pos: &mut usize) -> u64 {
    let (mut v, mut shift) = (0u64, 0u32);
    loop {
        let b = bytes[*pos];
        *pos += 1;
        v |= ((b & 0x7F) as u64) << shift;
        if b & 0x80 == 0 {
            break;
        }
        shift += 7;
    }
    v
}
fn write_svar(buf: &mut Vec<u8>, v: i64) {
    write_uvar(buf, zigzag(v));
}
fn read_svar(bytes: &[u8], pos: &mut usize) -> i64 {
    unzigzag(read_uvar(bytes, pos))
}

// Each arc as origin + packed delta stream (polycodec, no base64), LEB128 length.
// Origins are delta-coded against the previous arc (consecutive arcs are created
// together during topology and lie close together), so the origin usually fits in
// far fewer bytes than an absolute coordinate.
fn encode_arcs(out: &Output, buf: &mut Vec<u8>) {
    let (mut px, mut py) = (0i64, 0i64);
    for (o, p) in &out.arc_packed {
        write_svar(buf, o[0] - px);
        write_svar(buf, o[1] - py);
        px = o[0];
        py = o[1];
        write_uvar(buf, p.len() as u64);
        buf.extend_from_slice(p);
    }
}

// A ring's arc references: count, then each as (zigzag(arcIdx - prevArcIdx) << 1
// | reversed).  Arc indices in ring order are ~50% consecutive, so delta-coding
// makes most refs 1 byte.  `prev` runs across the zone's rings (outer then holes).
fn encode_arc_refs(refs: &[ArcRef], prev: &mut i64, buf: &mut Vec<u8>) {
    write_uvar(buf, refs.len() as u64);
    for r in refs {
        let idx = r.arc as i64;
        write_uvar(buf, (zigzag(idx - *prev) << 1) | r.rev as u64);
        *prev = idx;
    }
}

fn encode_zones(out: &Output, order: &[usize], tz_remap: &[u32], buf: &mut Vec<u8>) {
    for &old in order {
        let zone = &out.zones[old];
        write_uvar(buf, tz_remap[zone.tzid] as u64);
        write_uvar(buf, zone.a as u64);
        write_svar(buf, zone.aabb[0][0]);
        write_svar(buf, zone.aabb[0][1]);
        write_uvar(buf, (zone.aabb[1][0] - zone.aabb[0][0]) as u64);
        write_uvar(buf, (zone.aabb[1][1] - zone.aabb[0][1]) as u64);
        let mut prev = 0i64;
        encode_arc_refs(&zone.outer_refs, &mut prev, buf);
        write_uvar(buf, zone.hole_refs.len() as u64);
        for hrefs in &zone.hole_refs {
            encode_arc_refs(hrefs, &mut prev, buf);
        }
    }
}

// Longest common byte prefix of two ASCII tz names.
fn common_prefix(a: &[u8], b: &[u8]) -> usize {
    a.iter().zip(b).take_while(|(x, y)| x == y).count()
}

// Front-coded (prefix-compressed) name table, in `tz_order` (sorted).  Each name
// shares its longest prefix with the previous, so common prefixes like `America/`
// are stored once — the linearized form of a prefix tree.  `zone.tzid` is
// remapped to this sorted order so the table is index = tzid.
fn encode_tz_names(out: &Output, tz_order: &[usize], buf: &mut Vec<u8>) {
    write_uvar(buf, out.tz.len() as u64);
    let mut prev: &[u8] = &[];
    for &ti in tz_order {
        let name = out.tz[ti].n.as_bytes();
        let shared = common_prefix(prev, name);
        write_uvar(buf, shared as u64);
        write_uvar(buf, (name.len() - shared) as u64);
        buf.extend_from_slice(&name[shared..]);
        prev = name;
    }
}

// --- round-trip decoding for sections 2, 3 & 4 ---

#[derive(PartialEq, Debug)]
struct DecArc {
    o: (i64, i64),
    p: Vec<u8>,
}

// An arc reference decodes to (arcIndex, reversed).
#[derive(PartialEq, Debug)]
struct DecZone {
    tzid: u64,
    area: u64,
    aabb: [i64; 4],
    outer: Vec<(u64, bool)>,
    holes: Vec<Vec<(u64, bool)>>,
}

fn decode_arcs(bytes: &[u8], pos: &mut usize, count: usize) -> Vec<DecArc> {
    let mut arcs = Vec::with_capacity(count);
    let (mut px, mut py) = (0i64, 0i64);
    for _ in 0..count {
        px += read_svar(bytes, pos);
        py += read_svar(bytes, pos);
        let plen = read_uvar(bytes, pos) as usize;
        let p = bytes[*pos..*pos + plen].to_vec();
        *pos += plen;
        arcs.push(DecArc { o: (px, py), p });
    }
    arcs
}

fn decode_arc_refs(bytes: &[u8], pos: &mut usize, prev: &mut i64) -> Vec<(u64, bool)> {
    let n = read_uvar(bytes, pos);
    (0..n)
        .map(|_| {
            let v = read_uvar(bytes, pos);
            *prev += unzigzag(v >> 1);
            (*prev as u64, v & 1 == 1)
        })
        .collect()
}

fn decode_zones(bytes: &[u8], pos: &mut usize, count: usize) -> Vec<DecZone> {
    let mut zones = Vec::with_capacity(count);
    for _ in 0..count {
        let tzid = read_uvar(bytes, pos);
        let area = read_uvar(bytes, pos);
        let xl = read_svar(bytes, pos);
        let yl = read_svar(bytes, pos);
        let xh = xl + read_uvar(bytes, pos) as i64;
        let yh = yl + read_uvar(bytes, pos) as i64;
        let mut prev = 0i64;
        let outer = decode_arc_refs(bytes, pos, &mut prev);
        let hn = read_uvar(bytes, pos);
        let holes = (0..hn).map(|_| decode_arc_refs(bytes, pos, &mut prev)).collect();
        zones.push(DecZone { tzid, area, aabb: [xl, yl, xh, yh], outer, holes });
    }
    zones
}

fn source_arcs(out: &Output) -> Vec<DecArc> {
    out.arc_packed.iter().map(|(o, p)| DecArc { o: (o[0], o[1]), p: p.clone() }).collect()
}

fn refs_to_dec(refs: &[ArcRef]) -> Vec<(u64, bool)> {
    refs.iter().map(|r| (r.arc as u64, r.rev)).collect()
}

fn source_zones(out: &Output, order: &[usize], tz_remap: &[u32]) -> Vec<DecZone> {
    order
        .iter()
        .map(|&old| {
            let z = &out.zones[old];
            DecZone {
                tzid: tz_remap[z.tzid] as u64,
                area: z.a as u64,
                aabb: [z.aabb[0][0], z.aabb[0][1], z.aabb[1][0], z.aabb[1][1]],
                outer: refs_to_dec(&z.outer_refs),
                holes: z.hole_refs.iter().map(|h| refs_to_dec(h)).collect(),
            }
        })
        .collect()
}

fn decode_tz_names(bytes: &[u8], pos: &mut usize) -> Vec<String> {
    let n = read_uvar(bytes, pos);
    let mut names = Vec::with_capacity(n as usize);
    let mut prev: Vec<u8> = Vec::new();
    for _ in 0..n {
        let shared = read_uvar(bytes, pos) as usize;
        let suflen = read_uvar(bytes, pos) as usize;
        let mut name = prev[..shared].to_vec();
        name.extend_from_slice(&bytes[*pos..*pos + suflen]);
        *pos += suflen;
        names.push(String::from_utf8_lossy(&name).into_owned());
        prev = name;
    }
    names
}
