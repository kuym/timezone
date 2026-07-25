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
//!     arcRuns := arcIdxDelta:q, runCount:q, runCount × ( gap:q, len:q )
//!         arcIdxDelta indexes the ring's arc-ref list; a reader decodes ONLY these
//!         crossing arcs (not the candidate's whole ring) to run the localized test.
//!   section 2 — arcs (`arcs` count entries — the shared polygon boundaries):
//!     arc := ox:coord, oy:coord, pLen:uintLE, p:bytes   (polycodec packed, no base64)
//!   section 3 — zones (`zones` count entries, in rank order):
//!     zone := tzid:q
//!             area:  uintLE
//!             aabb:  xLo:coord, yLo:coord, (xHi-xLo):uintLE, (yHi-yLo):uintLE
//!             outer: refCount:q, refCount × arcRef
//!             holeCount:q, holeCount × ( refCount:q, refCount × arcRef )
//!     arcRef := uintLE( (arcIndex << 1) | reversed )
//!   section 4 — tz names (`tz` count entries, indexed by tzid):
//!     tzCount:q, tzCount × ( nameLen:q, name:UTF-8 bytes )
//! ```
//!
//! A zone's ring is rebuilt by concatenating its referenced arcs (reversed when
//! the low bit is set), dropping the junction shared with the previous arc and
//! the final closure — see `topology::reconstruct`.  `coord` = `unzigzag(uintLE)`;
//! `uintLE` = a `q` byte-count then that many LE bytes (exact for any magnitude,
//! unlike the even-only `q` 3-byte form).  `windCode`: 0→w=0, 1→w=−1, 2→w=+1,
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
        encode_zones(out, &order, &mut buf);
        let mut zpos = zones_start;
        if decode_zones(&buf, &mut zpos, out.zones.len()) != source_zones(out, &order) {
            return Err("binary zones round-trip mismatch".to_string());
        }
        eprintln!("binary: zones section {} bytes ({} polygons)", zpos - zones_start, out.zones.len());

        // --- section 4: tz names ---
        let tz_start = buf.len();
        encode_tz_names(out, &mut buf);
        let mut tpos = tz_start;
        if decode_tz_names(&buf, &mut tpos) != out.tz.iter().map(|t| t.n.clone()).collect::<Vec<_>>() {
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

// Each crossing arc: arcIndex (delta from previous, sorted ascending), then its
// local edge run count and runs.
fn encode_arc_runs(e: &[ArcRun], buf: &mut Vec<u8>) {
    let mut prev = 0u32;
    for ar in e {
        write_q(buf, (ar.a - prev) as u64);
        prev = ar.a;
        write_q(buf, ar.runs.len() as u64);
        encode_runs(&ar.runs, buf);
    }
}

// Sorted, non-overlapping [first, last] runs → gap-from-previous-end + length.
fn encode_runs(runs: &[(u32, u32)], buf: &mut Vec<u8>) {
    let mut prev = 0u64;
    for &(first, last) in runs {
        write_q(buf, first as u64 - prev);
        write_q(buf, (last - first) as u64);
        prev = last as u64 + 1;
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

// arcCount crossing arcs: each an arcIndex (delta-decoded) then its local runs.
fn decode_arc_runs(bytes: &[u8], pos: &mut usize, arc_count: u64) -> DecArcRuns {
    let mut out = Vec::with_capacity(arc_count as usize);
    let mut cum = 0u64;
    for _ in 0..arc_count {
        cum += read_q(bytes, pos);
        let rc = read_q(bytes, pos);
        out.push((cum, decode_runs(bytes, pos, rc)));
    }
    out
}

fn decode_runs(bytes: &[u8], pos: &mut usize, count: u64) -> Vec<(u64, u64)> {
    let mut runs = Vec::with_capacity(count as usize);
    let mut prev = 0u64;
    for _ in 0..count {
        let first = prev + read_q(bytes, pos);
        let last = first + read_q(bytes, pos);
        runs.push((first, last));
        prev = last + 1;
    }
    runs
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

// --- section 2: zones (polygon geometry) ---

// A signed coordinate via ZigZag + the exact varint below.
fn write_coord(buf: &mut Vec<u8>, v: i64) {
    write_uint_le(buf, zigzag(v));
}
fn read_coord(bytes: &[u8], pos: &mut usize) -> i64 {
    unzigzag(read_uint_le(bytes, pos))
}

// An exact unsigned value as minimal little-endian bytes behind a `q` byte-count.
//
// This is the zones section's "q-style variable-byte length": a `q` byte count
// followed by that many LE bytes.  Unlike a plain `q` (whose 3-byte form can only
// represent EVEN values ≥ 16512), it encodes any magnitude exactly — required
// for polygon byte-lengths, coordinates and areas, which are large and of
// arbitrary parity.
fn write_uint_le(buf: &mut Vec<u8>, mut v: u64) {
    let mut bytes = [0u8; 8];
    let mut n = 0;
    while v > 0 {
        bytes[n] = (v & 0xFF) as u8;
        v >>= 8;
        n += 1;
    }
    write_q(buf, n as u64);
    buf.extend_from_slice(&bytes[..n]);
}
fn read_uint_le(bytes: &[u8], pos: &mut usize) -> u64 {
    let n = read_q(bytes, pos) as usize;
    let mut v = 0u64;
    for i in 0..n {
        v |= (bytes[*pos + i] as u64) << (8 * i);
    }
    *pos += n;
    v
}

// Each arc as origin + packed delta stream (polycodec, no base64) behind an exact
// varint length.  These are pre-packed in `out.arc_packed`.
fn encode_arcs(out: &Output, buf: &mut Vec<u8>) {
    for (o, p) in &out.arc_packed {
        write_coord(buf, o[0]);
        write_coord(buf, o[1]);
        write_uint_le(buf, p.len() as u64);
        buf.extend_from_slice(p);
    }
}

// A ring's arc references: count, then each ref as (arcIndex << 1 | reversed).
fn encode_arc_refs(refs: &[ArcRef], buf: &mut Vec<u8>) {
    write_q(buf, refs.len() as u64);
    for r in refs {
        write_uint_le(buf, ((r.arc as u64) << 1) | r.rev as u64);
    }
}

fn encode_zones(out: &Output, order: &[usize], buf: &mut Vec<u8>) {
    for &old in order {
        let zone = &out.zones[old];
        write_q(buf, zone.tzid as u64);
        write_uint_le(buf, zone.a as u64);
        write_coord(buf, zone.aabb[0][0]);
        write_coord(buf, zone.aabb[0][1]);
        write_uint_le(buf, (zone.aabb[1][0] - zone.aabb[0][0]) as u64);
        write_uint_le(buf, (zone.aabb[1][1] - zone.aabb[0][1]) as u64);
        encode_arc_refs(&zone.outer_refs, buf);
        write_q(buf, zone.hole_refs.len() as u64);
        for hrefs in &zone.hole_refs {
            encode_arc_refs(hrefs, buf);
        }
    }
}

fn encode_tz_names(out: &Output, buf: &mut Vec<u8>) {
    write_q(buf, out.tz.len() as u64);
    for tz in &out.tz {
        write_q(buf, tz.n.len() as u64);
        buf.extend_from_slice(tz.n.as_bytes());
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
    for _ in 0..count {
        let ox = read_coord(bytes, pos);
        let oy = read_coord(bytes, pos);
        let plen = read_uint_le(bytes, pos) as usize;
        let p = bytes[*pos..*pos + plen].to_vec();
        *pos += plen;
        arcs.push(DecArc { o: (ox, oy), p });
    }
    arcs
}

fn decode_arc_refs(bytes: &[u8], pos: &mut usize) -> Vec<(u64, bool)> {
    let n = read_q(bytes, pos);
    (0..n)
        .map(|_| {
            let v = read_uint_le(bytes, pos);
            (v >> 1, v & 1 == 1)
        })
        .collect()
}

fn decode_zones(bytes: &[u8], pos: &mut usize, count: usize) -> Vec<DecZone> {
    let mut zones = Vec::with_capacity(count);
    for _ in 0..count {
        let tzid = read_q(bytes, pos);
        let area = read_uint_le(bytes, pos);
        let xl = read_coord(bytes, pos);
        let yl = read_coord(bytes, pos);
        let xh = xl + read_uint_le(bytes, pos) as i64;
        let yh = yl + read_uint_le(bytes, pos) as i64;
        let outer = decode_arc_refs(bytes, pos);
        let hn = read_q(bytes, pos);
        let holes = (0..hn).map(|_| decode_arc_refs(bytes, pos)).collect();
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

fn source_zones(out: &Output, order: &[usize]) -> Vec<DecZone> {
    order
        .iter()
        .map(|&old| {
            let z = &out.zones[old];
            DecZone {
                tzid: z.tzid as u64,
                area: z.a as u64,
                aabb: [z.aabb[0][0], z.aabb[0][1], z.aabb[1][0], z.aabb[1][1]],
                outer: refs_to_dec(&z.outer_refs),
                holes: z.hole_refs.iter().map(|h| refs_to_dec(h)).collect(),
            }
        })
        .collect()
}

fn decode_tz_names(bytes: &[u8], pos: &mut usize) -> Vec<String> {
    let n = read_q(bytes, pos);
    let mut names = Vec::with_capacity(n as usize);
    for _ in 0..n {
        let len = read_q(bytes, pos) as usize;
        names.push(String::from_utf8_lossy(&bytes[*pos..*pos + len]).into_owned());
        *pos += len;
    }
    names
}
