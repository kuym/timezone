//! Experimental quadtree-only binary serializer.
//!
//! Emits a compact quadtree with NO polygon geometry: every leaf resolves to
//! exactly one timezone id (tzid).  A cell stops splitting when it is
//! homogeneous (one tzid covers it) or when its longest edge falls below a
//! configurable length in metres (`--leaf-km`, default 10 km); at that limit the
//! tzid with the most area in the cell wins.
//!
//! ## Integer encoding (`q`)
//!
//! Every integer (node length, tzid, table entry) is written with a 3-form,
//! self-describing code selected by the top bits of the first byte.  The encoded
//! quantity `q` relates to the raw field `a` as:
//!
//! ```text
//!   A  0aaaaaaa                            q = a           →   0 ..   127   (1 byte)
//!   B  10aaaaaa aaaaaaaa                   q = a + 128      → 128 .. 16511   (2 bytes)
//!   C  11aaaaaa aaaaaaaa aaaaaaaa          q = 2a + 16514   → even, 16514.. (3 bytes)
//! ```
//!
//! Equivalently the "value" `(q + 1) * 2` is `(a+1)*2` / `a*2+258` / `a*4+33030`.
//! `q` is exact below 16512 (so tzids, name lengths and most node lengths pad
//! nothing); only node payloads ≥ 16512 bytes round up to the next even `q`.
//! Node lengths may reach form C; tzids never exceed a couple hundred, so they
//! only ever use A or B.  The maximum encodable `q` is 8_404_120, which caps the
//! whole tree at ~8 MB — reduce resolution (larger `--leaf-km`) if exceeded.
//!
//! ## File layout
//!
//! ```text
//!   magic   "TZQ2"                              4 bytes
//!   section 1 — quadtree: one recursive node
//!       node := len:q
//!               len == 0 -> leaf:     tzid:q
//!               len  > 0 -> internal: `len` bytes = the 4 child nodes (quadrants 0..3)
//!   section 2 — tzid table:
//!       count:q, then count × ( nameLen:q, name:UTF-8 bytes )   — indexed by tzid
//! ```
//!
//! tzids are **sorted by descending reference count** before encoding, so the
//! most-used timezones get the lowest ids (form A, 1 byte).  A table entry with
//! an empty name is "no timezone" (open ocean).

use std::collections::HashMap;

use super::Serializer;
use crate::build::{classify, Output, Rel, Zone};
use crate::geom::{self, Aabb, Pt};
use crate::lookup::{cell_center, quadrant_for_point, segment_crossings, split_cell};
use crate::quant::{ROOT_CELL, X_SCALE, Y_SCALE};

const MAGIC: &[u8; 4] = b"TZQ2";
const QMAX_DEPTH: usize = 18;
const M_PER_DEG: f64 = 111_320.0;
const SAMPLES: i64 = 8;

// Largest `q` the 3-byte C form can represent (2 * (2^22 - 1) + 16514).
const Q_MAX: u64 = 2 * 0x3F_FFFF + 16514;

pub struct QuadtreeSerializer {
    pub leaf_meters: f64,
}

impl Serializer for QuadtreeSerializer {
    fn serialize(&self, out: &Output) -> Result<Vec<u8>, String> {
        let none_id = out.tz.len() as u64;
        let overlapping: Vec<usize> = (0..out.zones.len()).collect();

        eprintln!("quad: building rough tree (leaf edge < {:.0} m) ...", self.leaf_meters);
        let mut root = build_node(&out.zones, &ROOT_CELL, 0, &overlapping, self.leaf_meters, none_id);

        let (nodes, leaves) = count(&root);
        eprintln!("quad: {nodes} nodes, {leaves} leaves");

        // Sort tzids by how many leaves reference them, so the most common get
        // the lowest ids (1-byte form A).  `none` (ocean) participates too.
        let ntz = out.tz.len() + 1; // +1 for "none"
        let mut refs = vec![0u64; ntz];
        count_refs(&root, &mut refs);
        let mut order: Vec<usize> = (0..ntz).collect();
        order.sort_by(|&x, &y| refs[y].cmp(&refs[x]).then(x.cmp(&y)));
        // remap[old_id] = new rank
        let mut remap = vec![0u64; ntz];
        for (rank, &old) in order.iter().enumerate() {
            remap[old] = rank as u64;
        }
        remap_leaves(&mut root, &remap);
        // names[rank] — empty for "none"
        let names: Vec<&str> = order
            .iter()
            .map(|&old| if old as u64 == none_id { "" } else { out.tz[old].n.as_str() })
            .collect();

        eprintln!(
            "quad: {} tzids used in form A (1-byte), {} in form B; ocean rank {}",
            order.iter().take(128).filter(|&&o| refs[o] > 0).count(),
            order.iter().skip(128).filter(|&&o| refs[o] > 0).count(),
            remap[none_id as usize]
        );

        // Self-check against exact brute-force lookup (misses are near borders).
        let agree = self_verify(out, &root, &names, 4000);
        eprintln!("quad: {agree:.2}% of 4000 random points match exact lookup (rest are near borders)");

        // Encode.
        let mut buf = Vec::new();
        buf.extend_from_slice(MAGIC);
        encode_node(&root, &mut buf)?;
        write_q(&mut buf, names.len() as u64);
        for name in &names {
            write_q(&mut buf, name.len() as u64);
            buf.extend_from_slice(name.as_bytes());
        }

        verify_roundtrip(&buf, &root)?;
        Ok(buf)
    }

    fn format_name(&self) -> &'static str {
        "quad"
    }

    fn uses_cost_tree(&self) -> bool {
        false
    }
}

// --- the rough quadtree (unchanged logic; leaves hold original tzids) ---

enum QNode {
    Leaf(u64),
    Internal(Box<[QNode; 4]>),
}

fn build_node(
    zones: &[Zone],
    cell: &Aabb,
    depth: usize,
    overlapping: &[usize],
    leaf_m: f64,
    none_id: u64,
) -> QNode {
    let mut candidates: Vec<usize> = Vec::new();
    let mut cover: Vec<usize> = Vec::new();
    for &zi in overlapping {
        match classify(cell, &zones[zi]) {
            Rel::Disjoint => {}
            Rel::Candidate => candidates.push(zi),
            Rel::Definite => cover.push(zi),
        }
    }

    if candidates.is_empty() {
        return QNode::Leaf(uniform_tzid(zones, &cover, none_id));
    }

    if depth < QMAX_DEPTH && longest_edge_m(cell) >= leaf_m {
        let mut child_overlap = candidates.clone();
        child_overlap.extend_from_slice(&cover);
        let children: [QNode; 4] = [
            build_node(zones, &split_cell(cell, 0), depth + 1, &child_overlap, leaf_m, none_id),
            build_node(zones, &split_cell(cell, 1), depth + 1, &child_overlap, leaf_m, none_id),
            build_node(zones, &split_cell(cell, 2), depth + 1, &child_overlap, leaf_m, none_id),
            build_node(zones, &split_cell(cell, 3), depth + 1, &child_overlap, leaf_m, none_id),
        ];
        if let (QNode::Leaf(a), QNode::Leaf(b), QNode::Leaf(c), QNode::Leaf(d)) =
            (&children[0], &children[1], &children[2], &children[3])
        {
            if a == b && b == c && c == d {
                return QNode::Leaf(*a);
            }
        }
        QNode::Internal(Box::new(children))
    } else {
        QNode::Leaf(majority_tzid(zones, cell, &candidates, &cover, none_id))
    }
}

fn uniform_tzid(zones: &[Zone], cover: &[usize], none_id: u64) -> u64 {
    let mut best: Option<usize> = None;
    for &zi in cover {
        if best.map_or(true, |b| zones[zi].a < zones[b].a) {
            best = Some(zi);
        }
    }
    best.map_or(none_id, |b| zones[b].tzid as u64)
}

struct Local {
    zi: usize,
    w: i64,
    e: Vec<(u32, u32)>,
    holes: Vec<(usize, i64, Vec<(u32, u32)>)>,
}

fn build_local(zone: &Zone, zi: usize, cell: &Aabb, center: Pt) -> Local {
    let mut holes = Vec::new();
    for (hi, hole) in zone.holes.iter().enumerate() {
        if !geom::aabbs_overlap(cell, &zone.hole_aabbs[hi]) {
            continue;
        }
        let runs = edge_runs(hole, cell);
        if runs.is_empty() {
            continue;
        }
        holes.push((hi, geom::poly_winding(hole, center), runs));
    }
    Local { zi, w: geom::poly_winding(&zone.outer, center), e: edge_runs(&zone.outer, cell), holes }
}

fn local_contains(local: &Local, zone: &Zone, center: Pt, point: Pt) -> bool {
    match segment_crossings(center, point, &zone.outer, &local.e) {
        None => return geom::rings_contain_point(&zone.outer, &zone.holes, point),
        Some(d) => {
            if local.w + d == 0 {
                return false;
            }
        }
    }
    for (hi, hw, runs) in &local.holes {
        match segment_crossings(center, point, &zone.holes[*hi], runs) {
            None => return geom::rings_contain_point(&zone.outer, &zone.holes, point),
            Some(d) => {
                if hw + d != 0 {
                    return false;
                }
            }
        }
    }
    true
}

fn majority_tzid(zones: &[Zone], cell: &Aabb, candidates: &[usize], cover: &[usize], none_id: u64) -> u64 {
    let center = cell_center(cell);
    let locals: Vec<Local> =
        candidates.iter().map(|&zi| build_local(&zones[zi], zi, cell, center)).collect();

    let mut counts: HashMap<u64, u32> = HashMap::new();
    for gy in 0..SAMPLES {
        for gx in 0..SAMPLES {
            let px = cell[0][0] + (cell[1][0] - cell[0][0]) * (2 * gx + 1) / (2 * SAMPLES);
            let py = cell[0][1] + (cell[1][1] - cell[0][1]) * (2 * gy + 1) / (2 * SAMPLES);
            let p = [px, py];

            let mut best: Option<usize> = None;
            for &zi in cover {
                if best.map_or(true, |b| zones[zi].a < zones[b].a) {
                    best = Some(zi);
                }
            }
            for local in &locals {
                if local_contains(local, &zones[local.zi], center, p)
                    && best.map_or(true, |b| zones[local.zi].a < zones[b].a)
                {
                    best = Some(local.zi);
                }
            }
            let t = best.map_or(none_id, |b| zones[b].tzid as u64);
            *counts.entry(t).or_insert(0) += 1;
        }
    }

    let mut winner = none_id;
    let mut best_count = 0u32;
    let mut keys: Vec<u64> = counts.keys().cloned().collect();
    keys.sort_unstable();
    for t in keys {
        let c = counts[&t];
        if c > best_count {
            best_count = c;
            winner = t;
        }
    }
    winner
}

fn longest_edge_m(cell: &Aabb) -> f64 {
    let h_units = (cell[1][1] - cell[0][1]) as f64;
    let w_units = (cell[1][0] - cell[0][0]) as f64;
    let h_deg = h_units / Y_SCALE;
    let w_deg = w_units / X_SCALE;
    let center_lat_deg = ((cell[0][1] + cell[1][1]) as f64 / 2.0) / Y_SCALE;
    let h_m = h_deg * M_PER_DEG;
    let w_m = w_deg * M_PER_DEG * center_lat_deg.to_radians().cos().abs();
    h_m.max(w_m)
}

fn edge_runs(ring: &[Pt], cell: &Aabb) -> Vec<(u32, u32)> {
    let mut runs: Vec<(u32, u32)> = Vec::new();
    let n = ring.len();
    for i in 0..n {
        if geom::edge_touches_aabb(cell, ring[i], ring[(i + 1) % n]) {
            let iu = i as u32;
            if let Some(last) = runs.last_mut() {
                if last.1 + 1 == iu {
                    last.1 = iu;
                    continue;
                }
            }
            runs.push((iu, iu));
        }
    }
    runs
}

fn count(node: &QNode) -> (usize, usize) {
    match node {
        QNode::Leaf(_) => (1, 1),
        QNode::Internal(ch) => {
            let (mut n, mut l) = (1usize, 0usize);
            for c in ch.iter() {
                let (cn, cl) = count(c);
                n += cn;
                l += cl;
            }
            (n, l)
        }
    }
}

fn count_refs(node: &QNode, refs: &mut [u64]) {
    match node {
        QNode::Leaf(t) => refs[*t as usize] += 1,
        QNode::Internal(ch) => ch.iter().for_each(|c| count_refs(c, refs)),
    }
}

fn remap_leaves(node: &mut QNode, remap: &[u64]) {
    match node {
        QNode::Leaf(t) => *t = remap[*t as usize],
        QNode::Internal(ch) => ch.iter_mut().for_each(|c| remap_leaves(c, remap)),
    }
}

// --- the `q` integer code ---

/// Smallest representable `q` >= n (identity below 16512; rounds up to an even
/// value in the form-C range).
fn representable(n: u64) -> u64 {
    if n <= 16511 {
        n
    } else {
        let r = n.max(16514);
        r + (r & 1) // make even
    }
}

fn write_q(buf: &mut Vec<u8>, q: u64) {
    if q <= 127 {
        buf.push(q as u8); // A: 0aaaaaaa
    } else if q <= 16511 {
        let a = q - 128; // 14 bits
        buf.push(0x80 | (a >> 8) as u8); // B: 10aaaaaa aaaaaaaa
        buf.push((a & 0xFF) as u8);
    } else {
        // C: even q only
        let a = (q - 16514) / 2; // 22 bits
        buf.push(0xC0 | (a >> 16) as u8);
        buf.push(((a >> 8) & 0xFF) as u8);
        buf.push((a & 0xFF) as u8);
    }
}

fn read_q(bytes: &[u8], pos: &mut usize) -> u64 {
    let b0 = bytes[*pos] as u64;
    *pos += 1;
    if b0 < 0x80 {
        b0
    } else if b0 < 0xC0 {
        let a = ((b0 & 0x3F) << 8) | bytes[*pos] as u64;
        *pos += 1;
        a + 128
    } else {
        let a = ((b0 & 0x3F) << 16) | ((bytes[*pos] as u64) << 8) | bytes[*pos + 1] as u64;
        *pos += 2;
        2 * a + 16514
    }
}

// --- encoding ---

fn encode_node(node: &QNode, buf: &mut Vec<u8>) -> Result<(), String> {
    match node {
        QNode::Leaf(tzid) => {
            write_q(buf, 0); // leaf marker (length 0)
            write_q(buf, *tzid);
        }
        QNode::Internal(children) => {
            let mut payload = Vec::new();
            for c in children.iter() {
                encode_node(c, &mut payload)?;
            }
            let len = representable(payload.len() as u64);
            if len > Q_MAX {
                return Err(format!(
                    "quad: node payload {} bytes exceeds the format's {}-byte limit; \
                     use a larger --leaf-km",
                    payload.len(),
                    Q_MAX
                ));
            }
            write_q(buf, len);
            buf.extend_from_slice(&payload);
            buf.resize(buf.len() + (len as usize - payload.len()), 0); // pad
        }
    }
    Ok(())
}

// --- decode / lookup (round-trip verification and reader reference) ---

fn skip_node(bytes: &[u8], pos: &mut usize) {
    let len = read_q(bytes, pos);
    if len == 0 {
        read_q(bytes, pos); // skip tzid
    } else {
        *pos += len as usize; // skip payload (incl. padding)
    }
}

fn lookup_encoded(bytes: &[u8], point: Pt) -> u64 {
    let mut pos = 4; // past magic
    let mut cell = ROOT_CELL;
    loop {
        let len = read_q(bytes, &mut pos);
        if len == 0 {
            return read_q(bytes, &mut pos);
        }
        let q = quadrant_for_point(&cell, point);
        for _ in 0..q {
            skip_node(bytes, &mut pos);
        }
        cell = split_cell(&cell, q);
    }
}

fn lookup_tree(node: &QNode, cell: &Aabb, point: Pt) -> u64 {
    match node {
        QNode::Leaf(t) => *t,
        QNode::Internal(children) => {
            let q = quadrant_for_point(cell, point);
            lookup_tree(&children[q], &split_cell(cell, q), point)
        }
    }
}

fn verify_roundtrip(bytes: &[u8], root: &QNode) -> Result<(), String> {
    let mut seed = 0x9E3779B97F4A7C15u64;
    for _ in 0..2000 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x = ROOT_CELL[0][0] + ((seed >> 33) as i64 % (ROOT_CELL[1][0] - ROOT_CELL[0][0]));
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y = ROOT_CELL[0][1] + ((seed >> 33) as i64 % (ROOT_CELL[1][1] - ROOT_CELL[0][1]));
        let p = [x, y];
        let a = lookup_tree(root, &ROOT_CELL, p);
        let b = lookup_encoded(bytes, p);
        if a != b {
            return Err(format!("quad encode round-trip mismatch at {p:?}: tree {a} != encoded {b}"));
        }
    }
    Ok(())
}

fn self_verify(out: &Output, root: &QNode, names: &[&str], samples: usize) -> f64 {
    let mut seed = 0xD1B54A32D192ED03u64;
    let mut agree = 0usize;
    for _ in 0..samples {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x = ROOT_CELL[0][0] + ((seed >> 33) as i64 % (ROOT_CELL[1][0] - ROOT_CELL[0][0]));
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y = ROOT_CELL[0][1] + ((seed >> 33) as i64 % (ROOT_CELL[1][1] - ROOT_CELL[0][1]));
        let p = [x, y];

        let rough = names[lookup_tree(root, &ROOT_CELL, p) as usize];
        let mut best: Option<usize> = None;
        for z in &out.zones {
            if geom::rings_contain_point(&z.outer, &z.holes, p)
                && best.map_or(true, |b| z.a < out.zones[b].a)
            {
                best = Some(z.id);
            }
        }
        let exact = best.map_or("", |b| out.tz[out.zones[b].tzid].n.as_str());
        if rough == exact {
            agree += 1;
        }
    }
    100.0 * agree as f64 / samples as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q_roundtrip() {
        for q in [0u64, 1, 127, 128, 129, 16511, 16514, 16516, 100000, Q_MAX] {
            let mut buf = Vec::new();
            write_q(&mut buf, q);
            let mut pos = 0;
            assert_eq!(read_q(&buf, &mut pos), q, "q={q}");
            assert_eq!(pos, buf.len());
        }
    }

    #[test]
    fn q_form_sizes() {
        // A is 1 byte (q<=127), B is 2 (q<=16511), C is 3.
        let sz = |q| {
            let mut b = Vec::new();
            write_q(&mut b, q);
            b.len()
        };
        assert_eq!(sz(0), 1);
        assert_eq!(sz(127), 1);
        assert_eq!(sz(128), 2);
        assert_eq!(sz(16511), 2);
        assert_eq!(sz(16514), 3);
    }

    #[test]
    fn representable_is_exact_below_c() {
        for n in [0u64, 1, 8, 127, 128, 16511] {
            assert_eq!(representable(n), n);
        }
        assert_eq!(representable(16512), 16514);
        assert_eq!(representable(16515), 16516);
    }

    #[test]
    fn encode_decode_tree() {
        let tree = QNode::Internal(Box::new([
            QNode::Leaf(3),
            QNode::Internal(Box::new([QNode::Leaf(1), QNode::Leaf(1), QNode::Leaf(2), QNode::Leaf(0)])),
            QNode::Leaf(7),
            QNode::Leaf(200), // form B tzid
        ]));
        let mut buf = Vec::new();
        buf.extend_from_slice(MAGIC);
        encode_node(&tree, &mut buf).unwrap();
        assert_eq!(lookup_encoded(&buf, [ROOT_CELL[1][0] - 1, ROOT_CELL[1][1] - 1]), 3);
        assert_eq!(lookup_encoded(&buf, [ROOT_CELL[0][0], ROOT_CELL[0][1]]), 7);
        for p in [[100, 100], [-100, -100], [500, -9], [ROOT_CELL[0][0], ROOT_CELL[1][1] - 1]] {
            assert_eq!(lookup_tree(&tree, &ROOT_CELL, p), lookup_encoded(&buf, p));
        }
    }
}
