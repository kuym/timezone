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
//!   magic   "TZQ3"                              4 bytes
//!   section 1 — quadtree: one recursive node
//!       node := len:q
//!               len == 0 -> ocean leaf: no timezone; NOTHING follows (1 byte total)
//!               len == 1 -> leaf:       tzid:q follows
//!               len >= 2 -> internal:   `len` bytes = the 4 child nodes (quadrants 0..3)
//!   section 2 — tzid table:
//!       count:q, then count × ( nameLen:q, name:UTF-8 bytes )   — indexed by tzid
//! ```
//!
//! Open ocean is the single most common leaf value, so it gets a dedicated
//! 1-byte encoding (`len == 0`, no tzid) instead of a table slot.  The remaining
//! (real) tzids are **sorted by descending reference count** before encoding, so
//! the most-used timezone gets id 0 (form A, 1 byte).

use std::collections::HashMap;

use super::qvarint::{read_q, representable, write_q, Q_MAX};
use super::Serializer;
use crate::build::{classify, Output, Rel, Zone};
use crate::geom::{self, Aabb, Pt};
use crate::lookup::{cell_center, quadrant_for_point, segment_crossings, split_cell};
use crate::quant::{ROOT_CELL, X_SCALE, Y_SCALE};

const MAGIC: &[u8; 4] = b"TZQ3";
const QMAX_DEPTH: usize = 18;
const M_PER_DEG: f64 = 111_320.0;
const SAMPLES: i64 = 8;

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

        // Ocean is encoded specially (length-0 leaf, no tzid), so only the real
        // timezones go in the table.  Sort them by how many leaves reference
        // them, so the most common gets id 0 (1-byte form A).
        let ocean = none_id; // sentinel value stored in ocean leaves
        let mut refs = vec![0u64; out.tz.len()];
        count_refs(&root, &mut refs, ocean);
        let mut order: Vec<usize> = (0..out.tz.len()).collect();
        order.sort_by(|&x, &y| refs[y].cmp(&refs[x]).then(x.cmp(&y)));
        // remap[old_id] = new rank (ocean leaves keep the sentinel unchanged)
        let mut remap = vec![0u64; out.tz.len()];
        for (rank, &old) in order.iter().enumerate() {
            remap[old] = rank as u64;
        }
        remap_leaves(&mut root, &remap, ocean);
        let names: Vec<&str> = order.iter().map(|&old| out.tz[old].n.as_str()).collect();

        eprintln!(
            "quad: {} tzids in form A (1-byte), {} in form B; ocean is length-0",
            order.iter().take(128).filter(|&&o| refs[o] > 0).count(),
            order.iter().skip(128).filter(|&&o| refs[o] > 0).count()
        );

        // Self-check against exact brute-force lookup (misses are near borders).
        let agree = self_verify(out, &root, &names, ocean, 4000);
        eprintln!("quad: {agree:.2}% of 4000 random points match exact lookup (rest are near borders)");

        // Encode.
        let mut buf = Vec::new();
        buf.extend_from_slice(MAGIC);
        encode_node(&root, &mut buf, ocean)?;
        write_q(&mut buf, names.len() as u64);
        for name in &names {
            write_q(&mut buf, name.len() as u64);
            buf.extend_from_slice(name.as_bytes());
        }

        verify_roundtrip(&buf, &root, ocean)?;
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

// Count leaf references per real tzid (ocean leaves are excluded — ocean has no
// table entry).
fn count_refs(node: &QNode, refs: &mut [u64], ocean: u64) {
    match node {
        QNode::Leaf(t) => {
            if *t != ocean {
                refs[*t as usize] += 1;
            }
        }
        QNode::Internal(ch) => ch.iter().for_each(|c| count_refs(c, refs, ocean)),
    }
}

fn remap_leaves(node: &mut QNode, remap: &[u64], ocean: u64) {
    match node {
        QNode::Leaf(t) => {
            if *t != ocean {
                *t = remap[*t as usize];
            }
        }
        QNode::Internal(ch) => ch.iter_mut().for_each(|c| remap_leaves(c, remap, ocean)),
    }
}

// --- encoding ---

fn encode_node(node: &QNode, buf: &mut Vec<u8>, ocean: u64) -> Result<(), String> {
    match node {
        QNode::Leaf(tzid) => {
            if *tzid == ocean {
                write_q(buf, 0); // ocean leaf: length 0, no tzid follows
            } else {
                write_q(buf, 1); // non-ocean leaf: length 1, then the tzid
                write_q(buf, *tzid);
            }
        }
        QNode::Internal(children) => {
            let mut payload = Vec::new();
            for c in children.iter() {
                encode_node(c, &mut payload, ocean)?;
            }
            // Lengths 0 and 1 are leaf markers; an internal payload is at least 4
            // bytes (four 1-byte ocean children), so `len` never collides.
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
        // ocean leaf: nothing follows
    } else if len == 1 {
        read_q(bytes, pos); // skip tzid
    } else {
        *pos += len as usize; // skip payload (incl. padding)
    }
}

fn lookup_encoded(bytes: &[u8], point: Pt, ocean: u64) -> u64 {
    let mut pos = 4; // past magic
    let mut cell = ROOT_CELL;
    loop {
        let len = read_q(bytes, &mut pos);
        if len == 0 {
            return ocean;
        }
        if len == 1 {
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

fn verify_roundtrip(bytes: &[u8], root: &QNode, ocean: u64) -> Result<(), String> {
    let mut seed = 0x9E3779B97F4A7C15u64;
    for _ in 0..2000 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x = ROOT_CELL[0][0] + ((seed >> 33) as i64 % (ROOT_CELL[1][0] - ROOT_CELL[0][0]));
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y = ROOT_CELL[0][1] + ((seed >> 33) as i64 % (ROOT_CELL[1][1] - ROOT_CELL[0][1]));
        let p = [x, y];
        let a = lookup_tree(root, &ROOT_CELL, p);
        let b = lookup_encoded(bytes, p, ocean);
        if a != b {
            return Err(format!("quad encode round-trip mismatch at {p:?}: tree {a} != encoded {b}"));
        }
    }
    Ok(())
}

fn self_verify(out: &Output, root: &QNode, names: &[&str], ocean: u64, samples: usize) -> f64 {
    let mut seed = 0xD1B54A32D192ED03u64;
    let mut agree = 0usize;
    for _ in 0..samples {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x = ROOT_CELL[0][0] + ((seed >> 33) as i64 % (ROOT_CELL[1][0] - ROOT_CELL[0][0]));
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y = ROOT_CELL[0][1] + ((seed >> 33) as i64 % (ROOT_CELL[1][1] - ROOT_CELL[0][1]));
        let p = [x, y];

        let v = lookup_tree(root, &ROOT_CELL, p);
        let rough = if v == ocean { "" } else { names[v as usize] };
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
    fn encode_decode_tree() {
        let ocean = 5000u64;
        let tree = QNode::Internal(Box::new([
            QNode::Leaf(3),
            // NW child holds an ocean leaf (length-0, 1 byte) among others.
            QNode::Internal(Box::new([QNode::Leaf(1), QNode::Leaf(ocean), QNode::Leaf(2), QNode::Leaf(0)])),
            QNode::Leaf(7),
            QNode::Leaf(200), // form B tzid
        ]));
        let mut buf = Vec::new();
        buf.extend_from_slice(MAGIC);
        encode_node(&tree, &mut buf, ocean).unwrap();
        assert_eq!(lookup_encoded(&buf, [ROOT_CELL[1][0] - 1, ROOT_CELL[1][1] - 1], ocean), 3);
        assert_eq!(lookup_encoded(&buf, [ROOT_CELL[0][0], ROOT_CELL[0][1]], ocean), 7);
        for p in [[100, 100], [-100, -100], [500, -9], [ROOT_CELL[0][0], ROOT_CELL[1][1] - 1]] {
            assert_eq!(lookup_tree(&tree, &ROOT_CELL, p), lookup_encoded(&buf, p, ocean));
        }
    }
}
