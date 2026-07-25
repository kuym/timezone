//! Topological shared-arc encoding (the TopoJSON model) + Visvalingam–Whyatt
//! simplification.
//!
//! Adjacent timezone polygons share their common border, so ~80% of edges are
//! stored twice.  This module factors the rings into **arcs** — maximal vertex
//! runs bounded by *junctions* (points where the sharing pattern changes) — so
//! each shared border is stored once.  A ring becomes a list of signed arc
//! references (a reversed arc reuses the same bytes traversed backwards).
//!
//! Reconstruction is exact: `reconstruct(build(rings))` yields each ring back
//! (as a cyclic rotation — winding and geometry preserved), so the encoding is
//! lossless.  The optional lossy layer is per-arc Visvalingam–Whyatt: drop the
//! interior vertices with the smallest effective (triangle) area, keeping arc
//! endpoints fixed so shared borders stay shared and stitch seamlessly.

use crate::geom::Pt;
use std::collections::{HashMap, HashSet};

/// A signed reference from a ring to an arc.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArcRef {
    pub arc: usize,
    pub rev: bool,
}

/// Extract shared arcs from a set of rings (each a vertex list without a repeated
/// closing vertex).  Returns the deduplicated arcs and, per input ring, its arc
/// references in traversal order.
pub fn build(rings: &[Vec<Pt>]) -> (Vec<Vec<Pt>>, Vec<Vec<ArcRef>>) {
    let junc = junctions(rings);

    let mut arcs: Vec<Vec<Pt>> = Vec::new();
    let mut index: HashMap<Vec<Pt>, usize> = HashMap::new();
    let mut ring_refs: Vec<Vec<ArcRef>> = Vec::with_capacity(rings.len());

    for ring in rings {
        let mut refs = Vec::new();
        for arc in cut_ring(ring, &junc) {
            refs.push(dedup_arc(arc, &mut arcs, &mut index));
        }
        ring_refs.push(refs);
    }
    (arcs, ring_refs)
}

/// Rebuild a ring (no repeated closing vertex) from its arc references.
pub fn reconstruct(arcs: &[Vec<Pt>], refs: &[ArcRef]) -> Vec<Pt> {
    let mut ring = Vec::new();
    for (k, r) in refs.iter().enumerate() {
        let arc = &arcs[r.arc];
        let start = usize::from(k > 0); // drop the shared junction with the previous arc
        if r.rev {
            for i in (0..arc.len().saturating_sub(start)).rev() {
                ring.push(arc[i]);
            }
        } else {
            ring.extend_from_slice(&arc[start..]);
        }
    }
    ring.pop(); // the concatenation ends back at the start junction (the closure)
    ring
}

/// Visvalingam–Whyatt on one arc: keep the two endpoints, drop interior vertices
/// by ascending effective area until `keep_fraction` of the vertices remain
/// (1.0 = untouched).  A closed arc (a ring with no junctions) keeps ≥4 vertices
/// so the reconstructed ring stays valid; an open arc keeps ≥2 (its endpoints).
pub fn vw_simplify(arc: &[Pt], keep_fraction: f64) -> Vec<Pt> {
    let n = arc.len();
    if n <= 2 || keep_fraction >= 1.0 {
        return arc.to_vec();
    }
    let min_keep = if arc[0] == arc[n - 1] { 4 } else { 2 };
    let mut target = (n as f64 * keep_fraction).round() as usize;
    target = target.clamp(min_keep, n);
    if target >= n {
        return arc.to_vec();
    }
    let mut pts = arc.to_vec();
    while pts.len() > target {
        let mut min_area = i64::MAX;
        let mut min_i = 0usize;
        for i in 1..pts.len() - 1 {
            let a = tri_area2(pts[i - 1], pts[i], pts[i + 1]);
            if a < min_area {
                min_area = a;
                min_i = i;
            }
        }
        if min_i == 0 {
            break;
        }
        pts.remove(min_i);
    }
    pts
}

// Twice the area of triangle (a, b, c) — the VW effective area of vertex b.
fn tri_area2(a: Pt, b: Pt, c: Pt) -> i64 {
    ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs()
}

// A point is a junction if, across all its occurrences, it is not always flanked
// by the same unordered pair of neighbours — i.e. the shared boundary through it
// diverges (a triple point, a coastline end of a shared border, a touch point).
fn junctions(rings: &[Vec<Pt>]) -> HashSet<Pt> {
    let mut seen: HashMap<Pt, Option<(Pt, Pt)>> = HashMap::new();
    let mut junc: HashSet<Pt> = HashSet::new();
    for ring in rings {
        let n = ring.len();
        if n == 0 {
            continue;
        }
        for i in 0..n {
            let cur = ring[i];
            let prev = ring[(i + n - 1) % n];
            let next = ring[(i + 1) % n];
            let pair = if prev <= next { (prev, next) } else { (next, prev) };
            match seen.get(&cur) {
                None => {
                    seen.insert(cur, Some(pair));
                }
                Some(None) => {} // already a junction
                Some(Some(existing)) => {
                    if *existing != pair {
                        seen.insert(cur, None);
                        junc.insert(cur);
                    }
                }
            }
        }
    }
    junc
}

// Cut a ring at its junctions into arcs (each including both endpoint junctions).
// A ring with no junctions becomes one closed arc [p0 .. p_{n-1}, p0].
fn cut_ring(ring: &[Pt], junc: &HashSet<Pt>) -> Vec<Vec<Pt>> {
    let n = ring.len();
    let jpos: Vec<usize> = (0..n).filter(|&i| junc.contains(&ring[i])).collect();
    if jpos.is_empty() {
        let mut arc = ring.to_vec();
        arc.push(ring[0]);
        return vec![arc];
    }
    let mut arcs = Vec::with_capacity(jpos.len());
    for k in 0..jpos.len() {
        let start = jpos[k];
        let end = jpos[(k + 1) % jpos.len()];
        let mut arc = Vec::new();
        let mut i = start;
        loop {
            arc.push(ring[i]);
            // `arc.len() > 1` so a single-junction ring (start == end) traverses
            // the whole loop back to the junction instead of stopping on the very
            // first vertex (which would yield a degenerate 1-vertex arc).
            if i == end && arc.len() > 1 {
                break;
            }
            i = (i + 1) % n;
        }
        arcs.push(arc);
    }
    arcs
}

// Add an arc (or find it, forward or reversed) and return a reference to it.
fn dedup_arc(arc: Vec<Pt>, arcs: &mut Vec<Vec<Pt>>, index: &mut HashMap<Vec<Pt>, usize>) -> ArcRef {
    if let Some(&i) = index.get(&arc) {
        return ArcRef { arc: i, rev: false };
    }
    let mut rev = arc.clone();
    rev.reverse();
    if let Some(&i) = index.get(&rev) {
        return ArcRef { arc: i, rev: true };
    }
    let i = arcs.len();
    index.insert(arc.clone(), i);
    arcs.push(arc);
    ArcRef { arc: i, rev: false }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Is `b` a cyclic rotation of `a`?  Reconstruction preserves geometry but may
    // start the ring at a different (junction) vertex.
    fn is_rotation(a: &[Pt], b: &[Pt]) -> bool {
        let n = a.len();
        if b.len() != n {
            return false;
        }
        (0..n).any(|s| (0..n).all(|i| a[i] == b[(i + s) % n]))
    }

    #[test]
    fn roundtrip_isolated_ring() {
        let ring = vec![[0, 0], [10, 0], [10, 10], [5, 15], [0, 10]];
        let (arcs, refs) = build(&[ring.clone()]);
        assert!(is_rotation(&ring, &reconstruct(&arcs, &refs[0])));
    }

    #[test]
    fn shared_border_is_one_arc() {
        // Two squares sharing the vertical edge x=10 (vertices (10,0) and (10,10)).
        let a = vec![[0, 0], [10, 0], [10, 10], [0, 10]];
        let b = vec![[10, 0], [20, 0], [20, 10], [10, 10]];
        let (arcs, refs) = build(&[a.clone(), b.clone()]);
        assert!(is_rotation(&a, &reconstruct(&arcs, &refs[0])));
        assert!(is_rotation(&b, &reconstruct(&arcs, &refs[1])));
        // The shared border is stored once: some arc is referenced by both rings
        // (once forward, once reversed), so there are fewer arcs than arc-uses.
        let uses: usize = refs.iter().map(|r| r.len()).sum();
        assert!(arcs.len() < uses, "the shared border should be a single deduped arc");
        let shared = refs[0].iter().find(|r| refs[1].iter().any(|s| s.arc == r.arc)).unwrap();
        assert!(refs[1].iter().any(|s| s.arc == shared.arc && s.rev != shared.rev));
    }

    #[test]
    fn single_junction_ring_survives() {
        // Two rings touching at exactly one shared vertex (10,0): that vertex is
        // the only junction of each ring.  Both must still reconstruct in full.
        let a = vec![[0, 0], [10, 0], [5, 8]];
        let b = vec![[10, 0], [20, 0], [15, 8]];
        let (arcs, refs) = build(&[a.clone(), b.clone()]);
        let ra = reconstruct(&arcs, &refs[0]);
        let rb = reconstruct(&arcs, &refs[1]);
        assert_eq!(ra.len(), a.len(), "single-junction ring A lost vertices");
        assert_eq!(rb.len(), b.len(), "single-junction ring B lost vertices");
        assert!(is_rotation(&a, &ra));
        assert!(is_rotation(&b, &rb));
    }

    #[test]
    fn vw_keeps_endpoints_and_reduces() {
        let arc = vec![[0, 0], [10, 1], [20, 0], [30, 1], [40, 0]];
        let s = vw_simplify(&arc, 0.5);
        assert!(s.len() < arc.len());
        assert_eq!(s[0], arc[0]);
        assert_eq!(*s.last().unwrap(), *arc.last().unwrap());
    }
}
