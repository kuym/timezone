//! Quantization domain and polygon simplification — port of tzmap.js.

use crate::geom::{self, Pt};

// The quantization domain: half-open `[-2^19, 2^19) × [-2^18, 2^18)`, both axes
// at the same scale so cells are square and every cell midpoint is exact.
pub const X_MIN: i64 = -524288;
pub const X_MAX: i64 = 524288; // 2^19
pub const Y_MIN: i64 = -262144;
pub const Y_MAX: i64 = 262144; // 2^18
pub const X_SCALE: f64 = 524288.0 / 180.0;
pub const Y_SCALE: f64 = 262144.0 / 90.0;

/// Root cell of the quadtree, half-open on both axes.
pub const ROOT_CELL: geom::Aabb = [[X_MIN, Y_MIN], [X_MAX, Y_MAX]];

/// Maximum subdivision depth (keeps the narrow axis ≥ 2 units, so midpoints stay
/// exact).  The subdivider stops here even if a leaf is still over budget.
pub const MAX_DEPTH: usize = 16;

// JavaScript's Math.round is floor(x + 0.5) (round half toward +∞), which
// differs from Rust's f64::round (round half away from zero) on negative .5
// cases.  Match the reference implementation exactly.
#[inline]
fn js_round(x: f64) -> i64 {
    (x + 0.5).floor() as i64
}

#[inline]
fn clamp(v: i64, lo: i64, hi: i64) -> i64 {
    v.max(lo).min(hi)
}

/// Convert `[longitude, latitude]` degree pairs to fixed-point integer vertices:
/// round (half toward +∞, like JS), clamp into the domain, drop consecutive
/// duplicates, and drop the GeoJSON repeated closing vertex (closure is
/// implicit everywhere in this codebase).
pub fn quantize(lonlat: &[[f64; 2]]) -> Vec<Pt> {
    let mut pts: Vec<Pt> = Vec::with_capacity(lonlat.len());
    for v in lonlat {
        let x = clamp(js_round(X_SCALE * v[0]), X_MIN, X_MAX - 1);
        let y = clamp(js_round(Y_SCALE * v[1]), Y_MIN, Y_MAX - 1);
        pts.push([x, y]);
    }

    // Drop consecutive duplicates.
    let mut deduped: Vec<Pt> = Vec::with_capacity(pts.len());
    for (i, &v) in pts.iter().enumerate() {
        if i == 0 || v != pts[i - 1] {
            deduped.push(v);
        }
    }

    // Drop the repeated closing vertex (all source rings repeat vertex 0 last).
    while deduped.len() > 1 && deduped[0] == deduped[deduped.len() - 1] {
        deduped.pop();
    }
    deduped
}

/// Index of the lexicographically smallest vertex — always on the convex hull,
/// so a deterministic, safe anchor for closed-ring simplification.
pub fn extreme_vertex_index(points: &[Pt]) -> usize {
    let mut best = 0usize;
    for i in 1..points.len() {
        if points[i][0] < points[best][0]
            || (points[i][0] == points[best][0] && points[i][1] < points[best][1])
        {
            best = i;
        }
    }
    best
}

// RDP on an open chain [start, end], mutating `points` in place.  Recurses to
// the far half first so its splices do not shift indices in the near half.
fn simplify_rdp_internal(points: &mut Vec<Pt>, epsilon: f64, start: usize, end: usize) {
    if start >= points.len() || end >= points.len() || start >= end {
        return;
    }
    let mut furthest: i64 = -1;
    let mut max_dist = 0.0f64;
    for i in (start + 1)..end {
        let dist = geom::vdist_to_line(points[start], points[end], points[i]);
        if dist > max_dist {
            max_dist = dist;
            furthest = i as i64;
        }
    }
    if furthest >= 0 && max_dist > epsilon {
        let f = furthest as usize;
        simplify_rdp_internal(points, epsilon, f, end);
        simplify_rdp_internal(points, epsilon, start, f);
    } else {
        // Remove points between start and end, exclusive: indices start+1..end.
        points.drain((start + 1)..end);
    }
}

/// Ramer–Douglas–Peucker simplification of a closed ring.  Anchored on a hull
/// vertex and the vertex furthest from it (an O(n) diameter estimate), so the
/// result does not depend on where the input ring happens to start.
pub fn simplify_rdp(points: &[Pt], epsilon: f64) -> Vec<Pt> {
    if points.len() < 4 {
        return points.to_vec();
    }
    let pivot = extreme_vertex_index(points);
    let mut p: Vec<Pt> = Vec::with_capacity(points.len());
    p.extend_from_slice(&points[pivot..]);
    p.extend_from_slice(&points[..pivot]);

    let mut far = 0usize;
    let mut far_dist = -1.0f64;
    for i in 1..p.len() {
        let d = geom::vmag(geom::vsub(p[i], p[0]));
        if d > far_dist {
            far_dist = d;
            far = i;
        }
    }
    if far == 0 || far >= p.len() - 1 {
        return p;
    }

    let last = p.len() - 1;
    simplify_rdp_internal(&mut p, epsilon, far, last);
    simplify_rdp_internal(&mut p, epsilon, 0, far);
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantize_domain_and_closure() {
        assert_eq!(quantize(&[[180.0, 90.0]]), vec![[X_MAX - 1, Y_MAX - 1]]);
        assert_eq!(quantize(&[[-180.0, -90.0]]), vec![[X_MIN, Y_MIN]]);
        assert_eq!(quantize(&[[1e-7, 1e-7]]), vec![[0, 0]]);
        // Repeated closing vertex is dropped.
        assert_eq!(
            quantize(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]]).len(),
            3
        );
    }

    #[test]
    fn js_round_matches_reference() {
        // JS Math.round rounds half toward +∞.
        assert_eq!(js_round(0.5), 1);
        assert_eq!(js_round(-0.5), 0);
        assert_eq!(js_round(2.5), 3);
        assert_eq!(js_round(-2.5), -2);
    }

    #[test]
    fn rdp_keeps_corners_drops_collinear() {
        let filled = vec![[0, 0], [500, 0], [1000, 0], [1000, 500], [1000, 1000], [0, 1000]];
        assert_eq!(simplify_rdp(&filled, 5.0).len(), 4);
    }

    #[test]
    fn rdp_rotation_invariant() {
        let a = vec![[0, 0], [100, 5], [200, 0], [200, 200], [100, 195], [0, 200]];
        let mut b = a[3..].to_vec();
        b.extend_from_slice(&a[..3]);
        assert_eq!(simplify_rdp(&a, 20.0).len(), simplify_rdp(&b, 20.0).len());
    }
}
