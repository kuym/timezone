//! Geometry primitives — port of geom.js.
//!
//! All spatial queries used by the build operate on quantized integer points
//! (`Pt = [i64; 2]`).  Only Ramer–Douglas–Peucker (in `quant.rs`) needs floating
//! point, and it uses the small vector helpers at the bottom of this file.

/// A quantized point: `[x, y]` in fixed-point units.
pub type Pt = [i64; 2];
/// An axis-aligned box: `[[xLo, yLo], [xHi, yHi]]`.
pub type Aabb = [Pt; 2];

/// How a polygon relates to a box (winding-order independent — the B1 fix).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AabbPoly {
    /// No overlap.
    Disjoint,
    /// The polygon boundary crosses the box, or the polygon lies inside it.
    Crosses,
    /// The polygon strictly encloses the whole box.
    Contains,
}

/// How a segment relates to a box.  Mirrors geom.js `SegmentIntersectsAABB`
/// returning `true` (Inside) / a clipped segment (Crosses) / `false` (Outside).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SegRel {
    Outside,
    Inside,
    Crosses,
}

pub fn aabbs_overlap(a: &Aabb, b: &Aabb) -> bool {
    !(a[1][0] < b[0][0] || a[0][0] > b[1][0] || a[1][1] < b[0][1] || a[0][1] > b[1][1])
}

pub fn point_in_aabb(p: Pt, a: &Aabb) -> bool {
    p[0] >= a[0][0] && p[0] <= a[1][0] && p[1] >= a[0][1] && p[1] <= a[1][1]
}

pub fn poly_aabb(poly: &[Pt]) -> Aabb {
    let mut aabb: Aabb = [[i64::MAX, i64::MAX], [i64::MIN, i64::MIN]];
    for v in poly {
        if v[0] < aabb[0][0] {
            aabb[0][0] = v[0];
        }
        if v[0] > aabb[1][0] {
            aabb[1][0] = v[0];
        }
        if v[1] < aabb[0][1] {
            aabb[0][1] = v[1];
        }
        if v[1] > aabb[1][1] {
            aabb[1][1] = v[1];
        }
    }
    aabb
}

// Cohen–Sutherland outcode for a floating-point point against an integer box.
fn outcode(a: &Aabb, x: f64, y: f64) -> u8 {
    let mut code = 0u8;
    if x < a[0][0] as f64 {
        code |= 4; // left
    } else if x > a[1][0] as f64 {
        code |= 1; // right
    }
    if y < a[0][1] as f64 {
        code |= 8; // bottom
    } else if y > a[1][1] as f64 {
        code |= 2; // top
    }
    code
}

// Faithful port of SegmentIntersectsAABB: distinguish a segment lying wholly
// inside the box, one that crosses its boundary, and one entirely outside.
fn seg_intersects_aabb(a: &Aabb, p0: Pt, p1: Pt) -> SegRel {
    let (mut x0, mut y0) = (p0[0] as f64, p0[1] as f64);
    let (mut x1, mut y1) = (p1[0] as f64, p1[1] as f64);
    let mut oc0 = outcode(a, x0, y0);
    let mut oc1 = outcode(a, x1, y1);
    let mut unclipped = true;

    loop {
        if (oc0 | oc1) == 0 {
            return if unclipped { SegRel::Inside } else { SegRel::Crosses };
        } else if (oc0 & oc1) != 0 {
            return SegRel::Outside;
        }
        unclipped = false;
        let out = if oc1 > oc0 { oc1 } else { oc0 };
        let (mut cx, mut cy) = (0.0, 0.0);
        let (xl, yl, xh, yh) = (
            a[0][0] as f64,
            a[0][1] as f64,
            a[1][0] as f64,
            a[1][1] as f64,
        );
        if out & 2 != 0 {
            cx = x0 + (x1 - x0) * (yh - y0) / (y1 - y0);
            cy = yh;
        } else if out & 8 != 0 {
            cx = x0 + (x1 - x0) * (yl - y0) / (y1 - y0);
            cy = yl;
        } else if out & 1 != 0 {
            cy = y0 + (y1 - y0) * (xh - x0) / (x1 - x0);
            cx = xh;
        } else if out & 4 != 0 {
            cy = y0 + (y1 - y0) * (xl - x0) / (x1 - x0);
            cx = xl;
        }
        if out == oc0 {
            x0 = cx;
            y0 = cy;
            oc0 = outcode(a, x0, y0);
        } else {
            x1 = cx;
            y1 = cy;
            oc1 = outcode(a, x1, y1);
        }
    }
}

/// Does edge `p0->p1` touch or cross the box (fully inside counts)?  This is the
/// `SegmentIntersectsAABB(...) !== false` test used to count edges in a cell.
pub fn edge_touches_aabb(a: &Aabb, p0: Pt, p1: Pt) -> bool {
    seg_intersects_aabb(a, p0, p1) != SegRel::Outside
}

/// Twice the signed area of triangle `p0,p1,p2`: > 0 if `p2` is left of `p0→p1`.
#[inline]
pub fn is_left(p0: Pt, p1: Pt, p2: Pt) -> i64 {
    (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])
}

/// Classify a polygon against a box, independent of winding order (B1 fix): the
/// box is Contained iff every corner has a non-zero winding number.
pub fn aabb_intersects_poly(aabb: &Aabb, poly: &[Pt]) -> AabbPoly {
    let corners = [
        [aabb[0][0], aabb[0][1]],
        [aabb[1][0], aabb[0][1]],
        [aabb[1][0], aabb[1][1]],
        [aabb[0][0], aabb[1][1]],
    ];
    let mut wn = [0i64; 4];
    let n = poly.len();
    for i in 0..n {
        let cur = poly[i];
        let next = poly[if i == n - 1 { 0 } else { i + 1 }];

        // A polygon vertex inside the box means overlap even if no edge crosses.
        if point_in_aabb(cur, aabb) {
            return AabbPoly::Crosses;
        }
        if seg_intersects_aabb(aabb, cur, next) == SegRel::Crosses {
            return AabbPoly::Crosses;
        }

        for j in 0..4 {
            let pj = corners[j];
            if cur[1] <= pj[1] {
                if next[1] > pj[1] && is_left(cur, next, pj) > 0 {
                    wn[j] += 1;
                }
            } else if next[1] <= pj[1] && is_left(cur, next, pj) < 0 {
                wn[j] -= 1;
            }
        }
    }
    if wn.iter().all(|&n| n != 0) {
        AabbPoly::Contains
    } else {
        AabbPoly::Disjoint
    }
}

/// Signed winding number of `point` w.r.t. closed ring `poly`.
pub fn poly_winding(poly: &[Pt], point: Pt) -> i64 {
    let mut wn = 0i64;
    let n = poly.len();
    for i in 0..n {
        let cur = poly[i];
        let next = poly[if i == n - 1 { 0 } else { i + 1 }];
        if cur[1] <= point[1] {
            if next[1] > point[1] && is_left(cur, next, point) > 0 {
                wn += 1;
            }
        } else if next[1] <= point[1] && is_left(cur, next, point) < 0 {
            wn -= 1;
        }
    }
    wn
}

pub fn poly_contains_point(poly: &[Pt], point: Pt) -> bool {
    poly_winding(poly, point) != 0
}

/// Containment for a polygon with holes: inside the exterior and outside every
/// interior ring.
pub fn rings_contain_point(outer: &[Pt], holes: &[Vec<Pt>], point: Pt) -> bool {
    if !poly_contains_point(outer, point) {
        return false;
    }
    for h in holes {
        if poly_contains_point(h, point) {
            return false;
        }
    }
    true
}

/// Signed doubled area of a ring (sign = winding, magnitude = 2·area).
pub fn ring_area2(poly: &[Pt]) -> i64 {
    let mut a = 0i64;
    let n = poly.len();
    for i in 0..n {
        let cur = poly[i];
        let next = poly[(i + 1) % n];
        a += cur[0] * next[1] - next[0] * cur[1];
    }
    a
}

// --- floating-point vector helpers, used only by RDP simplification ---

pub fn vsub(a: Pt, b: Pt) -> [f64; 2] {
    [(a[0] - b[0]) as f64, (a[1] - b[1]) as f64]
}
pub fn vmag(a: [f64; 2]) -> f64 {
    (a[0] * a[0] + a[1] * a[1]).sqrt()
}

/// Distance from point `p` to the infinite line through `a`,`b`; falls back to
/// the distance `p`→`a` when the segment is degenerate (matches geom.js).
pub fn vdist_to_line(a: Pt, b: Pt, p: Pt) -> f64 {
    let ab = vsub(b, a);
    let ap = vsub(p, a);
    let m = vmag(ab);
    if m == 0.0 {
        return vmag(ap);
    }
    // Match geom.js exactly: VScale(ab, 1/VMag(ab)) multiplies by the reciprocal
    // rather than dividing, which rounds differently in the last ULP and can flip
    // an RDP keep/drop decision.
    let inv = 1.0 / m;
    let n = [ab[0] * inv, ab[1] * inv];
    let d = ap[0] * n[0] + ap[1] * n[1];
    vmag([ap[0] - n[0] * d, ap[1] - n[1] * d])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn winding_independent_classification() {
        let cell: Aabb = [[100, 100], [1000, 1000]];
        let contains = [[10, 10], [1100, 10], [1100, 1100], [10, 1100]];
        let crosses = [[10, 10], [110, 10], [110, 110], [10, 110]];
        let disjoint = [[-1, -1], [-10, 1], [-10, -10], [-1, -10]];
        for poly in [&contains[..], &crosses[..], &disjoint[..]] {
            let rev: Vec<Pt> = poly.iter().rev().cloned().collect();
            assert_eq!(aabb_intersects_poly(&cell, poly), aabb_intersects_poly(&cell, &rev));
        }
        assert_eq!(aabb_intersects_poly(&cell, &contains), AabbPoly::Contains);
        assert_eq!(aabb_intersects_poly(&cell, &crosses), AabbPoly::Crosses);
        assert_eq!(aabb_intersects_poly(&cell, &disjoint), AabbPoly::Disjoint);
    }

    #[test]
    fn winding_and_area() {
        let sq = [[0, 0], [10, 0], [10, 10], [0, 10]];
        assert_eq!(poly_winding(&sq, [5, 5]), 1);
        let rev: Vec<Pt> = sq.iter().rev().cloned().collect();
        assert_eq!(poly_winding(&rev, [5, 5]), -1);
        assert_eq!(poly_winding(&sq, [15, 5]), 0);
        assert_eq!(ring_area2(&sq), 200);
        assert_eq!(ring_area2(&rev), -200);
    }

    #[test]
    fn holes() {
        let outer = vec![[0, 0], [100, 0], [100, 100], [0, 100]];
        let hole = vec![[40, 40], [60, 40], [60, 60], [40, 60]];
        assert!(!rings_contain_point(&outer, &[hole.clone()], [50, 50]));
        assert!(rings_contain_point(&outer, &[hole], [10, 10]));
    }
}
