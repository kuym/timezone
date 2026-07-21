//! Quadtree cell arithmetic and the localized point-in-polygon test — the
//! reader-side geometry shared by the build's verifier.  Port of the pure
//! helpers in tzlookup.js (the tree walk itself lives in build.rs, where the
//! node types are defined).

use crate::geom::{self, Aabb, Pt};

/// Split a cell into the child of the given quadrant.
///
/// Quadrants are cartesian I–IV: 0 = +x+y (NE), 1 = −x+y (NW), 2 = −x−y (SW),
/// 3 = +x−y (SE).  Cells are half-open and every bound is a multiple of the
/// cell's power-of-two size, so `>> 1` is exact (floor == trunc == arithmetic
/// shift all agree).
pub fn split_cell(cell: &Aabb, quadrant: usize) -> Aabb {
    let mx = (cell[0][0] + cell[1][0]) >> 1;
    let my = (cell[0][1] + cell[1][1]) >> 1;
    match quadrant {
        0 => [[mx, my], [cell[1][0], cell[1][1]]],
        1 => [[cell[0][0], my], [mx, cell[1][1]]],
        2 => [[cell[0][0], cell[0][1]], [mx, my]],
        3 => [[mx, cell[0][1]], [cell[1][0], my]],
        _ => unreachable!("quadrant out of range"),
    }
}

pub fn cell_center(cell: &Aabb) -> Pt {
    [
        (cell[0][0] + cell[1][0]) >> 1,
        (cell[0][1] + cell[1][1]) >> 1,
    ]
}

/// Which child quadrant of `cell` contains `point`, under half-open bounds.
pub fn quadrant_for_point(cell: &Aabb, point: Pt) -> usize {
    let mx = (cell[0][0] + cell[1][0]) >> 1;
    let my = (cell[0][1] + cell[1][1]) >> 1;
    if point[0] >= mx {
        if point[1] >= my {
            0
        } else {
            3
        }
    } else if point[1] >= my {
        1
    } else {
        2
    }
}

/// Signed number of times the directed segment `c→q` crosses the ring edges in
/// `runs` (inclusive `[first, last]` edge-index ranges).  `None` signals a
/// degenerate orientation (a vertex on `c→q`, or a collinear edge) so the caller
/// falls back to a full point-in-polygon test.
pub fn segment_crossings(c: Pt, q: Pt, ring: &[Pt], runs: &[(u32, u32)]) -> Option<i64> {
    let n = ring.len();
    let mut delta = 0i64;
    for &(first, last) in runs {
        for i in first..=last {
            let a = ring[i as usize];
            let b = ring[(i as usize + 1) % n];
            let o1 = geom::is_left(a, b, c);
            let o2 = geom::is_left(a, b, q);
            if o1 == 0 || o2 == 0 {
                return None;
            }
            if (o1 > 0) == (o2 > 0) {
                continue; // c and q on the same side
            }
            let o3 = geom::is_left(c, q, a);
            let o4 = geom::is_left(c, q, b);
            if o3 == 0 || o4 == 0 {
                return None;
            }
            if (o3 > 0) == (o4 > 0) {
                continue; // a and b on the same side
            }
            delta += if o2 > 0 { 1 } else { -1 };
        }
    }
    Some(delta)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quant::{MAX_DEPTH, ROOT_CELL};

    #[test]
    fn midpoints_exact_to_max_depth() {
        // >>1 == floor((lo+hi)/2) at every level, because sums stay even.
        let mut cell = ROOT_CELL;
        for _ in 0..=MAX_DEPTH {
            for axis in 0..2 {
                let sum = cell[0][axis] + cell[1][axis];
                assert_eq!(sum % 2, 0);
                assert_eq!(sum >> 1, (sum as f64 / 2.0).floor() as i64);
            }
            cell = split_cell(&cell, 0);
        }
    }

    #[test]
    fn point_stays_in_chosen_cell() {
        let mut cell = ROOT_CELL;
        let p = [12345, -6789];
        for _ in 0..MAX_DEPTH {
            let q = quadrant_for_point(&cell, p);
            cell = split_cell(&cell, q);
            assert!(p[0] >= cell[0][0] && p[0] < cell[1][0]);
            assert!(p[1] >= cell[0][1] && p[1] < cell[1][1]);
        }
    }
}
