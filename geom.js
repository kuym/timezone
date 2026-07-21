// Geometry primitives, loadable from both Node (require) and the browser
// (as a plain <script>, exposing `geom` on the global object).

(function(root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.geom = factory();
  }
})(typeof self !== "undefined" ? self : this, function() {

// Add vector `a` to `b`
function VAdd(a, b) {return [a[0] + b[0], a[1] + b[1]];}
// Subtract vector `b` from `a`
function VSub(a, b) {return [a[0] - b[0], a[1] - b[1]];}
// Scale vector `a` by scalar `s`
function VScale(a, s) {return [a[0] * s, a[1] * s];}
// Dot-product of `a` and `b`
function VDot(a, b) {return a[0] * b[0] + a[1] * b[1];}
// Return magnitude of vector `a`
function VMag(a)    {return Math.sqrt(a[0] * a[0] + a[1] * a[1]);}
// Normalize vector `a` to length 1
function VNorm(a) {return VScale(a, 1 / VMag(a));}
// Does vector `a` have zero length?
function VIsZero(a) {return (a[0] == 0) && (a[1] == 0);}

// Point on line AB closest to P
function VClosest(a, b, p) {
  const n = VNorm(VSub(b, a));
  return VAdd(a, VScale(n, VDot(VSub(p, a), n)));
}

// Point on segment AB closest to P
function VClosestSegment(a, b, p) {
  const ab = VSub(b, a), abMag = VMag(ab);
  if(abMag == 0) { return a; }
  const n = VScale(ab, 1 / abMag), t = VDot(VSub(p, a), n);
  // `t` is measured along the unit vector `n`, so it must be scaled by `n`, not by `ab`.
  return (t < 0)? a : ((t > abMag)? b : VAdd(a, VScale(n, t)));
}

// Distance from P to any point on line AB
// If A and B coincide the line is undefined, so fall back to the distance from P to A.
function VDistToLine(a, b, p) {
  const ab = VSub(b, a), ap = VSub(p, a);
  if(VIsZero(ab)) { return VMag(ap); }
  const n = VScale(ab, 1 / VMag(ab));
  return VMag(VSub(ap, VScale(n, VDot(ap, n))));
}

function VSegmentsIntersect(p1, q1, p2, q2) {
  function orientation(p, q, r) {
    const val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    return (val == 0)? 0 : ((val > 0)? 1 : 2); // collinear(0), CW(1) or CCW(2)
  }
  function onSegment(p, q, r) {
    return (
      q[0] <= ((p[0] > r[0])? p[0] : r[0]) && q[0] >= ((p[0] < r[0])? p[0] : r[0]) &&
      q[1] <= ((p[1] > r[1])? p[1] : r[1]) && q[1] >= ((p[1] < r[1])? p[1] : r[1]));
  }

  // Find the four orientations needed for general and special cases
  const o1 = orientation(p1, q1, p2), o2 = orientation(p1, q1, q2),
    o3 = orientation(p2, q2, p1), o4 = orientation(p2, q2, q1);

  return (
    (o1 != o2 && o3 != o4) || // ordinary intersection
    (o1 == 0 && onSegment(p1, p2, q1)) || // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    (o2 == 0 && onSegment(p1, q2, q1)) || // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    (o3 == 0 && onSegment(p2, p1, q2)) || // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    (o4 == 0 && onSegment(p2, q1, q2)));  // p2, q2 and q1 are collinear and q1 lies on segment p2q2
}

// AABB of polygon
function PolyAABB(poly) {
  const aabb = [[Infinity, Infinity], [-Infinity, -Infinity]];
  for(let i = 0; i < poly.length; i++) {
    const x = poly[i][0], y = poly[i][1];
    if(x < aabb[0][0]) { aabb[0][0] = x; }
    if(x > aabb[1][0]) { aabb[1][0] = x; }
    if(y < aabb[0][1]) { aabb[0][1] = y; }
    if(y > aabb[1][1]) { aabb[1][1] = y; }
  }
  return aabb;
}

// Delta ("loop") encoding of a polygon now lives in polycodec.js, which owns
// the origin/delta split; the PolyToLoop / PolyLoopEndpoint / PolyLoopArea
// helpers that used to live here were superseded by it and by tzmap.RingArea2.

// Returns a line segment (array of two 2D points) which intersects the
//   provided AABB, or false if it does not intersect
// Uses Cohen–Sutherland clipping algorithm
//   (https://en.wikipedia.org/wiki/Cohen–Sutherland_algorithm)
function SegmentIntersectsAABB(aabb, p0, p1) {
  function outcode(aabb, point) {
    let code = 0;

    if (point[0] < aabb[0][0])  code |= 4;  // left
    else if (point[0] > aabb[1][0]) code |= 1;  // right
    if (point[1] < aabb[0][1])  code |= 8;  // bottom
    else if (point[1] > aabb[1][1]) code |= 2;  // top

    return code;
  }
  let outcode0 = outcode(aabb, p0), outcode1 = outcode(aabb, p1), unclipped = true;

  while (true) {
    if (!(outcode0 | outcode1)) {
      return unclipped? true : [p0, p1];
    } else if (outcode0 & outcode1) {
      return false;
    } else {
      unclipped = false;
      const clippedPoint = [0, 0], outcodeOut = outcode1 > outcode0 ? outcode1 : outcode0;
      
      if (outcodeOut & 2) {           // point is above the clip window
        clippedPoint[0] = p0[0] + (p1[0] - p0[0]) * (aabb[1][1] - p0[1]) / (p1[1] - p0[1]);
        clippedPoint[1] = aabb[1][1];
      } else if (outcodeOut & 8) { // point is below the clip window
        clippedPoint[0] = p0[0] + (p1[0] - p0[0]) * (aabb[0][1] - p0[1]) / (p1[1] - p0[1]);
        clippedPoint[1] = aabb[0][1];
      } else if (outcodeOut & 1) {  // point is to the right of clip window
        clippedPoint[1] = p0[1] + (p1[1] - p0[1]) * (aabb[1][0] - p0[0]) / (p1[0] - p0[0]);
        clippedPoint[0] = aabb[1][0];
      } else if (outcodeOut & 4) {   // point is to the left of clip window
        clippedPoint[1] = p0[1] + (p1[1] - p0[1]) * (aabb[0][0] - p0[0]) / (p1[0] - p0[0]);
        clippedPoint[0] = aabb[0][0];
      }

      // Now we move outside point to intersection point to clip
      // and get ready for next pass.
      if (outcodeOut == outcode0) {
        outcode0 = outcode(aabb, (p0 = clippedPoint));
      } else {
        outcode1 = outcode(aabb, (p1 = clippedPoint));
      }
    }
  }
}

// Classification results for AABBIntersectsPoly().
const AABB_DISJOINT = 0;  // poly and aabb do not overlap at all
const AABB_CROSSES  = 1;  // poly boundary crosses the aabb, or poly lies inside it
const AABB_CONTAINS = 2;  // poly strictly encloses the whole aabb

// Classify how polygon `poly` relates to axis-aligned box `aabb`, treating the
//   box bounds as inclusive.
//
// IMPORTANT: the result is deliberately INDEPENDENT OF THE POLYGON'S WINDING
//   ORDER.  An earlier version returned the signed sum of the corner winding
//   numbers, so a clockwise ring that enclosed the box scored -4 instead of +4
//   and callers testing `== 4` silently discarded it.  Roughly 60% of the rings
//   in the source dataset are clockwise, so most large timezones lost their
//   interiors.  Containment is now decided by "is every corner's winding number
//   non-zero", which is sign-agnostic.
function AABBIntersectsPoly(aabb, poly) {
  // Check three things in one pass: whether any segment of the poly crosses the
  //   aabb boundary (via line clipping in SegmentIntersectsAABB), whether any
  //   poly vertex falls inside the aabb, and the winding number of each of the
  //   four aabb corners with respect to the poly.

  function isLeft(p0, p1, p2) {
    const calc = ((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]));
    return (calc > 0)? 1 : ((calc < 0)? -1 : 0);
  }
  const points = [
    [aabb[0][0], aabb[0][1]],
    [aabb[1][0], aabb[0][1]],
    [aabb[1][0], aabb[1][1]],
    [aabb[0][0], aabb[1][1]]
  ];
  const wn = Array(points.length).fill(0);

  for (let i = 0; i < poly.length; i++) {
    const next = (i == (poly.length - 1))? poly[0] : poly[i + 1];

    // A poly vertex inside the box means they overlap, even if no edge ever
    //   crosses the boundary (i.e. the poly is entirely contained by the box).
    if(poly[i][0] >= aabb[0][0] && poly[i][0] <= aabb[1][0] &&
       poly[i][1] >= aabb[0][1] && poly[i][1] <= aabb[1][1]) {
      return AABB_CROSSES;
    }

    const intersection = SegmentIntersectsAABB(aabb, poly[i], next);
    // if SegmentIntersectsAABB() returns a clipped segment, the edge crosses the
    //   aabb boundary, so we can return early
    if(intersection !== false && intersection !== true) {
      return AABB_CROSSES;
    }

    // update winding numbers for the four aabb vertices
    for (let j = 0; j < wn.length; j++) {
      if (poly[i][1] <= points[j][1]) { // start y <= P.y
        if (next[1] > points[j][1]) {  // an upward crossing
          if (isLeft(poly[i], next, points[j]) > 0) {  // P left of edge
            wn[j]++;            // have a valid up intersect
          }
        }
      } else {  // start y > P.y (no test needed)
        if (next[1] <= points[j][1]) {     // a downward crossing
          if (isLeft(poly[i], next, points[j]) < 0) {  // P right of edge
            wn[j]--;            // have a valid down intersect
          }
        }
      }
    }
  }

  // No edge crossed the boundary, so every corner has the same containment
  //   status.  Non-zero winding means inside, regardless of sign or magnitude.
  return wn.every(function(n) {return n != 0;})? AABB_CONTAINS : AABB_DISJOINT;
}

function AABBFullyEnclosesAABB(aabbOuter, aabbInner) {
  return (aabbInner[0][0] >= aabbOuter[0][0]) &&
    (aabbInner[0][1] >= aabbOuter[0][1]) &&
    (aabbInner[1][0] <= aabbOuter[1][0]) &&
    (aabbInner[1][1] <= aabbOuter[1][1]);
}

function PolyContainsPoints(poly, points) {
  function isLeft(p0, p1, p2) {
    const calc = ((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]));
    return (calc > 0)? 1 : ((calc < 0)? -1 : 0);
  }

  const wn = Array(points.length).fill(0);

  // loop through all edges of the polygon
  for (let i = 0; i < poly.length; i++) {
    const next = (i == (poly.length - 1))? poly[0] : poly[i + 1];
    for (let j = 0; j < wn.length; j++) {
      if (poly[i][1] <= points[j][1]) { // start y <= P.y
        if (next[1] > points[j][1]) {  // an upward crossing
          if (isLeft(poly[i], next, points[j]) > 0) {  // P left of edge
            wn[j]++;            // have a valid up intersect
          }
        }
      } else {  // start y > P.y (no test needed)
        if (next[1] <= points[j][1]) {     // a downward crossing
          if (isLeft(poly[i], next, points[j]) < 0) {  // P right of edge
            wn[j]--;            // have a valid down intersect
          }
        }
      }
    }
  }
  return wn.map(function(n) {return n != 0;});
}

// Signed winding number of `point` with respect to closed ring `poly`.
// Positive for a CCW ring enclosing the point, negative for CW; 0 when outside.
// PolyContainsPoint is exactly (PolyWinding != 0); this exposes the integer so
// the localized lookup can add an incremental correction to it.
function PolyWinding(poly, point) {
  function isLeft(p0, p1, p2) {
    return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
  }
  let wn = 0;
  for (let i = 0; i < poly.length; i++) {
    const cur = poly[i], next = (i == (poly.length - 1))? poly[0] : poly[i + 1];
    if (cur[1] <= point[1]) {
      if (next[1] > point[1] && isLeft(cur, next, point) > 0) { wn++; }
    } else {
      if (next[1] <= point[1] && isLeft(cur, next, point) < 0) { wn--; }
    }
  }
  return wn;
}

// Convenience single-point form of PolyContainsPoints()
function PolyContainsPoint(poly, point) {
  return PolyContainsPoints(poly, [point])[0];
}

// Containment for a polygon with holes: inside the exterior ring and outside
//   every interior ring.  `holes` may be undefined or empty.
function RingsContainPoint(outer, holes, point) {
  if(!PolyContainsPoint(outer, point)) {
    return false;
  }
  if(holes) {
    for(let i = 0; i < holes.length; i++) {
      if(PolyContainsPoint(holes[i], point)) {
        return false;
      }
    }
  }
  return true;
}

return {
  AABB_DISJOINT: AABB_DISJOINT,
  AABB_CROSSES: AABB_CROSSES,
  AABB_CONTAINS: AABB_CONTAINS,
  VAdd: VAdd,
  VSub: VSub,
  VScale: VScale,
  VDot: VDot,
  VMag: VMag,
  VNorm: VNorm,
  VIsZero: VIsZero,
  VClosest: VClosest,
  VClosestSegment: VClosestSegment,
  VDistToLine: VDistToLine,
  VSegmentsIntersect: VSegmentsIntersect,
  PolyAABB: PolyAABB,
  PolyContainsPoints: PolyContainsPoints,
  PolyContainsPoint: PolyContainsPoint,
  PolyWinding: PolyWinding,
  RingsContainPoint: RingsContainPoint,
  SegmentIntersectsAABB: SegmentIntersectsAABB,
  AABBIntersectsPoly: AABBIntersectsPoly,
  AABBFullyEnclosesAABB: AABBFullyEnclosesAABB,
};

});


if (typeof require !== "undefined" && typeof module !== "undefined" && require.main == module) {
  const UnitTest = require("./unitTest");
  const g = module.exports;
  const VClosestSegment = g.VClosestSegment, VDistToLine = g.VDistToLine,
        VSegmentsIntersect = g.VSegmentsIntersect, PolyContainsPoints = g.PolyContainsPoints,
        PolyContainsPoint = g.PolyContainsPoint, PolyWinding = g.PolyWinding,
        RingsContainPoint = g.RingsContainPoint,
        SegmentIntersectsAABB = g.SegmentIntersectsAABB, AABBIntersectsPoly = g.AABBIntersectsPoly,
        AABB_DISJOINT = g.AABB_DISJOINT, AABB_CROSSES = g.AABB_CROSSES,
        AABB_CONTAINS = g.AABB_CONTAINS;

  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[0.5, 0.5]]), [true]);
  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[0.5, 0.5], [0.5, 0.75], [0.75, 0.5]]), [true, true, true]);
  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[0.5, 0.5], [1.5, 0.5], [0.5, 0.75]]), [true, false, true]);
  
  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[-0.5, 0.5]]), [false]);
  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[1.5, 0.5]]), [false]);
  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[0.5, 1.5]]), [false]);
  UnitTest(PolyContainsPoints([[0, 0], [1, 0], [1, 1], [0, 1]], [[0.5, -0.5]]), [false]);

  // Pathological case that locked up
  //UnitTest(SegmentIntersectsAABB([[65535, 32767], [131071, 65535]], [101872, 32897], [101734, 32731]));

  UnitTest(SegmentIntersectsAABB([[100, 100], [1000, 1000]], [0, 1], [1, 0]), false);
  UnitTest(SegmentIntersectsAABB([[100, 100], [1000, 1000]], [110, 120], [120, 110]), true);
  UnitTest(SegmentIntersectsAABB([[100, 100], [1000, 1000]], [90, 140], [120, 110]), [[100, 130], [120, 110]]);


  const cell = [[100, 100], [1000, 1000]];
  UnitTest(AABBIntersectsPoly(cell, [[-1, -1], [-10, 1], [-10, -10], [-1, -10]]), AABB_DISJOINT);
  UnitTest(AABBIntersectsPoly(cell, [[10, 10], [110, 10], [110, 110], [10, 110]]), AABB_CROSSES);
  UnitTest(AABBIntersectsPoly(cell, [[10, 10], [1100, 10], [1100, 1100], [10, 1100]]), AABB_CONTAINS);
  // a poly entirely inside the cell overlaps it, even though no edge crosses the boundary
  UnitTest(AABBIntersectsPoly(cell, [[200, 200], [300, 200], [300, 300], [200, 300]]), AABB_CROSSES);

  // REGRESSION (B1): classification must not depend on winding order.  Nearly
  //   60% of the source dataset's outer rings are clockwise, and the previous
  //   signed-sum implementation silently discarded every one of them that
  //   enclosed a cell.
  [
    [[-1, -1], [-10, 1], [-10, -10], [-1, -10]],            // disjoint
    [[10, 10], [110, 10], [110, 110], [10, 110]],           // crossing
    [[10, 10], [1100, 10], [1100, 1100], [10, 1100]],       // containing
    [[200, 200], [300, 200], [300, 300], [200, 300]]        // contained by cell
  ].forEach(function(poly, i) {
    UnitTest(
      AABBIntersectsPoly(cell, poly.slice().reverse()),
      AABBIntersectsPoly(cell, poly),
      "winding-invariance[" + i + "]");
  });

  // PolyContainsPoints is winding-agnostic too
  const sq = [[0, 0], [10, 0], [10, 10], [0, 10]];
  UnitTest(PolyContainsPoint(sq, [5, 5]), true);
  UnitTest(PolyContainsPoint(sq.slice().reverse(), [5, 5]), true);

  // Holes
  const hole = [[4, 4], [6, 4], [6, 6], [4, 6]];
  UnitTest(RingsContainPoint(sq, [hole], [5, 5]), false);
  UnitTest(RingsContainPoint(sq, [hole], [1, 1]), true);
  UnitTest(RingsContainPoint(sq, [hole.slice().reverse()], [5, 5]), false);
  UnitTest(RingsContainPoint(sq, [], [5, 5]), true);

  // B6: VSegmentsIntersect used to throw ReferenceError on the collinear path
  UnitTest(VSegmentsIntersect([0, 0], [10, 0], [5, 0], [20, 0]), true);
  UnitTest(VSegmentsIntersect([0, 0], [10, 0], [0, 5], [10, 5]), false);
  UnitTest(VSegmentsIntersect([0, 0], [10, 10], [0, 10], [10, 0]), true);

  // B7: VClosestSegment scaled by `ab` instead of the unit vector `n`
  UnitTest(VClosestSegment([0, 0], [10, 0], [5, 5]), [5, 0]);
  UnitTest(VClosestSegment([0, 0], [10, 0], [-5, 5]), [0, 0]);
  UnitTest(VClosestSegment([0, 0], [10, 0], [15, 5]), [10, 0]);
  UnitTest(VClosestSegment([3, 3], [3, 3], [15, 5]), [3, 3]);   // degenerate segment

  // B9: VDistToLine must not return NaN for a degenerate line
  UnitTest(VDistToLine([1, 1], [1, 1], [4, 5]), 5);

  // PolyWinding: +1 inside a CCW ring, -1 inside a CW ring, 0 outside; and
  //   (PolyWinding != 0) must agree with PolyContainsPoint everywhere.
  const wsq = [[0, 0], [10, 0], [10, 10], [0, 10]];
  UnitTest(PolyWinding(wsq, [5, 5]), 1);
  UnitTest(PolyWinding(wsq.slice().reverse(), [5, 5]), -1);
  UnitTest(PolyWinding(wsq, [15, 5]), 0);
  [[5, 5], [1, 9], [15, 5], [-3, 2], [9, 1]].forEach(function(p) {
    UnitTest(PolyWinding(wsq, p) != 0, PolyContainsPoint(wsq, p), "winding vs contains " + p);
  });

  console.log("geom.js: all tests passed");
}
