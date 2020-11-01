const UnitTest = require("./unitTest");

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
  const ab = VSub(b, a), abMag = VMag(ab), n = VScale(ab, 1 / abMag), t = VDot(VSub(p, a), n);
  return (t < 0)? a : ((t > abMag)? b : VAdd(a, VScale(ab, t)));
}

// Distance from P to any point on line AB
function VDistToLine(a, b, p) {
  const n = VNorm(VSub(b, a)), ap = VSub(p, a);
  return VMag(VSub(ap, VScale(n, VDot(ap, n))));
}

function VSegmentsIntersect(p1, q1, p2, q2) {
  function orientation(p, q, r) {
    const val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    return (val == 0)? 0 : ((val > 0)? 1 : 2); // collinear(0), CW(1) or CCW(2)
  }
  function onSegment(p, q, r) {
    return (
      q[0] <= ((p[0] > r[0])? p[0] : y[0]) && q[0] >= ((p[0] < r[0])? p[0] : r[0]) &&
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

function PolyToLoop(poly) {
  let p = [0, 0];
  return poly.map(function(v, i) {
    const d = VSub(v, p);
    p = v;
    return d;
  });
}

// Find the final point of polygon loop `polyLoop`
function PolyLoopEndpoint(polyLoop) {
  return polyLoop.reduce(function(a, b) {return VAdd(a, b)});
}

function PolyLoopArea(polyLoop) {
  let a = 0, p = [0, 0];
  polyLoop.forEach(function(v) {
    a += (p[0] * (p[1] + v[1]) - p[1] * (p[0] + v[0]));
    p[0] += v[0];
    p[1] += v[1];
  });
  return 0.5 * a;
}

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

function AABBIntersectsPoly(aabb, poly) {
  // Check two things in parallel: both if any segment of the poly intersects
  //   the aabb, which is done using line clipping via SegmentIntersectsAABB(),
  //   and determine the number of vertices of the aabb which are within the poly.

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
    const intersection = SegmentIntersectsAABB(aabb, poly[i], next);
    // if SegmentIntersectsAABB() returns a clipped segment, there was an intersection
    //   between poly and aabb, so we can return early
    if(intersection !== false && intersection !== true) {
      return 1;
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
  // return the number of vertices inside the poly, which could only
  //   be 0 or 4 because we return early above if the poly and aabb intersect.
  return wn[0] + wn[1] + wn[2] + wn[3];
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

module.exports = {
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
  PolyToLoop: PolyToLoop,
  PolyLoopEndpoint: PolyLoopEndpoint,
  PolyLoopArea: PolyLoopArea,
  AABBIntersectsPoly: AABBIntersectsPoly,
  AABBFullyEnclosesAABB: AABBFullyEnclosesAABB,
};


if (require.main == module) {
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


  UnitTest(AABBIntersectsPoly([[100, 100], [1000, 1000]], [[-1, -1], [-10, 1], [-10, -10], [-1, -10]]), 0);
  UnitTest(AABBIntersectsPoly([[100, 100], [1000, 1000]], [[10, 10], [110, 10], [110, 110], [10, 110]]), 1);
  UnitTest(AABBIntersectsPoly([[100, 100], [1000, 1000]], [[10, 10], [1100, 10], [1100, 1100], [10, 1100]]), 4);
}
