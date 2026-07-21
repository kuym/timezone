const geom = require("./geom");
const UnitTest = require("./unitTest");

// ---------------------------------------------------------------------------
// Quantization domain
// ---------------------------------------------------------------------------
//
// Longitude maps to x over [-2^19, 2^19), latitude maps to y over [-2^18, 2^18).
// Both axes therefore use the same scale (2912.71 units per degree), so cells
// are square in lat/lon space, and both extents are exact powers of two.
//
// The domain is HALF-OPEN: a cell is [lo, hi) on both axes.  This is what makes
// the quadtree portable.  Because every cell bound is a multiple of that cell's
// power-of-two size, lo + hi is always even, so the midpoint (lo + hi) / 2 is
// exact -- truncate-toward-zero, floor, and an arithmetic >>1 all agree.  The
// previous inclusive-max domain ([-524288 .. 524287]) had odd sums, so the
// original `parseInt((lo + hi) / 2)` truncated toward zero while the obvious C
// or Rust `(lo + hi) >> 1` would floor, silently disagreeing on every cell left
// of the prime meridian.
const X_MIN = -524288, X_MAX = 524288;   // 2^19
const Y_MIN = -262144, Y_MAX = 262144;   // 2^18
const X_SCALE = 524288 / 180;
const Y_SCALE = 262144 / 90;

// Root cell of the quadtree, half-open on both axes.
const ROOT_CELL = [[X_MIN, Y_MIN], [X_MAX, Y_MAX]];

// Maximum subdivision depth.  Must stay <= 19 so that the narrower axis still
// has a cell size of at least 2 units and midpoints remain exact.
const MAX_DEPTH = 16;

// Split threshold: a leaf holding this many candidate polygons (refs)
// subdivides when the next one is inserted, so a leaf below MAX_DEPTH holds at
// most SPLIT_THRESHOLD candidates.  Lower means a deeper tree with fewer
// polygons to test per lookup, at the cost of more nodes.  `eref` zones (those
// that provably cover the whole cell) are resolved without any test and do not
// count toward this limit.
const SPLIT_THRESHOLD = 2;

function clamp(v, lo, hi) {
  return (v < lo)? lo : ((v > hi)? hi : v);
}

// Convert [longitude, latitude] degree pairs into fixed-point integer vertices.
//
// Uses Math.round rather than the original parseInt for two reasons: parseInt
// stringifies its argument first, so a scaled value small enough to reach
// exponential notation ("5e-7") parsed as its mantissa (5), and truncation
// toward zero biased every vertex toward the origin.  Results are clamped into
// the domain so that a vertex at exactly +-180 or +-90 degrees stays inside the
// quadtree root instead of falling out of it.
function Quantize(lonLatPoints) {
  const points = lonLatPoints.map(function(v) {
    return [
      clamp(Math.round(X_SCALE * v[0]), X_MIN, X_MAX - 1),
      clamp(Math.round(Y_SCALE * v[1]), Y_MIN, Y_MAX - 1)
    ];
  });

  // Drop consecutive duplicates, which quantization creates in abundance.
  const deduped = points.filter(function(v, i) {
    return (i == 0) || !geom.VIsZero(geom.VSub(v, points[i - 1]));
  });

  // GeoJSON rings repeat their first vertex at the end (all 1456 rings in the
  //   source dataset do).  Closure is implicit everywhere in this codebase, so
  //   the duplicate is pure waste: it costs bytes, adds a zero-length edge that
  //   every containment test has to skip, and gives RDP a degenerate
  //   start == end segment that used to collapse an entire lobe of the ring.
  while (deduped.length > 1 &&
         geom.VIsZero(geom.VSub(deduped[0], deduped[deduped.length - 1]))) {
    deduped.pop();
  }
  return deduped;
}

// Index of the lexicographically smallest vertex.  This is always a vertex of
// the convex hull, so anchoring simplification there is safe and deterministic.
function ExtremeVertexIndex(points) {
  let best = 0;
  for (let i = 1; i < points.length; i++) {
    if ((points[i][0] < points[best][0]) ||
        ((points[i][0] == points[best][0]) && (points[i][1] < points[best][1]))) {
      best = i;
    }
  }
  return best;
}

// Ramer-Douglas-Peucker polygon simplification algorithm
// https://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm
//
// A ring has no natural endpoints, so it is cut into two chains at two anchor
// vertices which are always retained.  The anchors are a convex-hull vertex and
// the vertex furthest from it (an O(n) approximation of the ring's diameter).
// The original code anchored at index length/2, an arbitrary point that could
// sit anywhere -- including in the middle of a detail-rich stretch -- which
// distorted the result for no benefit.
function SimplifyRDP(points, epsilon) {
  function simplifyRDPInternal(points, epsilon, start, end) {
    if(start < 0 || start >= points.length || end < 0 || end >= points.length || start >= end) {
      return;
    }

    let furthest = -1;
    let maxDist = 0;
    for(let i = start + 1; i < end; i++) {
      const dist = geom.VDistToLine(points[start], points[end], points[i]);
      if(dist > maxDist) {
        maxDist = dist;
        furthest = i;
      }
    }

    if((furthest >= 0) && (maxDist > epsilon)) {
      // recurse to second half first, as it will modify the `points` array
      simplifyRDPInternal(points, epsilon, furthest, end);
      simplifyRDPInternal(points, epsilon, start, furthest);
    } else {
      // cut all points between `start` and `end`, exclusive
      points.splice(start + 1, end - start - 1);
    }
  }

  if(points.length < 4) {
    return points.slice();
  }

  // Rotate so the ring begins at a hull vertex, then anchor the far end at the
  //   vertex furthest from it.
  const pivot = ExtremeVertexIndex(points);
  const p = points.slice(pivot).concat(points.slice(0, pivot));

  let far = 0, farDist = -1;
  for(let i = 1; i < p.length; i++) {
    const d = geom.VMag(geom.VSub(p[i], p[0]));
    if(d > farDist) { farDist = d; far = i; }
  }
  if(far <= 0 || far >= p.length - 1) {
    // Degenerate ring (all vertices coincident, or the far point is an
    //   endpoint); nothing meaningful to split on.
    return p;
  }

  // Simplify the far chain first so the near chain's indices stay valid.
  simplifyRDPInternal(p, epsilon, far, p.length - 1);
  simplifyRDPInternal(p, epsilon, 0, far);
  return p;
}

// Signed area doubled; sign carries winding order, magnitude carries area.
function RingArea2(points) {
  let a = 0;
  for(let i = 0; i < points.length; i++) {
    const n = points[(i + 1) % points.length];
    a += points[i][0] * n[1] - n[0] * points[i][1];
  }
  return a;
}

// Does any non-adjacent pair of edges cross?  O(n^2), so this is a validation
// aid for tests and small rings, not something to run over a 148k-vertex ring.
function IsSelfIntersecting(points) {
  const n = points.length;
  for(let i = 0; i < n; i++) {
    const a1 = points[i], a2 = points[(i + 1) % n];
    for(let j = i + 1; j < n; j++) {
      if((j == i) || ((j + 1) % n == i) || (j == (i + 1) % n)) { continue; }
      const b1 = points[j], b2 = points[(j + 1) % n];
      if(geom.VSegmentsIntersect(a1, a2, b1, b2)) { return true; }
    }
  }
  return false;
}

module.exports = {
  X_MIN: X_MIN, X_MAX: X_MAX, Y_MIN: Y_MIN, Y_MAX: Y_MAX,
  X_SCALE: X_SCALE, Y_SCALE: Y_SCALE,
  ROOT_CELL: ROOT_CELL,
  MAX_DEPTH: MAX_DEPTH,
  SPLIT_THRESHOLD: SPLIT_THRESHOLD,
  Quantize: Quantize,
  SimplifyRDP: SimplifyRDP,
  ExtremeVertexIndex: ExtremeVertexIndex,
  RingArea2: RingArea2,
  IsSelfIntersecting: IsSelfIntersecting,
};


if (require.main == module) {
  // MAX_DEPTH must keep midpoints exact on the narrower axis.
  UnitTest(MAX_DEPTH <= 19, true, "MAX_DEPTH keeps midpoints exact");

  // Quantization range: the extremes of the coordinate system must land inside
  //   the quadtree root rather than one unit outside it.
  UnitTest(Quantize([[180, 90]]), [[X_MAX - 1, Y_MAX - 1]]);
  UnitTest(Quantize([[-180, -90]]), [[X_MIN, Y_MIN]]);
  UnitTest(Quantize([[0, 0]]), [[0, 0]]);

  // B8: parseInt read "5e-7" as 5; Math.round does not.
  UnitTest(Quantize([[1e-7, 1e-7]]), [[0, 0]]);

  // B17: the repeated closing vertex is dropped.
  UnitTest(Quantize([[0, 0], [1, 0], [1, 1], [0, 0]]).length, 3);

  // Consecutive duplicates collapse.
  UnitTest(Quantize([[0, 0], [0, 0], [1, 0]]).length, 2);

  // B9: a ring whose first and last vertices coincide used to make RDP compute
  //   a NaN distance and splice out everything in between.
  const collapsed = SimplifyRDP([[0, 0], [1000, 0], [1000, 1000], [0, 1000], [0, 0]], 5);
  UnitTest(collapsed.length >= 4, true, "closed ring survives simplification");

  // Simplification keeps a square's corners and removes collinear filler.
  const filled = [[0, 0], [500, 0], [1000, 0], [1000, 500], [1000, 1000], [0, 1000]];
  UnitTest(SimplifyRDP(filled, 5).length, 4);

  // A coarse epsilon reduces a zigzag to its anchors.
  UnitTest(SimplifyRDP([[0, 0], [10, 50], [20, 0], [30, 50], [40, 0]], 1000).length <= 3, true,
    "coarse epsilon collapses zigzag");

  // Simplification must not depend on where the input ring happens to start.
  const ringA = [[0, 0], [100, 5], [200, 0], [200, 200], [100, 195], [0, 200]];
  const ringB = ringA.slice(3).concat(ringA.slice(0, 3));
  UnitTest(SimplifyRDP(ringA, 20).length, SimplifyRDP(ringB, 20).length,
    "rotation-invariant vertex count");

  UnitTest(RingArea2([[0, 0], [10, 0], [10, 10], [0, 10]]), 200);
  UnitTest(RingArea2([[0, 0], [0, 10], [10, 10], [10, 0]]), -200);

  UnitTest(IsSelfIntersecting([[0, 0], [10, 0], [10, 10], [0, 10]]), false);
  UnitTest(IsSelfIntersecting([[0, 0], [10, 10], [10, 0], [0, 10]]), true);

  console.log("tzmap.js: all tests passed");
}
