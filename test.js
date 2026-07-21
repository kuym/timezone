// Project test suite.  Run with `node test.js`.
//
// geom.js and tzmap.js carry their own unit tests (`node geom.js`, `node
// tzmap.js`); this file covers the codec, the quadtree, and the cross-module
// invariants that the C and Rust ports will have to reproduce.

const geom = require("./geom");
const tzmap = require("./tzmap");
const polycodec = require("./polycodec");
const tzlookup = require("./tzlookup");
const tzconvert = require("./tzconvert");
const UnitTest = require("./unitTest");

let sections = 0;
function section(name) { sections++; console.log("  " + name); }

// Deterministic PRNG so failures reproduce.
let seed = 987654321;
function rnd() {
  seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
  return seed / 0x7FFFFFFF;
}
function rndInt(lo, hi) { return lo + Math.floor(rnd() * (hi - lo)); }

console.log("polycodec");

section("every wire form round-trips at its boundary values");
polycodec.FORMS.forEach(function(form) {
  const limit = 1 << (form.bits - 1);
  [[0, 0], [limit - 1, limit - 1], [-limit, -limit], [limit - 1, -limit], [-limit, limit - 1]]
    .forEach(function(v) {
      UnitTest(polycodec.decodeVectors(polycodec.encodeVectors([v])), [v],
        form.bits + "-bit form " + JSON.stringify(v));
    });
});

section("form selection picks the smallest form that fits");
UnitTest(polycodec.encodeVectors([[0, 0]]).length, 2);
UnitTest(polycodec.encodeVectors([[63, -64]]).length, 2);
UnitTest(polycodec.encodeVectors([[64, 0]]).length, 3);
UnitTest(polycodec.encodeVectors([[1023, -1024]]).length, 3);
UnitTest(polycodec.encodeVectors([[1024, 0]]).length, 5);
UnitTest(polycodec.encodeVectors([[262143, -262144]]).length, 5);
UnitTest(polycodec.encodeVectors([[262144, 0]]).length, 6);

section("B3 regression: negative y delta must not take the short form");
// The old range check tested x twice, so a y delta below -1024 was truncated to
// 11 bits and every subsequent vertex was displaced.
UnitTest(polycodec.decodeVectors(polycodec.encodeVectors([[10, -2000]])), [[10, -2000]]);
UnitTest(polycodec.decodeVectors(polycodec.encodeVectors([[-2000, 10]])), [[-2000, 10]]);

section("B2 regression: polygons far from the origin survive encoding");
// The delta walk used to be seeded from [0,0], making vector #0 an absolute
// coordinate that overflowed the 19-bit form.
[
  [[10, 10], [20, 20], [30, 10]],
  [[400000, 100000], [400010, 100010]],
  [[-524288, -262144], [-524200, -262100]],
  [[524287, 262143], [524000, 262000]],
  [[10, 10], [10, -2000], [500, 500]]
].forEach(function(poly, i) {
  const enc = polycodec.encodePolygon(poly);
  UnitTest(polycodec.decodePolygon(enc.o, enc.p), poly, "far-poly[" + i + "]");
});

section("random polygons across the whole domain round-trip");
for (let t = 0; t < 300; t++) {
  const poly = [];
  let x = rndInt(tzmap.X_MIN, tzmap.X_MAX), y = rndInt(tzmap.Y_MIN, tzmap.Y_MAX);
  poly.push([x, y]);
  for (let i = 0; i < 40; i++) {
    x = Math.max(tzmap.X_MIN, Math.min(tzmap.X_MAX - 1, x + rndInt(-3000, 3000)));
    y = Math.max(tzmap.Y_MIN, Math.min(tzmap.Y_MAX - 1, y + rndInt(-3000, 3000)));
    poly.push([x, y]);
  }
  const enc = polycodec.encodePolygon(poly);
  UnitTest(polycodec.decodePolygon(enc.o, enc.p), poly, "random-poly[" + t + "]");
}

section("base64 transport preserves bytes");
for (let t = 0; t < 50; t++) {
  const bytes = new Uint8Array(rndInt(1, 200));
  for (let i = 0; i < bytes.length; i++) { bytes[i] = rndInt(0, 256); }
  const back = polycodec.base64ToBytes(polycodec.bytesToBase64(bytes));
  UnitTest(Array.from(back), Array.from(bytes), "base64[" + t + "]");
}

console.log("tzlookup / portability");

section("B11: cell midpoints are exact at every depth, so >>1 == floor == trunc");
// This is the property that lets a C or Rust port write (lo + hi) >> 1 verbatim
// and still agree with this encoder.  It holds because the domain extents are
// powers of two and cells are half-open, so lo + hi is always even.
function checkMidpointExact(cell, depth) {
  for (let axis = 0; axis < 2; axis++) {
    const sum = cell[0][axis] + cell[1][axis];
    UnitTest(sum % 2, 0, "depth " + depth + " axis " + axis + " sum is even");
    UnitTest(sum >> 1, Math.floor(sum / 2), "depth " + depth + " axis " + axis + " >>1 == floor");
    UnitTest(sum >> 1, Math.trunc(sum / 2), "depth " + depth + " axis " + axis + " >>1 == trunc");
  }
}
// Exhaustive over the shallow levels...
(function checkAll(cell, depth) {
  checkMidpointExact(cell, depth);
  if (depth >= 7) { return; }
  for (let q = 0; q < 4; q++) { checkAll(tzlookup.splitCell(cell, q), depth + 1); }
})(tzmap.ROOT_CELL, 0);
// ...and along random descents all the way to MAX_DEPTH.
for (let t = 0; t < 2000; t++) {
  let cell = tzmap.ROOT_CELL;
  for (let d = 0; d <= tzmap.MAX_DEPTH; d++) {
    checkMidpointExact(cell, d);
    cell = tzlookup.splitCell(cell, rndInt(0, 4));
  }
}
// At MAX_DEPTH the narrow axis must still be at least 2 units wide, otherwise
// the midpoint stops being exact and the >>1 == floor == trunc identity breaks.
UnitTest((tzmap.Y_MAX - tzmap.Y_MIN) >> tzmap.MAX_DEPTH >= 2, true,
  "MAX_DEPTH leaves the narrow axis >= 2 units");

section("quadrants tile their parent exactly, with no gaps or overlaps");
(function checkTiling(cell, depth) {
  const q = [0, 1, 2, 3].map(function(i) { return tzlookup.splitCell(cell, i); });
  const mx = (cell[0][0] + cell[1][0]) >> 1, my = (cell[0][1] + cell[1][1]) >> 1;
  UnitTest(q[0], [[mx, my], [cell[1][0], cell[1][1]]], "quadrant 0 (NE)");
  UnitTest(q[1], [[cell[0][0], my], [mx, cell[1][1]]], "quadrant 1 (NW)");
  UnitTest(q[2], [[cell[0][0], cell[0][1]], [mx, my]], "quadrant 2 (SW)");
  UnitTest(q[3], [[mx, cell[0][1]], [cell[1][0], my]], "quadrant 3 (SE)");
  if (depth >= 6) { return; }
  q.forEach(function(c) { checkTiling(c, depth + 1); });
})(tzmap.ROOT_CELL, 0);

section("quadrantForPoint agrees with the cell bounds it selects");
for (let t = 0; t < 5000; t++) {
  const p = [rndInt(tzmap.X_MIN, tzmap.X_MAX), rndInt(tzmap.Y_MIN, tzmap.Y_MAX)];
  let cell = tzmap.ROOT_CELL;
  for (let d = 0; d < tzmap.MAX_DEPTH; d++) {
    cell = tzlookup.splitCell(cell, tzlookup.quadrantForPoint(cell, p));
    UnitTest(p[0] >= cell[0][0] && p[0] < cell[1][0] &&
             p[1] >= cell[0][1] && p[1] < cell[1][1], true,
      "point stays inside its chosen cell at depth " + d);
  }
}

console.log("quadtree");

function rectCCW(aabb) {
  return [
    [aabb[0][0], aabb[0][1]], [aabb[1][0], aabb[0][1]],
    [aabb[1][0], aabb[1][1]], [aabb[0][0], aabb[1][1]]
  ];
}
function rectCW(aabb) { return rectCCW(aabb).slice().reverse(); }

// The fixture the original manualTest() described; it never ran because it
// referenced an undefined `tzconvert` binding inside its own module (B13).
const FIXTURE = [
  [[100, 100], [1000, 1000]],
  [[-1000, -1000], [-100, -100]],
  [[-1000, 100], [-100, 1000]],
  [[-100, -100], [100, 100]],
  [[100, -1000], [1000, -100]],
  [[-100, -1000], [100, -100]],
  [[-1000000, -600000], [1000000, 600000]]   // encloses the whole root cell
];

function buildFixture(mk) {
  const out = tzconvert.newOutput();
  FIXTURE.forEach(function(r, i) {
    tzconvert.appendZone(out, mk(r), [], "zone" + i);
  });
  tzconvert.annotateLeaves(out);
  return out;
}

// Every leaf below MAX_DEPTH must hold at most SPLIT_THRESHOLD candidates.
function maxLeafRefsBelowMaxDepth(out) {
  let worst = 0;
  (function walk(node, depth) {
    if (node.q && node.q.length > 0) {
      node.q.forEach(function(q, i) { walk(q, depth + 1); });
    } else if (depth < tzmap.MAX_DEPTH) {
      worst = Math.max(worst, node.ref.length);
    }
  })(out.quadtree, 0);
  return worst;
}

section("B1 regression: the tree is identical for clockwise and counter-clockwise input");
const ccw = buildFixture(rectCCW), cw = buildFixture(rectCW);
UnitTest(tzconvert.treeStats(cw), tzconvert.treeStats(ccw), "winding-invariant tree shape");
UnitTest(tzconvert.treeStats(ccw).erefs > 0, true, "the enclosing rect produces erefs");

section("B1 regression: a large clockwise zone covers its own interior");
// This is the exact failure that made a probe of Kansas return zero candidates:
// the enclosing polygon scored -4, matched neither branch, and was dropped.
[[rectCCW, "ccw"], [rectCW, "cw"]].forEach(function(pair) {
  const out = buildFixture(pair[0]);
  const hit = tzlookup.probe(out.quadtree, out.rootCell, [-300000, -150000]);
  UnitTest(hit.definite.length > 0, true, pair[1] + ": deep interior resolves definitively");
  UnitTest(hit.definite[0].id, 6, pair[1] + ": resolves to the enclosing zone");
});

section("B1 regression: erefs are produced below the root, for both windings");
// Cluster enough small polygons to force deep subdivision, then lay a large
// rectangle over the same region.  It must eref on the sub-cells it covers --
// under the old signed-sum test a clockwise rectangle produced none.
[[rectCCW, "ccw"], [rectCW, "cw"]].forEach(function(pair) {
  const mk = pair[0], out = tzconvert.newOutput();
  for (let i = 0; i < 12; i++) {
    const x = 200000 + (i % 4) * 400, y = 100000 + Math.floor(i / 4) * 400;
    tzconvert.appendZone(out, mk([[x, y], [x + 200, y + 200]]), [], "small" + i);
  }
  tzconvert.appendZone(out, mk([[150000, 60000], [300000, 160000]]), [], "big");
  const stats = tzconvert.treeStats(out);
  tzconvert.annotateLeaves(out);
  UnitTest(stats.maxDepth > 0, true, pair[1] + ": tree subdivided");
  UnitTest(stats.erefs > 0, true, pair[1] + ": large zone erefs below the root");
  UnitTest(tzconvert.verify(out, 2000), 0, pair[1] + ": agrees with brute force");
});

section("classify() handles holes");
{
  const outer = rectCCW([[-1000, -1000], [1000, 1000]]);
  const hole = rectCCW([[-500, -500], [500, 500]]);
  const zone = {
    id: 0, outer: outer, holes: [hole],
    aabb: geom.PolyAABB(outer), holeAABBs: [geom.PolyAABB(hole)], tzid: 0
  };
  // A cell wholly inside the hole is not covered by the zone at all.
  UnitTest(tzconvert.classify([[-100, -100], [100, 100]], zone), tzconvert.REL_DISJOINT);
  // A cell straddling the hole boundary needs a geometry test.
  UnitTest(tzconvert.classify([[-600, -600], [-400, -400]], zone), tzconvert.REL_CANDIDATE);
  // A cell inside the ring but clear of the hole is definitive.
  UnitTest(tzconvert.classify([[-900, -900], [-700, -700]], zone), tzconvert.REL_DEFINITE);
  // Holes must be winding-agnostic too.
  const zoneRev = Object.assign({}, zone, {holes: [hole.slice().reverse()]});
  UnitTest(tzconvert.classify([[-100, -100], [100, 100]], zoneRev), tzconvert.REL_DISJOINT);
}

section("B4 regression: a hole is punched out, not filled in");
{
  const out = tzconvert.newOutput();
  const outer = rectCCW([[-100000, -100000], [100000, 100000]]);
  const hole = rectCCW([[-20000, -20000], [20000, 20000]]);
  tzconvert.appendZone(out, outer, [hole], "donut");
  const db = {rootCell: out.rootCell, quadtree: out.quadtree, zones: out.zones};

  // Inside the ring but outside the hole -> covered.
  let hit = tzlookup.probe(db.quadtree, db.rootCell, [50000, 50000]);
  UnitTest(hit.definite.length + hit.candidates.length > 0, true, "ring interior is covered");
  // Inside the hole -> must never resolve definitively, and a geometry test
  //   must reject it.
  hit = tzlookup.probe(db.quadtree, db.rootCell, [0, 0]);
  UnitTest(hit.definite.length, 0, "a point in the hole has no definitive hit");
  UnitTest(geom.RingsContainPoint(outer, [hole], [0, 0]), false, "hole rejects the point");
}

section("overlapping zones resolve to the smallest (enclave beats its host)");
{
  // An enclave is stored twice: as its own polygon, and as a hole in the
  // surrounding zone.  The two copies simplify differently, so they overlap
  // slightly.  Model that here with a host whose hole is deliberately smaller
  // than the enclave, so both contain the probe point.
  const out = tzconvert.newOutput();
  const host = rectCCW([[-100000, -100000], [100000, 100000]]);
  const undersizedHole = rectCCW([[-4000, -4000], [4000, 4000]]);
  const enclave = rectCCW([[-5000, -5000], [5000, 5000]]);
  tzconvert.appendZone(out, host, [undersizedHole], "host");
  tzconvert.appendZone(out, enclave, [], "enclave");
  tzconvert.annotateLeaves(out);
  tzconvert.purge(out);

  const db = {rootCell: out.rootCell, quadtree: out.quadtree, zones: out.zones};
  // A point in the overlap band: inside the enclave and inside the host proper.
  const p = [4500, 0];
  UnitTest(geom.RingsContainPoint(host, [undersizedHole], p), true, "host contains the point");
  UnitTest(geom.PolyContainsPoint(enclave, p), true, "enclave contains the point");
  UnitTest(tzlookup.resolve(db, p).zone.tzid, out.tz["enclave"].id,
    "the smaller zone wins");
}

section("a smaller candidate beats a larger enclosing eref (overlap, Kashmir case)");
{
  // The large zone becomes an `eref` for interior leaves; the smaller zone
  // overlaps it and is a `ref` candidate.  A point inside both must resolve to
  // the smaller one -- resolve() must not short-circuit on the eref.  This is
  // the exact failure the real build surfaced in the disputed Kashmir region.
  const out = tzconvert.newOutput();
  const bigZone = rectCCW([[-200000, -120000], [200000, 120000]]);
  const smallZone = rectCCW([[0, 0], [40000, 40000]]);       // smaller area, inside big
  tzconvert.appendZone(out, bigZone, [], "big");
  tzconvert.appendZone(out, smallZone, [], "small");
  // Clutter near the small zone's right edge forces deep subdivision, so a leaf
  //   there is fully inside `big` (eref) while `small` only crosses it (ref).
  let n = 0;
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      const x = 36000 + i * 700, y = 10000 + j * 700;
      tzconvert.appendZone(out, rectCCW([[x, y], [x + 1500, y + 1500]]), [], "c" + (n++));
    }
  }
  tzconvert.annotateLeaves(out);
  const db = {rootCell: out.rootCell, quadtree: out.quadtree, zones: out.zones};

  // A point inside `small` (and inside `big`), near small's right edge.
  const p = [39000, 20000];
  UnitTest(geom.PolyContainsPoint(bigZone, p), true, "big contains the point");
  UnitTest(geom.PolyContainsPoint(smallZone, p), true, "small contains the point");
  const res = tzlookup.resolve(db, p);
  UnitTest(res.zone.tzid, out.tz["small"].id, "the smaller overlapping zone wins over the eref");
  UnitTest(tzconvert.verify(out, 3000), 0, "agrees with brute force");
}

section("quadtree agrees with brute force over random geometry");
{
  const out = tzconvert.newOutput();
  // A pile of overlapping-free rectangles in a grid, alternating winding.
  let n = 0;
  for (let gx = 0; gx < 6; gx++) {
    for (let gy = 0; gy < 4; gy++) {
      const x0 = -400000 + gx * 130000, y0 = -200000 + gy * 100000;
      const r = [[x0 + 5000, y0 + 5000], [x0 + 120000, y0 + 90000]];
      tzconvert.appendZone(out, (n % 2)? rectCW(r) : rectCCW(r), [], "grid" + n);
      n++;
    }
  }
  tzconvert.annotateLeaves(out);
  UnitTest(maxLeafRefsBelowMaxDepth(out) <= tzmap.SPLIT_THRESHOLD, true,
    "no leaf below max depth exceeds the split threshold");
  const failures = tzconvert.verify(out, 4000);
  UnitTest(failures, 0, "brute-force agreement over 4000 points");
}

section("leaves subdivide to at most SPLIT_THRESHOLD candidate polygons");
{
  UnitTest(tzmap.SPLIT_THRESHOLD, 2, "threshold is 2 as requested");
  // A dense cluster of overlapping rectangles forces deep subdivision.
  const out = tzconvert.newOutput();
  let n = 0;
  for (let i = 0; i < 8; i++) {
    for (let j = 0; j < 8; j++) {
      const x = 10000 + i * 900, y = 10000 + j * 900;
      tzconvert.appendZone(out, rectCCW([[x, y], [x + 3000, y + 3000]]), [], "r" + (n++));
    }
  }
  tzconvert.annotateLeaves(out);
  UnitTest(maxLeafRefsBelowMaxDepth(out) <= 2, true, "every leaf below max depth holds <= 2");
  UnitTest(tzconvert.treeStats(out).maxDepth > 1, true, "the cluster actually subdivided");
}

section("localized point-in-polygon tests only local edges and matches the full test");
{
  // One large many-vertex ring, plus a cluster of small polygons over part of it
  // to force fine subdivision there.  In each leaf the big ring should contribute
  // only the few edges that pass through that cell, not its whole boundary.
  const out = tzconvert.newOutput();
  const big = [];
  const N = 240, R = 300000;
  for (let i = 0; i < N; i++) {
    const t = 2 * Math.PI * i / N;
    big.push([Math.round(R * Math.cos(t)), Math.round(0.6 * R * Math.sin(t))]);
  }
  tzconvert.appendZone(out, big, [], "big");
  let n = 0;
  for (let i = 0; i < 5; i++) {
    for (let j = 0; j < 5; j++) {
      const x = 20000 + i * 1500, y = 20000 + j * 1500;
      tzconvert.appendZone(out, rectCCW([[x, y], [x + 4000, y + 4000]]), [], "s" + (n++));
    }
  }
  tzconvert.annotateLeaves(out);

  // Find leaves where the big ring is a candidate; its stored edge count must be
  // far below the full ring, and never exceed it.
  let maxLocalEdges = 0, sawLocalized = false;
  const bigId = out.tz["big"].id, bigZoneId = out.zones.find(function(z){return z.tzid==bigId;}).id;
  (function walk(node) {
    if (node.q && node.q.length > 0) { node.q.forEach(walk); return; }
    node.ref.forEach(function(cand) {
      if (cand.z !== bigZoneId) { return; }
      let e = 0;
      cand.e.forEach(function(r) { e += r[1] - r[0] + 1; });
      UnitTest(e <= N, true, "local edge count never exceeds the full ring");
      if (e > 0 && e < N / 4) { sawLocalized = true; }
      maxLocalEdges = Math.max(maxLocalEdges, e);
    });
  })(out.quadtree);
  UnitTest(sawLocalized, true, "at least one leaf tests a small subset of the ring");

  // The localized test must agree with the full winding-number test everywhere.
  UnitTest(tzconvert.verify(out, 4000), 0, "localized lookup agrees with brute force");
}

section("purge() produces a decodable artifact");
{
  const out = buildFixture(rectCW);
  tzconvert.purge(out);
  UnitTest(typeof out.zones[0].p, "string", "zones carry a base64 polygon");
  UnitTest(out.zones[0].outer, undefined, "raw rings are dropped");

  const db = {rootCell: out.rootCell, quadtree: out.quadtree, zones: out.zones};
  const res = tzlookup.resolve(db, [-300000, -150000]);
  UnitTest(res.zone != null, true, "resolves against the purged artifact");
  UnitTest(res.zone.id, 6, "resolves to the enclosing zone");

  // Decoded geometry must match what went in.
  const rings = tzlookup.zoneRings(out.zones[0]);
  UnitTest(rings.outer, rectCW(FIXTURE[0]), "decoded ring matches the input");
}

console.log("\nall tests passed (" + sections + " sections)");
