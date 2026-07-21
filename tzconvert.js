const geom = require("./geom");
const tzmap = require("./tzmap");
const polycodec = require("./polycodec");
const tzlookup = require("./tzlookup");
const UnitTest = require("./unitTest");

// Tracing and the O(n) tree-walk invariant checks are far too expensive to run
// during a real build (the original unconditionally console.log'd the entire
// subtree on every recursive call, which dominated runtime by orders of
// magnitude).  Enable with DEBUG=1.
const DEBUG = !!process.env.DEBUG;

function trace() {
  if (DEBUG) { console.error.apply(console, arguments); }
}

// Simplification tolerance, in quantization units.  1 unit is ~38m at the
// equator.  Override with EPSILON=<units>.
//
// The original value of 80 (~3km) was wider than several whole timezones: it
// flattened Vatican City and Monaco into unusable slivers and erased the
// matching holes in the surrounding zone, so those points resolved to
// Europe/Rome and Europe/Paris.  Measured against the unsimplified geometry
// over 1500 sample points:
//
//   eps=80  0.43 MB  99.80% agreement  (Vatican and Monaco wrong)
//   eps=8   1.40 MB  100.0% agreement
//   eps=4   2.03 MB  100.0% agreement
//   eps=2   3.01 MB  100.0% agreement
//
// 8 (~300m) is the knee of that curve: it is the coarsest value that still
// resolves every microstate, and tightening further buys no measurable
// accuracy.
const DEFAULT_EPSILON = 8;

// How a zone relates to a quadtree cell.
const REL_DISJOINT  = 0;   // no overlap; the zone does not belong in this cell
const REL_CANDIDATE = 1;   // partial overlap; needs a point-in-polygon test
const REL_DEFINITE  = 2;   // the zone covers the entire cell; no test needed

function processRing(ring, epsilon) {
  return tzmap.SimplifyRDP(tzmap.Quantize(ring), epsilon);
}

// Pack a quantized ring into {o, p}: an explicit integer origin plus a base64
// delta stream.  encodePolygon() verifies its own round-trip and throws on
// mismatch, so a codec regression cannot reach the artifact silently.
function exportRing(ring) {
  const encoded = polycodec.encodePolygon(ring);
  return {o: encoded.o, p: polycodec.bytesToBase64(encoded.p)};
}

function splitCell(cell, quadrant) {
  return tzlookup.splitCell(cell, quadrant);
}

function aabbsOverlap(a, b) {
  return !(a[1][0] < b[0][0] || a[0][0] > b[1][0] ||
           a[1][1] < b[0][1] || a[0][1] > b[1][1]);
}

// Classify a zone against a cell, accounting for holes.
//
// A hole that covers the whole cell means the zone does not cover any of it,
// even though the exterior ring encloses it.  A hole that merely crosses the
// cell downgrades a would-be definitive hit to a candidate.  This is what keeps
// `eref` honest: `eref` is consumed at lookup time as "answer immediately, skip
// the geometry", so a hole inside an eref region would be an unrecoverable
// wrong answer.
function classify(cell, zone) {
  if (!aabbsOverlap(cell, zone.aabb)) {
    return REL_DISJOINT;
  }

  const outerRel = geom.AABBIntersectsPoly(cell, zone.outer);
  if (outerRel == geom.AABB_DISJOINT) {
    return REL_DISJOINT;
  }
  if (outerRel == geom.AABB_CROSSES) {
    return REL_CANDIDATE;
  }

  // The exterior ring encloses the whole cell; holes may still carve it out.
  let rel = REL_DEFINITE;
  for (let i = 0; i < zone.holes.length; i++) {
    if (!aabbsOverlap(cell, zone.holeAABBs[i])) { continue; }
    const holeRel = geom.AABBIntersectsPoly(cell, zone.holes[i]);
    if (holeRel == geom.AABB_CONTAINS) {
      return REL_DISJOINT;      // the cell lies entirely within a hole
    }
    if (holeRel == geom.AABB_CROSSES) {
      rel = REL_CANDIDATE;
    }
  }
  return rel;
}

// Does `ref` appear anywhere at or below `node`?  Debug-only invariant check;
// it walks the whole subtree, so it is O(n) per insertion.
function findRef(node, ref) {
  if (node.eref.indexOf(ref) != -1 || node.ref.indexOf(ref) != -1) {
    return true;
  }
  return node.q.some(function(q) { return findRef(q, ref); });
}

function newNode() {
  return {q: [], eref: [], ref: []};
}

// The quadtree works as follows:
//   quadrants are indexed 0-3 as cartesian quadrants I-IV on the `.q` member
//   the `.ref` member lists zones that partially overlap this cell (candidates)
//   the `.eref` member lists zones that fully cover this cell (definitive)
// Refs live only at leaves; erefs may live at any depth.
function insertZoneInternal(node, cell, zone, depth, debugName) {
  const rel = classify(cell, zone);

  if (rel == REL_DISJOINT) {
    return node;
  }

  if (rel == REL_DEFINITE) {
    trace(debugName + ": erefing " + zone.id);
    node.eref.push(zone);
    return node;
  }

  if (node.q.length > 0) {
    trace(debugName + ": delegating " + zone.id);
    node.q.forEach(function(q, i) {
      insertZoneInternal(q, splitCell(cell, i), zone, depth + 1, debugName + i);
    });
    if (DEBUG && !findRef(node, zone)) {
      throw Error(debugName + ": ref not found when delegating " + zone.id);
    }
    return node;
  }

  if (depth >= tzmap.MAX_DEPTH || node.ref.length < tzmap.SPLIT_THRESHOLD) {
    node.ref.push(zone);
    return node;
  }

  // Split this leaf: redistribute the refs it held, then insert the new zone.
  trace(debugName + ": splitting qty=" + node.ref.length);
  const held = node.ref;
  node.q = [0, 1, 2, 3].map(function() { return newNode(); });
  node.q.forEach(function(q, i) {
    const childCell = splitCell(cell, i);
    held.forEach(function(heldZone) {
      insertZoneInternal(q, childCell, heldZone, depth + 1, debugName + i);
    });
    insertZoneInternal(q, childCell, zone, depth + 1, debugName + i);
  });

  if (DEBUG) {
    held.concat([zone]).forEach(function(r) {
      if (!findRef(node, r)) {
        throw Error(debugName + ": ref " + r.id + " lost when splitting");
      }
    });
  }

  // Refs are delegated to the quadrants; erefs already at this node remain.
  node.ref = [];
  return node;
}

// Add one polygon (an exterior ring plus its holes, already quantized and
// simplified) to the output under timezone `name`.
function appendZone(output, outer, holes, name) {
  if (outer.length < 3) {
    trace("skipping degenerate ring for " + name);
    return null;
  }

  let tz = output.tz[name];
  if (!tz) {
    tz = output.tz[name] = {
      id: Object.keys(output.tz).length,
      n: name,
      ref: []
    };
  }

  const zone = {
    id: output.zones.length,
    outer: outer,                                    // dropped for export
    holes: holes,                                    // dropped for export
    aabb: geom.PolyAABB(outer),
    holeAABBs: holes.map(geom.PolyAABB),             // dropped for export
    // Unsigned doubled area, used at lookup time to break ties when several
    //   overlapping zones contain the same point (see tzlookup.smallest).
    a: Math.abs(tzmap.RingArea2(outer)),
    tzid: tz.id
  };
  output.zones.push(zone);
  tz.ref.push(zone.id);

  insertZoneInternal(output.quadtree, output.rootCell, zone, 0, "");
  return zone;
}

function newOutput() {
  return {
    // The artifact is self-describing: a consumer needs no compiled-in
    //   constants to convert degrees to quantized units or to walk the tree.
    quant: {
      xMin: tzmap.X_MIN, xMax: tzmap.X_MAX,
      yMin: tzmap.Y_MIN, yMax: tzmap.Y_MAX,
      xScale: tzmap.X_SCALE, yScale: tzmap.Y_SCALE,
      maxDepth: tzmap.MAX_DEPTH
    },
    rootCell: tzmap.ROOT_CELL,
    quadtree: newNode(),
    zones: [],
    tz: {}
  };
}

// Compress a sorted list of edge indices into inclusive [first, last] runs.
// Adjacent edges of a ring are usually contiguous where the ring enters and
// leaves a cell, so a few runs capture what would otherwise be a long index
// list.  Runs are kept linear (not wrapped across the n-1 -> 0 seam); a wrap
// simply becomes two runs, which costs nothing at lookup time.
function edgeRuns(indices) {
  const runs = [];
  for (let i = 0; i < indices.length; i++) {
    if (runs.length > 0 && indices[i] == runs[runs.length - 1][1] + 1) {
      runs[runs.length - 1][1] = indices[i];
    } else {
      runs.push([indices[i], indices[i]]);
    }
  }
  return runs;
}

// Edge indices of `ring` that intersect `cell` (touch or cross it).  Edge i runs
// from vertex i to vertex (i+1) mod n.
function ringEdgesInCell(ring, cell) {
  const idx = [];
  const n = ring.length;
  for (let i = 0; i < n; i++) {
    if (geom.SegmentIntersectsAABB(cell, ring[i], ring[(i + 1) % n]) !== false) {
      idx.push(i);
    }
  }
  return edgeRuns(idx);
}

// Walk the tree and attach localization data to every leaf candidate so the
// lookup can test only the ring edges that pass through the leaf cell.  For each
// (leaf, candidate zone) it records the winding number of the cell center wrt
// the full outer ring and the outer edge runs in the cell, plus the same for any
// hole that crosses the cell (holes that miss the cell cannot affect points in
// it).  Replaces each ref (a zone object) with {z, w, e, h?}.
function annotateLeaves(output) {
  let candidates = 0, storedRuns = 0, storedEdges = 0;

  (function walk(node, cell) {
    if (node.q && node.q.length > 0) {
      node.q.forEach(function(q, i) { walk(q, splitCell(cell, i)); });
      return;
    }
    const center = tzlookup.cellCenter(cell);
    node.ref = node.ref.map(function(zone) {
      const cand = {
        z: zone.id,
        w: geom.PolyWinding(zone.outer, center),
        e: ringEdgesInCell(zone.outer, cell)
      };
      const holes = [];
      for (let h = 0; h < zone.holes.length; h++) {
        if (!aabbsOverlap(cell, zone.holeAABBs[h])) { continue; }
        const runs = ringEdgesInCell(zone.holes[h], cell);
        if (runs.length == 0) { continue; }             // hole does not cross this cell
        holes.push({i: h, w: geom.PolyWinding(zone.holes[h], center), e: runs});
      }
      if (holes.length > 0) { cand.h = holes; }

      candidates++;
      cand.e.forEach(function(r) { storedRuns++; storedEdges += r[1] - r[0] + 1; });
      return cand;
    });
  })(output.quadtree, output.rootCell);

  return {candidates: candidates, runs: storedRuns, edges: storedEdges};
}

// Convert the annotated tree to its exported form: erefs become bare ids, ref
// candidates keep their localization data (their `.z` is already an id), empty
// members are dropped, and zone geometry is packed.  Requires annotateLeaves()
// to have run first so that `node.ref` holds candidate objects rather than zone
// objects.
function purge(output) {
  (function walk(node) {
    if (node.ref.length == 0) { delete node.ref; }      // candidates already {z, w, e, h?}
    node.eref = node.eref.map(function(z) { return z.id; });
    if (node.eref.length == 0) { delete node.eref; }
    node.q.forEach(walk);
    if (node.q.length == 0) { delete node.q; }
  })(output.quadtree);

  output.zones.forEach(function(z) {
    const packed = exportRing(z.outer);
    z.o = packed.o;
    z.p = packed.p;
    if (z.holes.length > 0) {
      z.h = z.holes.map(exportRing);
    }
    delete z.outer;
    delete z.holes;
    delete z.holeAABBs;
    delete z._rings;
  });
  return output;
}

function treeStats(output) {
  let nodes = 0, leaves = 0, refs = 0, erefs = 0, maxDepth = 0, maxLeaf = 0;
  (function walk(node, d) {
    nodes++;
    maxDepth = Math.max(maxDepth, d);
    const r = node.ref || [], e = node.eref || [];
    refs += r.length;
    erefs += e.length;
    if (!node.q || node.q.length == 0) {
      leaves++;
      maxLeaf = Math.max(maxLeaf, r.length);
    } else {
      node.q.forEach(function(q) { walk(q, d + 1); });
    }
  })(output.quadtree, 0);
  return {nodes: nodes, leaves: leaves, refs: refs, erefs: erefs,
          maxDepth: maxDepth, maxLeafRefs: maxLeaf};
}

module.exports = {
  REL_DISJOINT: REL_DISJOINT,
  REL_CANDIDATE: REL_CANDIDATE,
  REL_DEFINITE: REL_DEFINITE,
  DEFAULT_EPSILON: DEFAULT_EPSILON,
  processRing: processRing,
  exportRing: exportRing,
  splitCell: splitCell,
  classify: classify,
  appendZone: appendZone,
  newOutput: newOutput,
  annotateLeaves: annotateLeaves,
  purge: purge,
  treeStats: treeStats,
};


if (require.main == module) {
  const fs = require("fs");
  const epsilon = process.env.EPSILON? parseInt(process.env.EPSILON, 10) : DEFAULT_EPSILON;
  const outPath = process.argv[2] || "quadtree.json";

  const tzdata = require("./data/combined.json"); // takes >10 seconds to load
  const output = newOutput();

  let ringsIn = 0, holesIn = 0, skipped = 0, wrapped = 0;

  // GeoJSON: ring 0 of a polygon is the exterior boundary; rings 1..n are holes.
  // The original code inserted every ring as an independent solid polygon, so
  // all 255 interior rings in the dataset were filled in rather than punched
  // out.
  function insertPolygon(rings, name) {
    const outer = processRing(rings[0], epsilon);
    const holes = [];
    for (let i = 1; i < rings.length; i++) {
      const hole = processRing(rings[i], epsilon);
      if (hole.length >= 3) { holes.push(hole); holesIn++; }
    }
    ringsIn++;

    // B18: nothing in this pipeline splits polygons at the antimeridian.  A ring
    //   that wraps would get an AABB spanning the world and would be inserted as
    //   a candidate into a huge number of cells.  Flag it rather than silently
    //   producing a slow, wrong tree.
    const aabb = geom.PolyAABB(outer);
    if ((aabb[1][0] - aabb[0][0]) > (tzmap.X_MAX - tzmap.X_MIN) / 2) {
      wrapped++;
      console.error("WARNING: ring for " + name + " spans more than half the " +
        "world in longitude (x " + aabb[0][0] + ".." + aabb[1][0] + "); " +
        "antimeridian wrapping is not handled.");
    }

    if (appendZone(output, outer, holes, name) == null) { skipped++; }
  }

  console.error("Processing " + tzdata.features.length + " features, epsilon=" + epsilon);
  tzdata.features.forEach(function(tz, n) {
    const name = tz.properties.tzid;
    const g = tz.geometry;
    if (g.type == "MultiPolygon") {
      g.coordinates.forEach(function(rings) { insertPolygon(rings, name); });
    } else {
      insertPolygon(g.coordinates, name);
    }
    if ((n % 25) == 0) { console.error("  " + n + "/" + tzdata.features.length + " " + name); }
  });

  console.error("exterior rings: " + ringsIn + ", holes: " + holesIn +
    ", skipped: " + skipped + ", antimeridian-wrapped: " + wrapped);

  // Attach per-leaf edge subsets so lookups test only the ring edges that pass
  // through each leaf cell.
  const annot = annotateLeaves(output);
  console.error("annotation: " + JSON.stringify(annot));

  // Verify the built tree against brute force before writing it out.  This runs
  // after annotation so it exercises the localized lookup, and is the oracle
  // that would have caught the winding-order bug immediately.
  const samples = process.env.VERIFY? parseInt(process.env.VERIFY, 10) : 2000;
  if (samples > 0) {
    console.error("Verifying " + samples + " random points against brute force...");
    const failures = verify(output, samples);
    if (failures > 0) {
      throw Error(failures + "/" + samples + " lookups disagreed with brute force");
    }
    console.error("  all " + samples + " lookups agree");
  }

  const stats = treeStats(output);
  console.error("quadtree: " + JSON.stringify(stats));

  purge(output);
  fs.writeFileSync(outPath, JSON.stringify(output));
  console.error("wrote " + outPath + " (" + fs.statSync(outPath).size + " bytes)");
}

// Smallest-area zone from a list, matching tzlookup's tie-break rule.
function smallestZone(zones) {
  let best = null;
  for (let i = 0; i < zones.length; i++) {
    if (best === null || zones[i].a < best.a) { best = zones[i]; }
  }
  return best;
}

// Compare quadtree resolution against an exhaustive scan of every zone.
// Must run AFTER annotateLeaves so it exercises the localized lookup path (the
// per-leaf edge subsets), not just the tree descent.  Operates on the unpurged
// output, so zones still carry their decoded rings.
function verify(output, samples) {
  let failures = 0;
  // Deterministic LCG so a failing run can be reproduced.
  let seed = 12345;
  function rnd() {
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return seed / 0x7FFFFFFF;
  }

  const db = {
    rootCell: output.rootCell,
    quadtree: output.quadtree,
    zones: output.zones
  };

  for (let i = 0; i < samples; i++) {
    const x = Math.floor(tzmap.X_MIN + rnd() * (tzmap.X_MAX - tzmap.X_MIN));
    const y = Math.floor(tzmap.Y_MIN + rnd() * (tzmap.Y_MAX - tzmap.Y_MIN));
    const point = [x, y];

    // Brute force applies the same smallest-wins rule as the real lookup, so
    //   that overlapping zones are compared meaningfully rather than leniently.
    const containing = output.zones.filter(function(z) {
      return geom.RingsContainPoint(z.outer, z.holes, point);
    });
    const expected = smallestZone(containing);

    // The real localized resolution path.
    const actual = tzlookup.resolve(db, point).zone;

    const ok = (expected === null)? (actual === null) :
      (actual !== null && actual.a == expected.a && actual.tzid == expected.tzid);
    if (!ok) {
      failures++;
      if (failures <= 10) {
        console.error("  MISMATCH at [" + x + ", " + y + "]: brute force " +
          (expected? expected.id : null) + ", quadtree " + (actual? actual.id : null));
      }
    }
  }
  return failures;
}

module.exports.verify = verify;
