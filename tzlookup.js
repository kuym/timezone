// Quadtree traversal and point resolution, shared by the encoder's self-test
// (Node) and the viewer (browser).  Operates on the exported artifact: quadtree
// nodes carry numeric zone ids, and zone geometry is base64-packed.
//
// splitCell() used to be copy-pasted into both tzconvert.js and tzview.html.
// Any divergence between the two silently produces wrong lookups near cell
// boundaries, so there is now exactly one copy.

(function(root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory(require("./polycodec"), require("./geom"));
  } else {
    root.tzlookup = factory(root.polycodec, root.geom);
  }
})(typeof self !== "undefined" ? self : this, function(polycodec, geom) {

  // Quadrants are indexed as cartesian quadrants I-IV:
  //   0 = +x +y (NE)   1 = -x +y (NW)   2 = -x -y (SW)   3 = +x -y (SE)
  //
  // Cells are half-open, [lo, hi) on both axes.  Every cell bound is a multiple
  // of that cell's power-of-two size, so lo + hi is always even and this
  // midpoint is exact: >>1, Math.floor and truncate-toward-zero all agree.  A C
  // or Rust port may write (lo + hi) >> 1 verbatim.  See the domain notes in
  // tzmap.js for why the half-open domain matters.
  function splitCell(cell, quadrant) {
    const mx = (cell[0][0] + cell[1][0]) >> 1, my = (cell[0][1] + cell[1][1]) >> 1;
    switch(quadrant) {
    case 0: return [[mx, my], [cell[1][0], cell[1][1]]];
    case 1: return [[cell[0][0], my], [mx, cell[1][1]]];
    case 2: return [[cell[0][0], cell[0][1]], [mx, my]];
    case 3: return [[mx, cell[0][1]], [cell[1][0], my]];
    }
  }

  // Which child quadrant of `cell` contains `point`, under half-open bounds.
  function quadrantForPoint(cell, point) {
    const mx = (cell[0][0] + cell[1][0]) >> 1, my = (cell[0][1] + cell[1][1]) >> 1;
    if (point[0] >= mx) {
      return (point[1] >= my)? 0 : 3;
    }
    return (point[1] >= my)? 1 : 2;
  }

  // Descend to the leaf containing `point`, collecting definitive hits (`eref`,
  // zones that fully enclose the cell) and candidates (`ref`, zones that merely
  // overlap it and still need a containment test).
  function probe(quadtree, rootCell, point) {
    const definite = [], candidates = [], path = [];
    let node = quadtree, cell = rootCell;

    while (node) {
      if (node.eref) { definite.push.apply(definite, node.eref); }
      if (node.ref) { candidates.push.apply(candidates, node.ref); }
      if (!node.q || node.q.length == 0) { break; }
      const i = quadrantForPoint(cell, point);
      path.push(i);
      cell = splitCell(cell, i);
      node = node.q[i];
    }
    return {definite: definite, candidates: candidates, path: path, cell: cell};
  }

  // Decode a zone's rings on first use and memoize them on the record.  Handles
  // both the exported artifact (origin + packed deltas in `o`/`p`/`h`) and the
  // builder's in-memory pre-export zones (`outer`/`holes` already decoded).
  function zoneRings(zone) {
    if (!zone._rings) {
      if (zone.outer) {
        zone._rings = {outer: zone.outer, holes: zone.holes || []};
      } else {
        zone._rings = {
          outer: polycodec.decodePolygon(zone.o, polycodec.base64ToBytes(zone.p)),
          holes: (zone.h || []).map(function(h) {
            return polycodec.decodePolygon(h.o, polycodec.base64ToBytes(h.p));
          })
        };
      }
    }
    return zone._rings;
  }

  function isLeft(p0, p1, p2) {
    return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
  }

  // Signed number of times the directed segment C->q crosses the ring edges
  // listed in `runs` (arrays of [firstEdge, lastEdge] inclusive index ranges).
  // Returns null if any orientation is degenerate (a vertex lies on C->q, or an
  // edge is collinear with it), signalling the caller to fall back to a full
  // point-in-polygon test rather than risk an off-by-one.
  //
  // `stats`, if given, accumulates `vertices` -- one per ring edge examined --
  // so a caller can report how much point-in-polygon work a lookup actually did.
  function segmentCrossings(C, q, ring, runs, stats) {
    const n = ring.length;
    let delta = 0;
    for (let r = 0; r < runs.length; r++) {
      for (let i = runs[r][0]; i <= runs[r][1]; i++) {
        if (stats) { stats.vertices++; }
        const A = ring[i], B = ring[(i + 1) % n];
        const o1 = isLeft(A, B, C), o2 = isLeft(A, B, q);
        if (o1 == 0 || o2 == 0) { return null; }
        if ((o1 > 0) == (o2 > 0)) { continue; }         // C and q on the same side
        const o3 = isLeft(C, q, A), o4 = isLeft(C, q, B);
        if (o3 == 0 || o4 == 0) { return null; }
        if ((o3 > 0) == (o4 > 0)) { continue; }         // A and B on the same side
        delta += (o2 > 0)? 1 : -1;
      }
    }
    return delta;
  }

  // Vertices a full-ring test would visit: one per edge of the outer ring and of
  // every hole.
  function fullRingVertices(rings) {
    let n = rings.outer.length;
    for (let i = 0; i < rings.holes.length; i++) { n += rings.holes[i].length; }
    return n;
  }

  // Center of a cell.  Cell bounds are half-open powers of two, so this is exact
  // (see the domain notes in tzmap.js) and matches the value used at build time.
  function cellCenter(cell) {
    return [(cell[0][0] + cell[1][0]) >> 1, (cell[0][1] + cell[1][1]) >> 1];
  }

  // Localized point-in-zone test.  Instead of walking every edge of the ring,
  // it corrects the precomputed winding number of the cell center by the signed
  // crossings of the segment center->point against only the edges that intersect
  // this cell.  This is exact: the segment lies inside the convex cell, so no
  // edge outside the cell can cross it, and wn(point) = wn(center) + crossings.
  //
  // `cand` carries, for the leaf's cell: `w` (winding of the cell center wrt the
  // full outer ring) and `e` (outer edge-index runs intersecting the cell), plus
  // `h` = [{i, w, e}] for each hole that crosses the cell (holes that do not
  // cross cannot change the answer for any point in the cell, so they are
  // omitted).  Falls back to the full test on any degeneracy.
  function localContainsZone(zone, cell, cand, point, stats) {
    const rings = zoneRings(zone);
    const C = cellCenter(cell);

    const outerDelta = segmentCrossings(C, point, rings.outer, cand.e, stats);
    if (outerDelta === null) {
      if (stats) { stats.fallbacks++; stats.vertices += fullRingVertices(rings); }
      return geom.RingsContainPoint(rings.outer, rings.holes, point);
    }
    if (cand.w + outerDelta == 0) { return false; }     // outside the exterior ring

    const holes = cand.h || [];
    for (let k = 0; k < holes.length; k++) {
      const hc = holes[k], holeRing = rings.holes[hc.i];
      const holeDelta = segmentCrossings(C, point, holeRing, hc.e, stats);
      if (holeDelta === null) {
        if (stats) { stats.fallbacks++; stats.vertices += fullRingVertices(rings); }
        return geom.RingsContainPoint(rings.outer, rings.holes, point);
      }
      if (hc.w + holeDelta != 0) { return false; }      // inside a hole
    }
    return true;
  }

  function pointInZone(zone, point, stats) {
    // Cheap reject before decoding anything.
    if (zone.aabb) {
      if (point[0] < zone.aabb[0][0] || point[0] > zone.aabb[1][0] ||
          point[1] < zone.aabb[0][1] || point[1] > zone.aabb[1][1]) {
        return false;
      }
    }
    const rings = zoneRings(zone);
    if (stats) { stats.vertices += fullRingVertices(rings); }
    return geom.RingsContainPoint(rings.outer, rings.holes, point);
  }

  // Of several zones covering the same point, the smallest wins.
  //
  // Zones can genuinely overlap.  An enclave is stored both as its own polygon
  // and as a hole in the surrounding zone, and the two copies of that shared
  // boundary are simplified independently (once as an exterior ring, once as an
  // interior one), so they do not come out identical.  That leaves a thin band
  // where both Europe/Vatican and Europe/Rome contain the point.  Returning
  // whichever candidate happened to be visited first resolved Vatican City to
  // Europe/Rome and Monaco to Europe/Paris.  The enclave is always the smaller
  // polygon, so area is the correct discriminator.
  function smallest(zones) {
    let best = null;
    for (let i = 0; i < zones.length; i++) {
      if (best === null || (zones[i].a || 0) < (best.a || 0)) { best = zones[i]; }
    }
    return best;
  }

  // An `eref` entry is a bare zone id in the exported artifact, but a zone
  // object in the builder's in-memory tree; a `ref` candidate is {z: id, ...}.
  function zoneOf(db, entry) {
    if (typeof entry == "number") { return db.zones[entry]; }
    if (entry.z !== undefined) { return db.zones[entry.z]; }
    return entry;
  }

  // Full resolution: quadtree descent to narrow the candidate set, then a
  // localized point-in-polygon test.  The quadtree alone is only a filter --
  // this second half is what the original demo was missing, which is why it
  // reported three timezones for London.
  //
  // Zones in this dataset can overlap (disputed regions such as Kashmir, and the
  // enclave-as-hole double representation), so `eref` -- a zone that provably
  // covers the whole leaf cell -- is a guaranteed match but not necessarily the
  // answer: a smaller candidate may also contain the point and, by the
  // smallest-wins rule, should win.  Every zone that contains the point is
  // either an eref on the descent path or a candidate at the leaf, so the
  // correct answer is the smallest of (erefs + passing candidates).  When there
  // are no candidates (the common case) this is still test-free.
  function resolve(db, point) {
    const hit = probe(db.quadtree, db.rootCell, point);
    const matches = [];

    // Cost accounting, so a caller can gauge the work a lookup took:
    //   depth      - quadtree levels descended (2 coordinate comparisons each)
    //   candidates - zones point-in-polygon-tested at the leaf
    //   vertices   - ring edges actually evaluated across those tests (the
    //                localized subsets, or a full ring on a degeneracy fallback)
    //   fallbacks  - tests that fell back to the full ring
    const stats = {depth: hit.path.length, candidates: hit.candidates.length,
                   vertices: 0, fallbacks: 0};

    // erefs contain the point by construction; no geometry test needed.
    for (let i = 0; i < hit.definite.length; i++) {
      matches.push(zoneOf(db, hit.definite[i]));
    }
    const definiteCount = matches.length;

    // Candidates carry per-leaf localization data (`.e`), so test only the ring
    //   edges that actually pass through this leaf cell.  Fall back to the whole
    //   ring if a candidate predates the annotation pass.
    for (let i = 0; i < hit.candidates.length; i++) {
      const cand = hit.candidates[i];
      const zone = zoneOf(db, cand);
      const inside = (cand && cand.e)?
        localContainsZone(zone, hit.cell, cand, point, stats) :
        pointInZone(zone, point, stats);
      if (inside) { matches.push(zone); }
    }

    const winner = smallest(matches);
    // `definite` reports whether the answer needed no polygon test at all: it
    //   came from an eref and no candidate had to be evaluated.
    const definite = winner !== null && definiteCount > 0 &&
      hit.candidates.length == 0;
    return {zone: winner, definite: definite, probe: hit, stats: stats};
  }

  return {
    splitCell: splitCell,
    quadrantForPoint: quadrantForPoint,
    probe: probe,
    zoneRings: zoneRings,
    pointInZone: pointInZone,
    localContainsZone: localContainsZone,
    cellCenter: cellCenter,
    zoneOf: zoneOf,
    resolve: resolve
  };
});
