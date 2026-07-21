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

  // Decode a zone's rings on first use and memoize them on the record.
  function zoneRings(zone) {
    if (!zone._rings) {
      zone._rings = {
        outer: polycodec.decodePolygon(zone.o, polycodec.base64ToBytes(zone.p)),
        holes: (zone.h || []).map(function(h) {
          return polycodec.decodePolygon(h.o, polycodec.base64ToBytes(h.p));
        })
      };
    }
    return zone._rings;
  }

  function pointInZone(zone, point) {
    // Cheap reject before decoding anything.
    if (zone.aabb) {
      if (point[0] < zone.aabb[0][0] || point[0] > zone.aabb[1][0] ||
          point[1] < zone.aabb[0][1] || point[1] > zone.aabb[1][1]) {
        return false;
      }
    }
    const rings = zoneRings(zone);
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

  // Full resolution: quadtree descent to narrow the candidate set, then an
  // actual point-in-polygon test.  The quadtree alone is only a filter -- this
  // second half is what the original demo was missing, which is why it reported
  // three timezones for London.
  function resolve(db, point) {
    const hit = probe(db.quadtree, db.rootCell, point);

    // `eref` means the zone provably encloses the whole leaf cell, so no
    //   geometry test is needed.
    if (hit.definite.length > 0) {
      const zones = hit.definite.map(function(i) { return db.zones[i]; });
      return {zone: smallest(zones), definite: true, probe: hit};
    }

    const matches = [];
    for (let i = 0; i < hit.candidates.length; i++) {
      const zone = db.zones[hit.candidates[i]];
      if (pointInZone(zone, point)) { matches.push(zone); }
    }
    return {zone: smallest(matches), definite: false, probe: hit};
  }

  return {
    splitCell: splitCell,
    quadrantForPoint: quadrantForPoint,
    probe: probe,
    zoneRings: zoneRings,
    pointInZone: pointInZone,
    resolve: resolve
  };
});
