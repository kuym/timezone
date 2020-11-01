const geom = require("./geom");
const tzmap = require("./tzmap");
const UnitTest = require("./unitTest");


function processPoly(poly) {
  return tzmap.SimplifyRDP(tzmap.Quantize(poly), (5 * 16));
}

function exportPoly(poly) {
  const vectorsInterleaved = geom.PolyToLoop(poly).flat();
  const lengthBytes = parseInt((vectorsInterleaved.length + 1) * 20 / 8);
  const b = Buffer.alloc(lengthBytes);
  let bi = 0;
  for(let i = 0; i < vectorsInterleaved.length; i += 2) {
    // If the point's coordinates both fit in 11 signed bits, store them as two 11-bit signed values
    //packed in three bytes.  Otherwise, store in two 19-bit values packed in five bytes.
    if( (vectorsInterleaved[i] <= 1023) && (vectorsInterleaved[i] >= -1024) &&
        (vectorsInterleaved[i + 1] <= 1023) && (vectorsInterleaved[i] >= -1024)) {
      // 10aaaaaa aaaaabbb bbbbbbbb
      b.writeUInt8((vectorsInterleaved[i] >> 5) & 0x3F | 0x80, bi);
      b.writeUInt8((vectorsInterleaved[i] << 3) & 0xF8 | (vectorsInterleaved[i + 1] >> 8) & 0x07, bi + 1);
      b.writeUInt8(vectorsInterleaved[i + 1] & 0xFF, bi + 2);
      bi += 3;
    } else {
      // (v0 << 20) | v1
      // 00aaaaaa aaaaaaaa aaaaabbb bbbbbbbb bbbbbbbb
      b.writeUInt8((vectorsInterleaved[i] >> 13) & 0x3F, bi + 0);
      b.writeUInt8((vectorsInterleaved[i] >> 5) & 0xFF, bi + 1);
      b.writeUInt8(((vectorsInterleaved[i] << 3) & 0xF8) | ((vectorsInterleaved[i + 1] >> 16) & 0x07), bi + 2);
      b.writeUInt8((vectorsInterleaved[i + 1] >> 8) & 0xFF, bi + 3);
      b.writeUInt8(vectorsInterleaved[i + 1] & 0xFF, bi + 4);
      bi += 5;
    }
    // Alternate encodings, if needed:
    // 01aaaaaa aaaaabbb bbbbbbbb bbbbbbbb
    // 10bbbbbb bbbbbaaa aaaaaaaa aaaaaaaa
  }
  return b.slice(0, bi);
}

function splitCell(cell, quadrant) {
  const mx = parseInt((cell[0][0] + cell[1][0]) / 2), my = parseInt((cell[0][1] + cell[1][1]) / 2);
  switch(quadrant) {
  case 0: return([[mx, my], [cell[1][0], cell[1][1]]]);
  case 1: return([[cell[0][0], my], [mx, cell[1][1]]]);
  case 2: return([[cell[0][0], cell[0][1]], [mx, my]]);
  case 3: return([[mx, cell[0][1]], [cell[1][0], my]]);
  }
}

function findRef(node, ref) {
  return node.q.some(function(q, i) {
    if(q.eref.indexOf(ref) != -1) {
      return true;
    }
    if(q.ref.indexOf(ref) != -1) {
      return true;
    }
    return findRef(q, ref);
  });
  return false;
}

// the quadtree works as follows:
//   quadrants are indexed 0-3 as cartesian quadrants I-IV and on the `.q` member
//   the `.ref` member contains a list of all polygon refs in `.q` at any depth
//   the `.eref` member is a list of all polygon refs which are fully contained by this quadtree node
function appendTZPolyInternal(node, cell, ref, depth, debugName) {
  console.log("appendTZPolyInternal(", node, cell, ref, depth, debugName, ")");
  const intersection = geom.AABBIntersectsPoly(cell, ref.poly);
  //DEBUG console.log("poly area:", geom.PolyLoopArea(geom.PolyToLoop(ref.poly)));
  if(intersection == 4) {
    // Poly fully encloses cell
    console.log(debugName + ": erefing " + ref.id);
    node.eref.push(ref);
  } else if(intersection > 0 || geom.AABBFullyEnclosesAABB(cell, ref.aabb)) {
    // Poly intersects cell or cell fully encloses poly
    if(node.q.length > 0) {
      // Delegate to children
      console.log(debugName + ": delegating " + ref.id);
      node.q.forEach(function(q, i) {
        appendTZPolyInternal(q, splitCell(cell, i), ref, depth + 1, debugName + i);
      });

      if(!findRef(node, ref)) {  // assert to ensure ref is preserved
        throw Error("ref not found when delegating");
      }

      //console.log(debugName + ": delegation complete " + ref.id);
    }
    else if(depth >= 16 || node.ref.length < 5) {
      // As a leaf, hold this reference
      //console.log(debugName + ": acquiring " + ref.id);
      node.ref.push(ref);
    } else {
      // Split this leaf
      //console.log(debugName + ": splitting qty=" + node.ref.length);
      node.q = [0, 1, 2, 3].map(function(i) {
        const n = {q: [], eref: [], ref: []};
        node.ref.forEach(function(splitRef) {
          appendTZPolyInternal(n, splitCell(cell, i), splitRef, depth + 1, debugName + i);
        });
        return appendTZPolyInternal(n, splitCell(cell, i), ref, depth + 1, debugName + i);
      });
      
      node.ref.forEach(function(ref) {
        if(!findRef(node, ref)) {  // assert to ensure refs are preserved
          function collect(node) {
            const a = [];
            node.q.forEach(function(q){a.push(q.ref); collect(q)});
            return a;
          }
          throw Error(debugName + ": ref not found when splitting: " + node.ref.map(function(r){return r.id;}).join(",") + " != " + collect(node).flat().map(function(r){return r.id;}).join(","));
        }
      });

      // Refs are delegated to quadrants but erefs (refs fully enclosing this cell) remain.
      node.ref = [];

      //console.log(debugName + ": split complete");
    }
  } else {
    //console.log("At quadtree path " + debugName + ", poly " + ref.id + " not fully enclosed by qt cell. Poly aabb:", ref.aabb, "cell:", cell, "intersection:", intersection);
  }
  return node;
}

function appendTZPoly(output, poly, name) {
  console.log("appendTZPoly", name);
  const aabb = geom.PolyAABB(poly);

  //debug: test aabb
  //for(let i = 0; i < poly.length)

  // Create a tz if one doesn't exist
  let tz = output.tz[name];
  if(!tz) {
    tz = output.tz[name] = {
      id: Object.keys(output.tz).length,
      n: name,
      ref: []
    };
  }

  // Create a ref
  const ref = {
    id: output.zones.length,
    poly: poly, // Dropped for export
    aabb: aabb,
    p: exportPoly(poly).toString("base64"),
    tzid: tz.id
  };
  output.zones.push(ref);
  tz.ref.push(ref.id);

  const ret = appendTZPolyInternal(output.quadtree, [[-524288, -262144], [524287, 262143]], ref, 0, "");
  //console.log("appended", ref.id, "into", ret);
}

function probeTZPoint(output, point) {
  function probeTZPointInternal(node, cell, point) {
    const hits = [];
    if (node.eref) {
      hits.push.apply(hits, node.eref);
    }
    if (node.ref) {
      hits.push.apply(hits, node.ref);
    }
    if (node.q && node.q.length > 0) {
      const q0 = splitCell(cell, 0);
      if (point[0] >= q0[0][0]) {
        if (point[1] >= q0[0][1]) {
          hits.push.apply(hits, probeTZPointInternal(node.q[0], splitCell(cell, 0), point));
        } else {
          hits.push.apply(hits, probeTZPointInternal(node.q[3], splitCell(cell, 3), point));
        }
      } else {
        if (point[1] >= q0[0][1]) {
          hits.push.apply(hits, probeTZPointInternal(node.q[1], splitCell(cell, 1), point));
        } else {
          hits.push.apply(hits, probeTZPointInternal(node.q[2], splitCell(cell, 2), point));
        }
      }
    }
    return hits;
  }

  return probeTZPointInternal(output.quadtree, [[-524288, -262144], [524287, 262143]], point);
}

function manualTest() {
  function rect(aabb) {return [
    [aabb[0][0], aabb[0][1]],
    [aabb[1][0], aabb[0][1]],
    [aabb[1][0], aabb[1][1]],
    [aabb[0][0], aabb[1][1]]
  ]};
  const tzOut = {quadtree: {q: [], eref: [], ref: []}, zones: [], tz: {}};
  tzconvert.appendTZPoly(tzOut, rect([[100, 100], [1000, 1000]]), "foo");
  tzconvert.appendTZPoly(tzOut, rect([[-1000, -1000], [-100, -100]]), "foo");
  tzconvert.appendTZPoly(tzOut, rect([[-1000, 100], [-100, 1000]]), "foo");
  tzconvert.appendTZPoly(tzOut, rect([[-100, -100], [100, 100]]), "foo");
  tzconvert.appendTZPoly(tzOut, rect([[100, -1000], [1000, -100]]), "foo");
  tzconvert.appendTZPoly(tzOut, rect([[-100, -1000], [100, -100]]), "foo");
  tzconvert.appendTZPoly(tzOut, rect([[-1000000, -1000000], [1000000, 1000000]]), "foo");
  return tzOut;
}

module.exports = {
  processPoly: processPoly,
  exportPoly: exportPoly,
  splitCell: splitCell,
  appendTZPoly: appendTZPoly,
};

if (require.main == module) {

  const tzdata = require("./data/combined.json"); // takes >10 seconds to load

  const tzOut = {
    quadtree: {
      q: [],
      eref: [],
      ref: []
    },
    zones: [],
    tz: {}
  };

  function insertZone(output, unprocessedPoly, name) {
    appendTZPoly(output, processPoly(unprocessedPoly), name);
  }

  // process the input dataset
  tzdata.features.forEach(function(tz) {
    console.log("=> Processing input timezone:", tz.properties);
    const type = tz.geometry.type;
    for(let i = 0; i < tz.geometry.coordinates.length; i++) {
      if(type == "MultiPolygon") {
        console.log("  > inserting MultiPolygon #", i);
        for(let j = 0; j < tz.geometry.coordinates[i].length; j++) {
          console.log("    > inserting MultiPolygon #", i, "poly #", j);
          insertZone(tzOut, tz.geometry.coordinates[i][j], tz.properties.tzid);
        }
      } else {
        console.log("  > inserting Polygon #", i);
        insertZone(tzOut, tz.geometry.coordinates[i], tz.properties.tzid);
      }
    }
  });

  if(0) {


  // test
  // pick a random point in the range
  function testPoint(output) {
    const quadtreeAABB = [[-524288, -262144], [524287, 262143]];
    const x = quadtreeAABB[0][0] + Math.random() * (quadtreeAABB[1][0] - quadtreeAABB[0][0]);
    const y = quadtreeAABB[0][1] + Math.random() * (quadtreeAABB[1][1] - quadtreeAABB[0][1]);

    const expected = output.zones.filter(function(z) {
      return geom.PolyContainsPoints(z.poly, [[x, y]])[0];
    }).map(function(v){return v.id;});

    const actual = probeTZPoint(output, [x, y]).map(function(v){return v.id;});

    console.log("testing point", [x, y], "expecting", expected, "got", actual);
    //UnitTest(expected, actual);
  }

  for(let i = 0; i < 100; i++) {
    testPoint(tzOut);
  }




  function purge(node) {
    node.ref = node.ref.map(function(ref) {
      return ref.id;
    });
    if(node.ref.length == 0) {
      delete node.ref;
    }
    node.eref = node.eref.map(function(eref) {
      return eref.id;
    });
    if(node.eref.length == 0) {
      delete node.eref;
    }
    node.q.forEach(function(q) {
      purge(q);
    });
    if(node.q.length == 0) {
      delete node.q;
    }
  }
  purge(tzOut.quadtree);

  tzOut.zones.forEach(function(z) {
    delete z.poly;
  });

  } else {
    console.log("skipping tetPoitn and purge phases")
  }


  process.stdout.write(JSON.stringify(tzOut));
}
