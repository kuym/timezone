// Reader for the `binary` serialization format (produced by the Rust
// `tzconvert --format=binary`).  Analogous to tzlookup.js, but reading the
// compact binary artifact instead of quadtree.json.
//
// The binary file holds the same cost-model quadtree and polygons as the JSON
// artifact, so the lookup is identical: descend the quadtree, collect definitive
// `eref` zones and candidate `ref` zones, run a localized point-in-polygon test
// on the candidates, and return the smallest-area zone containing the point.
// This module parses the binary into the same in-memory shapes tzlookup.js uses
// and reuses its primitives (probe / localContainsZone / cellCenter) plus
// polycodec.decodePolygon, so the resolution logic is shared, not duplicated.
//
// Format (see rust/src/serialize/binary.rs for the authority):
//   header (52 bytes): magic "TZQT", version:u16, quant (xMin/xMax/yMin/yMax:i32,
//     xScale/yScale:f64, maxDepth:u16), counts (zones/tz/nodes: u32)  — all LE
//   section 1 quadtree: node := bodyLen:q, hdr:q (bit0=internal, bit1=hasEref,
//     leaf: bits2+=refCount), [erefCount:q, erefCount×rankDelta:q],
//     internal → 4 children | leaf → refCount × candidate
//     candidate := zDelta:q, packed:q ((runCount<<3)|(windCode<<1)|hasHoles),
//       [w:zigzag-q if windCode==3], runCount×(gap:q,len:q),
//       [holeCount:q, holeCount×(i:q, w:zigzag-q, count:q, count×(gap:q,len:q))]
//   section 2 zones (rank order): tzid:q, area:uintLE, aabb (xLo:coord, yLo:coord,
//     w:uintLE, h:uintLE), ring (ox:coord, oy:coord, pLen:uintLE, p bytes),
//     holeCount:q, holeCount × ring
//   section 3 tz names: count:q, count × (nameLen:q, UTF-8 bytes)
//
//   q       = the 3-form varint (A 1-byte <128, B 2-byte <16512, C 3-byte even).
//   uintLE  = exact varint: q byte-count, then that many little-endian bytes.
//   coord   = unzigzag(uintLE)  (signed).
//   Zones are referenced by rank (frequency order); rank indexes the zones list.

(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory(require("./polycodec"), require("./tzlookup"));
  } else {
    root.tzlookupBinary = factory(root.polycodec, root.tzlookup);
  }
})(typeof self !== "undefined" ? self : this, function (polycodec, tzlookup) {

  const MAGIC = "TZQT";

  function unzigzag(z) {
    return (z % 2 === 0) ? z / 2 : -(z + 1) / 2;
  }

  // A forward cursor over the byte buffer.
  function Cursor(bytes) {
    this.b = bytes;
    this.pos = 0;
  }
  // The 3-form `q` varint.
  Cursor.prototype.q = function () {
    const b = this.b;
    const b0 = b[this.pos++];
    if (b0 < 0x80) { return b0; }
    if (b0 < 0xC0) { return (((b0 & 0x3F) << 8) | b[this.pos++]) + 128; }
    const a = ((b0 & 0x3F) << 16) | (b[this.pos] << 8) | b[this.pos + 1];
    this.pos += 2;
    return 2 * a + 16514;
  };
  // Exact unsigned varint: q byte-count, then that many LE bytes.
  Cursor.prototype.uintLE = function () {
    const n = this.q();
    let v = 0;
    for (let i = 0; i < n; i++) { v += this.b[this.pos + i] * Math.pow(2, 8 * i); }
    this.pos += n;
    return v;
  };
  Cursor.prototype.coord = function () {
    return unzigzag(this.uintLE());
  };
  Cursor.prototype.take = function (n) {
    const s = this.b.subarray(this.pos, this.pos + n);
    this.pos += n;
    return s;
  };

  function readRuns(c, count) {
    const runs = [];
    let prev = 0;
    for (let i = 0; i < count; i++) {
      const first = prev + c.q();
      const last = first + c.q();
      runs.push([first, last]);
      prev = last + 1;
    }
    return runs;
  }

  function parseCandidate(c) {
    const packed = c.q();
    const runCount = packed >> 3;
    const windCode = (packed >> 1) & 3;
    const hasHoles = packed & 1;
    const w = windCode === 0 ? 0 : windCode === 1 ? -1 : windCode === 2 ? 1 : unzigzag(c.q());
    const e = readRuns(c, runCount);
    const h = [];
    if (hasHoles) {
      const hc = c.q();
      for (let i = 0; i < hc; i++) {
        const hi = c.q();
        const hw = unzigzag(c.q());
        const hrc = c.q();
        h.push({ i: hi, w: hw, e: readRuns(c, hrc) });
      }
    }
    return { w: w, e: e, h: h }; // `z` filled in by the caller
  }

  // Parse one quadtree node into the JSON-shaped { q?, eref?, ref? } tzlookup
  // uses, tagged with `_off` = the byte offset of its length prefix (so a lookup
  // can report the file offsets a byte-navigating reader would seek to).
  function parseNode(c) {
    const off = c.pos;
    const bodyLen = c.q();
    const end = c.pos + bodyLen;
    const hdr = c.q();
    const isInternal = hdr & 1;
    const hasEref = hdr & 2;
    const node = { _off: off };

    if (hasEref) {
      const ec = c.q();
      const eref = [];
      let cum = 0;
      for (let i = 0; i < ec; i++) { cum += c.q(); eref.push(cum); }
      node.eref = eref;
    }
    if (isInternal) {
      node.q = [parseNode(c), parseNode(c), parseNode(c), parseNode(c)];
    } else {
      const refCount = hdr >> 2;
      const ref = [];
      let cum = 0;
      for (let i = 0; i < refCount; i++) {
        cum += c.q();
        const cand = parseCandidate(c);
        cand.z = cum; // rank
        ref.push(cand);
      }
      node.ref = ref;
    }
    c.pos = end; // skip padding
    return node;
  }

  function readRing(c) {
    const ox = c.coord();
    const oy = c.coord();
    const pLen = c.uintLE();
    return { o: [ox, oy], praw: c.take(pLen) };
  }

  function bufToStr(bytes) {
    if (typeof TextDecoder !== "undefined") { return new TextDecoder("utf-8").decode(bytes); }
    return Buffer.from(bytes).toString("utf8");
  }

  function BinaryReader(bytes) {
    this.b = bytes;
    if (bytes.length < 52 || String.fromCharCode(bytes[0], bytes[1], bytes[2], bytes[3]) !== MAGIC) {
      throw Error("not a binary tz file (bad magic)");
    }
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.version = dv.getUint16(4, true);
    this.xMin = dv.getInt32(6, true);
    this.xMax = dv.getInt32(10, true);
    this.yMin = dv.getInt32(14, true);
    this.yMax = dv.getInt32(18, true);
    this.xScale = dv.getFloat64(22, true);
    this.yScale = dv.getFloat64(30, true);
    this.maxDepth = dv.getUint16(38, true);
    const zoneCount = dv.getUint32(40, true);
    const tzCount = dv.getUint32(44, true);
    this.nodeCount = dv.getUint32(48, true);
    this.rootCell = [[this.xMin, this.yMin], [this.xMax, this.yMax]];

    const c = new Cursor(bytes);
    c.pos = 52;

    // section 1: quadtree
    this.quadtree = parseNode(c);

    // section 2: zones (rank order; rings decoded lazily).  Record each record's
    // byte offset so a lookup can report the file position of a zone it reads.
    this.zones = new Array(zoneCount);
    this.zoneOffset = new Array(zoneCount);
    for (let rank = 0; rank < zoneCount; rank++) {
      this.zoneOffset[rank] = c.pos;
      const tzid = c.q();
      const area = c.uintLE();
      const xLo = c.coord(), yLo = c.coord();
      const w = c.uintLE(), h = c.uintLE();
      const outer = readRing(c);
      const holeCount = c.q();
      const hraw = new Array(holeCount);
      for (let i = 0; i < holeCount; i++) { hraw[i] = readRing(c); }
      this.zones[rank] = {
        tzid: tzid, a: area, aabb: [[xLo, yLo], [xLo + w, yLo + h]],
        o: outer.o, praw: outer.praw, hraw: hraw,
      };
    }

    // section 3: tz names (indexed by tzid)
    const nNames = c.q();
    this.tzNames = new Array(tzCount);
    for (let i = 0; i < nNames; i++) {
      const len = c.q();
      this.tzNames[i] = bufToStr(c.take(len));
    }
  }

  // Decode a zone's rings on first use (memoized on the record, so tzlookup's
  // zoneRings picks them up from `.outer`/`.holes`).
  BinaryReader.prototype._rings = function (zone) {
    if (!zone.outer) {
      zone.outer = polycodec.decodePolygon(zone.o, zone.praw);
      zone.holes = zone.hraw.map(function (h) { return polycodec.decodePolygon(h.o, h.praw); });
    }
  };

  // Resolve a longitude/latitude to a timezone.  Mirrors tzlookup.resolve, but
  // decodes candidate polygons lazily from the binary and reports the file byte
  // offsets a byte-navigating reader would seek to during the lookup:
  //   - the quadtree nodes visited on the descent (the chosen child at each level
  //     plus the siblings skipped to reach it — the same trace tzlookup_quad.js
  //     produces), then
  //   - the zone records read for each eref / candidate (a jump into the zones
  //     section — the binary lookup's cross-section access that quad has no
  //     equivalent for).
  // Returns { tzid, name, x, y, definite, depth, candidates, vertices, ops,
  //           seeks, seekCount, treeSeeks, zoneSeeks }.
  BinaryReader.prototype.resolve = function (lon, lat) {
    const x = Math.round(this.xScale * lon);
    const y = Math.round(this.yScale * lat);
    const point = [x, y];

    const hit = tzlookup.probe(this.quadtree, this.rootCell, point);
    const stats = { depth: hit.path.length, candidates: hit.candidates.length, vertices: 0, fallbacks: 0 };

    // Quadtree-descent seeks: the root, then at each level the chosen child and
    // the siblings a skip-based reader would read to reach it.
    const seeks = [this.quadtree._off];
    let node = this.quadtree;
    for (let d = 0; d < hit.path.length; d++) {
      const q = hit.path[d];
      for (let i = 0; i <= q; i++) { seeks.push(node.q[i]._off); }
      node = node.q[q];
    }
    const treeSeeks = seeks.length;

    let best = null;
    for (let i = 0; i < hit.definite.length; i++) { // eref zones (definitive)
      const rank = hit.definite[i];
      seeks.push(this.zoneOffset[rank]); // read the zone record (tzid, area)
      const z = this.zones[rank];
      if (best === null || z.a < best.a) { best = z; }
    }
    for (let i = 0; i < hit.candidates.length; i++) {
      const cand = hit.candidates[i];
      seeks.push(this.zoneOffset[cand.z]); // read the zone record (incl. polygon)
      const zone = this.zones[cand.z];
      this._rings(zone);
      if (tzlookup.localContainsZone(zone, hit.cell, cand, point, stats)) {
        if (best === null || zone.a < best.a) { best = zone; }
      }
    }

    const definite = best !== null && hit.definite.length > 0 && hit.candidates.length === 0;
    return {
      tzid: best ? best.tzid : null,
      name: best ? this.tzNames[best.tzid] : null,
      x: x, y: y, definite: definite,
      depth: stats.depth, candidates: stats.candidates, vertices: stats.vertices,
      ops: stats.depth + 2 * stats.vertices,
      seeks: seeks, seekCount: seeks.length, treeSeeks: treeSeeks, zoneSeeks: seeks.length - treeSeeks,
    };
  };

  return { BinaryReader: BinaryReader };
});

// --- CLI ---
if (typeof require !== "undefined" && typeof module !== "undefined" && require.main === module) {
  const fs = require("fs");
  const { BinaryReader } = module.exports;

  const args = process.argv.slice(2);
  if (args.length < 2 || args[0] === "-h" || args[0] === "--help") {
    console.error(
      "Usage: node tzlookup_binary.js <file.bin> <lat,lon> [<lat,lon> ...]\n" +
      "\n" +
      "  Resolves each point against a `binary`-format artifact and prints the\n" +
      "  timezone plus the lookup cost (descent depth, candidates tested, polygon\n" +
      "  vertices evaluated).\n" +
      "\n" +
      "Example:\n" +
      "  node tzlookup_binary.js tz.bin 38.5,-98.5 51.5074,-0.1276 -33.87,151.2"
    );
    process.exit(args.length && args[0] !== "-h" && args[0] !== "--help" ? 1 : 0);
  }

  const reader = new BinaryReader(fs.readFileSync(args[0]));
  console.log("loaded " + args[0] + " (" + reader.b.length + " bytes, " +
    reader.zones.length + " polygons, " + reader.tzNames.length + " timezones)\n");

  const points = args.slice(1);
  let totalOps = 0, totalSeeks = 0;
  for (const tok of points) {
    const parts = tok.split(",");
    if (parts.length !== 2) { console.error("skipping '" + tok + "': expected <lat,lon>"); continue; }
    const lat = parseFloat(parts[0]);
    const lon = parseFloat(parts[1]);
    const r = reader.resolve(lon, lat);
    totalOps += r.ops;
    totalSeeks += r.seekCount;
    console.log("lat " + lat + ", lon " + lon +
      "  (quantized " + r.x + ", " + r.y + ")  ->  " +
      (r.name === null ? "(no timezone)" : r.name) +
      (r.tzid === null ? "" : "  [tzid " + r.tzid + "]"));
    console.log("  " + (r.definite ? "resolved from eref (no polygon test)" : "resolved by point-in-polygon") +
      "   depth " + r.depth + "   candidates tested " + r.candidates +
      "   vertices evaluated " + r.vertices + "   ~" + r.ops + " ops");
    console.log("  seeks performed: " + r.seekCount +
      " (" + r.treeSeeks + " quadtree-descent + " + r.zoneSeeks + " zone-record)");
    console.log("  seek byte offsets: [" + r.seeks.join(", ") + "]\n");
  }
  console.log("lookups performed: " + points.length +
    (points.length ? "   avg ops/lookup: " + (totalOps / points.length).toFixed(1) +
      "   avg seeks/lookup: " + (totalSeeks / points.length).toFixed(1) : ""));
}
