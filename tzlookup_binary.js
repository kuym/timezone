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

  // Each crossing arc: arcIndex (delta from previous) then local edge run count +
  // runs.  Returns [[arcIndex, [[first,last],...]], ...] as tzlookup expects.
  function readArcRuns(c, arcCount) {
    const out = new Array(arcCount);
    let cum = 0;
    for (let i = 0; i < arcCount; i++) {
      cum += c.q();
      const rc = c.q();
      out[i] = [cum, readRuns(c, rc)];
    }
    return out;
  }

  function parseCandidate(c) {
    const packed = c.q();
    const arcCount = packed >> 3;
    const windCode = (packed >> 1) & 3;
    const hasHoles = packed & 1;
    const w = windCode === 0 ? 0 : windCode === 1 ? -1 : windCode === 2 ? 1 : unzigzag(c.q());
    const e = readArcRuns(c, arcCount);
    const h = [];
    if (hasHoles) {
      const hc = c.q();
      for (let i = 0; i < hc; i++) {
        const hi = c.q();
        const hw = unzigzag(c.q());
        const hac = c.q();
        h.push({ i: hi, w: hw, e: readArcRuns(c, hac) });
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

  // A ring's arc references: count, then each (arcIndex<<1|reversed) as uintLE,
  // returned in the signed convention tzlookup.reconstructRing expects
  // (i forward, -i-1 reversed).
  function readArcRefs(c) {
    const n = c.q();
    const refs = new Array(n);
    for (let i = 0; i < n; i++) {
      const v = c.uintLE();
      refs[i] = (v & 1) ? -(v >> 1) - 1 : (v >> 1);
    }
    return refs;
  }

  function bufToStr(bytes) {
    if (typeof TextDecoder !== "undefined") { return new TextDecoder("utf-8").decode(bytes); }
    return Buffer.from(bytes).toString("utf8");
  }

  function BinaryReader(bytes) {
    this.b = bytes;
    if (bytes.length < 56 || String.fromCharCode(bytes[0], bytes[1], bytes[2], bytes[3]) !== MAGIC) {
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
    const arcCount = dv.getUint32(52, true);
    this.rootCell = [[this.xMin, this.yMin], [this.xMax, this.yMax]];

    const c = new Cursor(bytes);
    c.pos = 56;

    // section 1: quadtree
    this.quadtree = parseNode(c);

    // section 2: arcs (shared boundaries; decoded to vertices lazily).  Record
    // each arc's byte offset for seek reporting.
    this._arcMeta = new Array(arcCount);
    this._arcs = new Array(arcCount);
    this.arcOffset = new Array(arcCount);
    for (let i = 0; i < arcCount; i++) {
      this.arcOffset[i] = c.pos;
      const ox = c.coord(), oy = c.coord();
      const pLen = c.uintLE();
      this._arcMeta[i] = { o: [ox, oy], praw: c.take(pLen) };
    }
    // A lazily-decoding view of the arcs: `arcs[i]` decodes arc i on first access.
    // Passed to tzlookup, so only the arcs a lookup actually touches are decoded.
    const self = this;
    this._arcView = new Proxy(this._arcs, {
      get: function (t, k) {
        return (typeof k === "string" && /^\d+$/.test(k)) ? self._arc(+k) : t[k];
      }
    });

    // section 3: zones (rank order) — metadata + arc references.  Record each
    // record's byte offset so a lookup can report the file position it reads.
    // `outer`/`h` hold signed arc refs, matching the JSON schema tzlookup expects.
    this.zones = new Array(zoneCount);
    this.zoneOffset = new Array(zoneCount);
    for (let rank = 0; rank < zoneCount; rank++) {
      this.zoneOffset[rank] = c.pos;
      const tzid = c.q();
      const area = c.uintLE();
      const xLo = c.coord(), yLo = c.coord();
      const w = c.uintLE(), h = c.uintLE();
      const outer = readArcRefs(c);
      const holeCount = c.q();
      const holeRefs = new Array(holeCount);
      for (let i = 0; i < holeCount; i++) { holeRefs[i] = readArcRefs(c); }
      this.zones[rank] = {
        tzid: tzid, a: area, aabb: [[xLo, yLo], [xLo + w, yLo + h]],
        outer: outer, h: holeRefs,
      };
    }

    // section 4: tz names (indexed by tzid)
    const nNames = c.q();
    this.tzNames = new Array(tzCount);
    for (let i = 0; i < nNames; i++) {
      const len = c.q();
      this.tzNames[i] = bufToStr(c.take(len));
    }
  }

  // Decode one arc's vertices on first use.
  BinaryReader.prototype._arc = function (idx) {
    if (!this._arcs[idx]) {
      const a = this._arcMeta[idx];
      this._arcs[idx] = polycodec.decodePolygon(a.o, a.praw);
    }
    return this._arcs[idx];
  };

  // The byte offsets of the arcs a candidate's localized test reads — only the
  // arcs that cross the cell (named in `cand.e` / `cand.h`), not the whole ring.
  BinaryReader.prototype._candArcSeeks = function (zone, cand) {
    const self = this, offs = [];
    const g = function (s) { return s < 0 ? -s - 1 : s; };
    cand.e.forEach(function (ar) { offs.push(self.arcOffset[g(zone.outer[ar[0]])]); });
    (cand.h || []).forEach(function (hc) {
      hc.e.forEach(function (ar) { offs.push(self.arcOffset[g(zone.h[hc.i][ar[0]])]); });
    });
    return offs;
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
    const stats = { depth: hit.path.length, candidates: hit.candidates.length,
                    vertices: 0, reconVertices: 0, fallbacks: 0 };

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
    let zoneSeeks = 0, arcSeeks = 0;
    for (let i = 0; i < hit.definite.length; i++) { // eref zones (definitive)
      const rank = hit.definite[i];
      seeks.push(this.zoneOffset[rank]); // read the zone record (tzid, area)
      zoneSeeks++;
      const z = this.zones[rank];
      if (best === null || z.a < best.a) { best = z; }
    }
    for (let i = 0; i < hit.candidates.length; i++) {
      const cand = hit.candidates[i];
      const zone = this.zones[cand.z];
      seeks.push(this.zoneOffset[cand.z]); // the zone record (metadata + arc refs)
      zoneSeeks++;
      const arcOffs = this._candArcSeeks(zone, cand); // only the crossing arcs
      for (let a = 0; a < arcOffs.length; a++) { seeks.push(arcOffs[a]); }
      arcSeeks += arcOffs.length;
      // localContainsZone decodes only the crossing arcs (via the lazy view) and
      // accumulates stats.reconVertices / stats.vertices.
      if (tzlookup.localContainsZone(zone, hit.cell, cand, point, stats, this._arcView)) {
        if (best === null || zone.a < best.a) { best = zone; }
      }
    }

    // Cost model: 1 op per quadtree level, 1 op per arc vertex decoded (only the
    // arcs crossing the cell), and 2 ops per ring edge evaluated in the localized
    // point-in-polygon test.
    const ops = stats.depth + stats.reconVertices + 2 * stats.vertices;
    const definite = best !== null && hit.definite.length > 0 && hit.candidates.length === 0;
    return {
      tzid: best ? best.tzid : null,
      name: best ? this.tzNames[best.tzid] : null,
      x: x, y: y, definite: definite,
      depth: stats.depth, candidates: stats.candidates,
      vertices: stats.vertices, reconVertices: stats.reconVertices, ops: ops,
      seeks: seeks, seekCount: seeks.length,
      treeSeeks: treeSeeks, zoneSeeks: zoneSeeks, arcSeeks: arcSeeks,
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
      "   depth " + r.depth + "   candidates tested " + r.candidates);
    console.log("  ring reconstruction: " + r.reconVertices + " arc vertices decoded" +
      "   localized test: " + r.vertices + " edges" +
      "   ~" + r.ops + " ops (" + r.depth + " + " + r.reconVertices + " + 2×" + r.vertices + ")");
    console.log("  seeks performed: " + r.seekCount +
      " (" + r.treeSeeks + " quadtree-descent + " + r.zoneSeeks + " zone-record + " + r.arcSeeks + " arc)");
    console.log("  seek byte offsets: [" + r.seeks.join(", ") + "]\n");
  }
  console.log("lookups performed: " + points.length +
    (points.length ? "   avg ops/lookup: " + (totalOps / points.length).toFixed(1) +
      "   avg seeks/lookup: " + (totalSeeks / points.length).toFixed(1) : ""));
}
