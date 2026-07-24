// Reader for the experimental "quad" binary format (see rust/src/serialize/quad.rs
// and rust/README.md).  Analogous to tzlookup.js, but for the polygon-free
// quadtree: every leaf resolves to exactly one timezone id, so a lookup is a
// pure integer cell descent with no geometry.
//
// Each lookup also reports the byte offsets it seeks to in the file — every node
// whose length prefix it reads while descending (the chosen child at each level
// plus the siblings it must skip over to reach it) — and how many such seeks it
// performed.  This exposes the file access pattern of a lookup.
//
// Format recap:
//   magic "TZQ3"                       4 bytes
//   section 1 — quadtree, one recursive node:
//       node := len:q
//               len == 0 -> ocean leaf: no timezone, nothing follows (1 byte)
//               len == 1 -> leaf:       tzid:q follows
//               len >= 2 -> internal:   `len` bytes = 4 child nodes (quadrants 0..3)
//   section 2 — tzid table:
//       count:q, then count × ( nameLen:q, UTF-8 name )   — indexed by tzid
//
//   `q` is the 3-form code (top bits of byte 0 select the form):
//       A  0aaaaaaa                     q = a           →   0 ..   127   (1 byte)
//       B  10aaaaaa aaaaaaaa            q = a + 128      → 128 .. 16511   (2 bytes)
//       C  11aaaaaa aaaaaaaa aaaaaaaa   q = 2a + 16514   → even, 16514.. (3 bytes)
//   tzids are sorted by descending reference count, so common timezones get low
//   ids (form A).  A table entry with an empty name is "no timezone" (ocean).

(function (root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.tzlookupQuad = factory();
  }
})(typeof self !== "undefined" ? self : this, function () {

  // The quantization domain is fixed by the format (the quad file stores no
  // quant block).  These match the Rust encoder's quant.rs constants.
  const X_MIN = -524288, X_MAX = 524288;
  const Y_MIN = -262144, Y_MAX = 262144;
  const X_SCALE = 524288 / 180, Y_SCALE = 262144 / 90;
  const MAGIC = "TZQ3";

  function QuadReader(bytes) {
    // Accept a Node Buffer, Uint8Array, or plain array.
    this.b = bytes;
    if (this.b.length < 4 || String.fromCharCode(this.b[0], this.b[1], this.b[2], this.b[3]) !== MAGIC) {
      throw Error("not a quad file (bad magic)");
    }
    this.rootPos = 4;

    // Parse the tzid table (real timezones only; ocean has no entry).
    let pos = this.skipNode(this.rootPos);
    const count = this.readQ(pos);
    pos = count.pos;
    this.names = [];
    for (let i = 0; i < count.value; i++) {
      const n = this.readQ(pos);
      pos = n.pos;
      let s = "";
      for (let j = 0; j < n.value; j++) { s += String.fromCharCode(this.b[pos + j]); }
      this.names.push(decodeUtf8(s));
      pos += n.value;
    }
    this.count = count.value; // real tzid count; a leaf of length 0 is ocean
  }

  // Read a `q` value at `pos`; returns {value, pos: next}.
  QuadReader.prototype.readQ = function (pos) {
    const b = this.b;
    const b0 = b[pos++];
    if (b0 < 0x80) {                       // A: 0aaaaaaa
      return { value: b0, pos: pos };
    }
    if (b0 < 0xC0) {                        // B: 10aaaaaa aaaaaaaa
      const a = ((b0 & 0x3F) << 8) | b[pos++];
      return { value: a + 128, pos: pos };
    }                                       // C: 11aaaaaa aaaaaaaa aaaaaaaa
    const a = ((b0 & 0x3F) << 16) | (b[pos] << 8) | b[pos + 1];
    return { value: 2 * a + 16514, pos: pos + 2 };
  };

  // Skip the node starting at `pos`; returns the position just past it.
  QuadReader.prototype.skipNode = function (pos) {
    const len = this.readQ(pos);
    if (len.value === 0) {
      return len.pos; // ocean leaf: nothing follows
    }
    if (len.value === 1) {
      return this.readQ(len.pos).pos; // non-ocean leaf: skip the tzid
    }
    return len.pos + len.value; // internal: skip the payload (incl. padding)
  };

  // Name for a tzid (or null for "no timezone").
  QuadReader.prototype.name = function (tzid) {
    return tzid < this.count ? this.names[tzid] : null;
  };

  // Resolve a longitude/latitude to a timezone, tracing file seeks.
  // Returns { tzid, name, x, y, depth, seeks: [byteOffset...], seekCount }.
  QuadReader.prototype.lookup = function (lon, lat) {
    const x = Math.round(X_SCALE * lon);
    const y = Math.round(Y_SCALE * lat);
    const seeks = [];
    let pos = this.rootPos;
    let cell = [[X_MIN, Y_MIN], [X_MAX, Y_MAX]];
    let depth = 0;

    for (;;) {
      seeks.push(pos); // seek: read this node's length prefix here
      const len = this.readQ(pos);
      if (len.value === 0) {
        // Ocean leaf: length 0, no tzid follows.
        return { tzid: this.count, name: null, x: x, y: y,
                 depth: depth, seeks: seeks, seekCount: seeks.length };
      }
      if (len.value === 1) {
        // Non-ocean leaf: the tzid follows the length.
        const tzid = this.readQ(len.pos).value;
        return { tzid: tzid, name: this.name(tzid), x: x, y: y,
                 depth: depth, seeks: seeks, seekCount: seeks.length };
      }
      // Internal node: pick the child quadrant containing the point, skipping the
      //   siblings before it (each skip reads that sibling's length prefix).
      const mx = (cell[0][0] + cell[1][0]) >> 1;
      const my = (cell[0][1] + cell[1][1]) >> 1;
      const q = x >= mx ? (y >= my ? 0 : 3) : (y >= my ? 1 : 2);
      pos = len.pos; // start of child 0
      for (let i = 0; i < q; i++) {
        seeks.push(pos); // seek: read a skipped sibling's length prefix
        pos = this.skipNode(pos);
      }
      cell = q === 0 ? [[mx, my], [cell[1][0], cell[1][1]]]
           : q === 1 ? [[cell[0][0], my], [mx, cell[1][1]]]
           : q === 2 ? [[cell[0][0], cell[0][1]], [mx, my]]
           :           [[mx, cell[0][1]], [cell[1][0], my]];
      depth++;
    }
  };

  function decodeUtf8(latin1) {
    // Names are ASCII/UTF-8; decode multibyte sequences read as raw bytes.
    if (typeof TextDecoder !== "undefined") {
      const arr = new Uint8Array(latin1.length);
      for (let i = 0; i < latin1.length; i++) { arr[i] = latin1.charCodeAt(i) & 0xFF; }
      return new TextDecoder("utf-8").decode(arr);
    }
    try { return decodeURIComponent(escape(latin1)); } catch (e) { return latin1; }
  }

  return {
    QuadReader: QuadReader,
    X_MIN: X_MIN, X_MAX: X_MAX, Y_MIN: Y_MIN, Y_MAX: Y_MAX,
    X_SCALE: X_SCALE, Y_SCALE: Y_SCALE,
  };
});

// --- CLI ---
if (typeof require !== "undefined" && typeof module !== "undefined" && require.main === module) {
  const fs = require("fs");
  const { QuadReader } = module.exports;

  const args = process.argv.slice(2);
  if (args.length < 2 || args[0] === "-h" || args[0] === "--help") {
    console.error(
      "Usage: node tzlookup_quad.js <file.quad> <lat,lon> [<lat,lon> ...]\n" +
      "\n" +
      "  Resolves each point and prints the timezone plus the file seek trace\n" +
      "  (the byte offset of every node whose length is read during the lookup).\n" +
      "\n" +
      "Example:\n" +
      "  node tzlookup_quad.js tz.quad 38.5,-98.5 51.5074,-0.1276 -33.87,151.2"
    );
    process.exit(args.length && args[0] !== "-h" && args[0] !== "--help" ? 1 : 0);
  }

  const reader = new QuadReader(fs.readFileSync(args[0]));
  console.log("loaded " + args[0] + " (" + reader.b.length + " bytes, " +
    reader.count + " timezones)\n");

  const points = args.slice(1);
  let totalSeeks = 0;
  for (const tok of points) {
    const parts = tok.split(",");
    if (parts.length !== 2) {
      console.error("skipping '" + tok + "': expected <lat,lon>");
      continue;
    }
    const lat = parseFloat(parts[0]);
    const lon = parseFloat(parts[1]);
    const r = reader.lookup(lon, lat);
    totalSeeks += r.seekCount;
    console.log("lat " + lat + ", lon " + lon +
      "  (quantized " + r.x + ", " + r.y + ")  ->  " +
      (r.name === null ? "(no timezone)" : r.name) +
      "  [tzid " + r.tzid + "]");
    console.log("  depth " + r.depth + "   seeks performed: " + r.seekCount);
    console.log("  seek byte offsets: [" + r.seeks.join(", ") + "]\n");
  }
  console.log("lookups performed: " + points.length +
    "   total seeks: " + totalSeeks +
    (points.length ? "   avg seeks/lookup: " + (totalSeeks / points.length).toFixed(1) : ""));
}
