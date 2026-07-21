// Packed polygon codec, shared by the encoder (Node) and the viewer (browser).
//
// A polygon is stored as an explicit integer origin plus a stream of deltas
// between consecutive vertices.  Keeping the origin out of the stream is what
// makes the variable-width forms below safe: every value in the stream is a
// *delta* between adjacent (already simplified) vertices, so it is small, and
// no stream value ever has to hold a full-range absolute coordinate.
//
// The previous format seeded the delta walk from [0, 0], which made vector #0
// an absolute coordinate needing 21 signed bits while the widest form held only
// 19.  Every polygon whose first vertex was beyond +-90 degrees of longitude
// silently wrapped modulo 2^19 (e.g. x=400000 decoded as -124288).
//
// Wire forms, discriminated by the top two bits of the first byte:
//
//   11aaaaaa abbbbbbb                                     2 bytes,  7-bit pair
//   10aaaaaa aaaaabbb bbbbbbbb                            3 bytes, 11-bit pair
//   00aaaaaa aaaaaaaa aaaaabbb bbbbbbbb bbbbbbbb          5 bytes, 19-bit pair
//   01aaaaaa aaaaaaaa aaaaaaaa aaabbbbb bbbbbbbb bbbbbbbb 6 bytes, 23-bit pair
//
// All values are two's-complement signed and must be sign-extended on decode.
// The 23-bit form is a guaranteed fallback: the quantization domain spans 2^20
// units, so no delta can ever exceed it.

(function(root, factory) {
  if (typeof module === "object" && module.exports) {
    module.exports = factory();
  } else {
    root.polycodec = factory();
  }
})(typeof self !== "undefined" ? self : this, function() {

  // Widths in bits of the signed pair carried by each form, and the tag written
  //   into the top two bits of the first byte.
  const FORMS = [
    {bits: 7,  bytes: 2, tag: 0xC0},
    {bits: 11, bytes: 3, tag: 0x80},
    {bits: 19, bytes: 5, tag: 0x00},
    {bits: 23, bytes: 6, tag: 0x40}
  ];

  function fitsInBits(v, bits) {
    const limit = 1 << (bits - 1);
    return (v >= -limit) && (v <= (limit - 1));
  }

  // Smallest form that can carry both components of a delta.
  function selectForm(a, b) {
    for (let i = 0; i < FORMS.length; i++) {
      if (fitsInBits(a, FORMS[i].bits) && fitsInBits(b, FORMS[i].bits)) {
        return FORMS[i];
      }
    }
    // Unreachable for any delta inside the quantization domain.
    throw Error("delta [" + a + ", " + b + "] does not fit in any wire form");
  }

  // Encode an array of [dx, dy] deltas into a Uint8Array.
  function encodeVectors(vectors) {
    // Worst case is the 6-byte form for every vector.
    const buf = new Uint8Array(vectors.length * 6);
    let n = 0;

    for (let i = 0; i < vectors.length; i++) {
      const a = vectors[i][0], b = vectors[i][1];
      const form = selectForm(a, b);

      switch (form.bytes) {
      case 2:
        buf[n++] = form.tag | ((a >> 1) & 0x3F);
        buf[n++] = ((a & 0x01) << 7) | (b & 0x7F);
        break;
      case 3:
        buf[n++] = form.tag | ((a >> 5) & 0x3F);
        buf[n++] = ((a << 3) & 0xF8) | ((b >> 8) & 0x07);
        buf[n++] = b & 0xFF;
        break;
      case 5:
        buf[n++] = form.tag | ((a >> 13) & 0x3F);
        buf[n++] = (a >> 5) & 0xFF;
        buf[n++] = ((a << 3) & 0xF8) | ((b >> 16) & 0x07);
        buf[n++] = (b >> 8) & 0xFF;
        buf[n++] = b & 0xFF;
        break;
      case 6:
        buf[n++] = form.tag | ((a >> 17) & 0x3F);
        buf[n++] = (a >> 9) & 0xFF;
        buf[n++] = (a >> 1) & 0xFF;
        buf[n++] = ((a & 0x01) << 7) | ((b >> 16) & 0x7F);
        buf[n++] = (b >> 8) & 0xFF;
        buf[n++] = b & 0xFF;
        break;
      }
    }
    return buf.subarray(0, n);
  }

  // Decode a Uint8Array back into an array of [dx, dy] deltas.
  function decodeVectors(bytes) {
    const out = [];
    let i = 0;

    while (i < bytes.length) {
      const b0 = bytes[i], tag = b0 & 0xC0;
      let a, b;

      if (tag == 0xC0) {                    // 2 bytes, 7-bit pair
        const b1 = bytes[i + 1];
        a = ((b0 & 0x3F) << 1) | (b1 >> 7);
        b = b1 & 0x7F;
        a = (a << 25) >> 25;  b = (b << 25) >> 25;
        i += 2;
      } else if (tag == 0x80) {             // 3 bytes, 11-bit pair
        const b1 = bytes[i + 1], b2 = bytes[i + 2];
        a = ((b0 & 0x3F) << 5) | (b1 >> 3);
        b = ((b1 & 0x07) << 8) | b2;
        a = (a << 21) >> 21;  b = (b << 21) >> 21;
        i += 3;
      } else if (tag == 0x00) {             // 5 bytes, 19-bit pair
        const b1 = bytes[i + 1], b2 = bytes[i + 2], b3 = bytes[i + 3], b4 = bytes[i + 4];
        a = ((b0 & 0x3F) << 13) | (b1 << 5) | (b2 >> 3);
        b = ((b2 & 0x07) << 16) | (b3 << 8) | b4;
        a = (a << 13) >> 13;  b = (b << 13) >> 13;
        i += 5;
      } else {                              // 6 bytes, 23-bit pair
        const b1 = bytes[i + 1], b2 = bytes[i + 2], b3 = bytes[i + 3],
              b4 = bytes[i + 4], b5 = bytes[i + 5];
        a = ((b0 & 0x3F) << 17) | (b1 << 9) | (b2 << 1) | (b3 >> 7);
        b = ((b3 & 0x7F) << 16) | (b4 << 8) | b5;
        a = (a << 9) >> 9;  b = (b << 9) >> 9;
        i += 6;
      }
      out.push([a, b]);
    }
    return out;
  }

  // Encode a polygon (array of absolute [x, y] vertices) into {o, p}, where `o`
  //   is the origin and `p` is the packed delta stream.
  //
  // The encode is verified by decoding it again before returning; a codec bug
  //   must never be allowed to reach the artifact silently, which is exactly how
  //   the 19-bit overflow and the range-check typo survived for years.
  function encodePolygon(poly) {
    if (poly.length == 0) {
      return {o: [0, 0], p: new Uint8Array(0)};
    }
    const origin = [poly[0][0], poly[0][1]];
    const vectors = [];
    for (let i = 1; i < poly.length; i++) {
      vectors.push([poly[i][0] - poly[i - 1][0], poly[i][1] - poly[i - 1][1]]);
    }
    const bytes = encodeVectors(vectors);

    const check = decodePolygon(origin, bytes);
    if (check.length != poly.length) {
      throw Error("encodePolygon round-trip changed vertex count: " +
        poly.length + " -> " + check.length);
    }
    for (let i = 0; i < poly.length; i++) {
      if (check[i][0] != poly[i][0] || check[i][1] != poly[i][1]) {
        throw Error("encodePolygon round-trip mismatch at vertex " + i + ": [" +
          poly[i] + "] -> [" + check[i] + "]");
      }
    }
    return {o: origin, p: bytes};
  }

  // Rebuild absolute vertices from an origin and a packed delta stream.
  function decodePolygon(origin, bytes) {
    const vectors = decodeVectors(bytes);
    const poly = [[origin[0], origin[1]]];
    let x = origin[0], y = origin[1];
    for (let i = 0; i < vectors.length; i++) {
      poly.push([x += vectors[i][0], y += vectors[i][1]]);
    }
    return poly;
  }

  function bytesToBase64(bytes) {
    if (typeof Buffer !== "undefined") {
      return Buffer.from(bytes).toString("base64");
    }
    let s = "";
    for (let i = 0; i < bytes.length; i++) { s += String.fromCharCode(bytes[i]); }
    return btoa(s);
  }

  function base64ToBytes(b64) {
    if (typeof Buffer !== "undefined") {
      return new Uint8Array(Buffer.from(b64, "base64"));
    }
    const s = atob(b64), bytes = new Uint8Array(s.length);
    for (let i = 0; i < s.length; i++) { bytes[i] = s.charCodeAt(i); }
    return bytes;
  }

  return {
    FORMS: FORMS,
    fitsInBits: fitsInBits,
    encodeVectors: encodeVectors,
    decodeVectors: decodeVectors,
    encodePolygon: encodePolygon,
    decodePolygon: decodePolygon,
    bytesToBase64: bytesToBase64,
    base64ToBytes: base64ToBytes
  };
});
