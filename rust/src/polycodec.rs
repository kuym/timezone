//! Packed polygon codec — port of polycodec.js.
//!
//! A ring is stored as an explicit integer origin plus a stream of signed
//! `(dx, dy)` deltas between consecutive vertices, so no stream value ever holds
//! a full-range absolute coordinate.  Each delta is packed in the smallest of
//! four variable-width forms, discriminated by the top two bits of the first
//! byte:
//!
//! ```text
//!   11aaaaaa abbbbbbb                                     2 bytes,  7-bit pair
//!   10aaaaaa aaaaabbb bbbbbbbb                            3 bytes, 11-bit pair
//!   00aaaaaa aaaaaaaa aaaaabbb bbbbbbbb bbbbbbbb          5 bytes, 19-bit pair
//!   01aaaaaa aaaaaaaa aaaaaaaa aaabbbbb bbbbbbbb bbbbbbbb 6 bytes, 23-bit pair
//! ```

use crate::geom::Pt;

struct Form {
    bits: u32,
    bytes: usize,
    tag: u8,
}

// Smallest form first.
const FORMS: [Form; 4] = [
    Form { bits: 7, bytes: 2, tag: 0xC0 },
    Form { bits: 11, bytes: 3, tag: 0x80 },
    Form { bits: 19, bytes: 5, tag: 0x00 },
    Form { bits: 23, bytes: 6, tag: 0x40 },
];

fn fits_in_bits(v: i64, bits: u32) -> bool {
    let limit = 1i64 << (bits - 1);
    v >= -limit && v <= (limit - 1)
}

fn select_form(a: i64, b: i64) -> &'static Form {
    for f in FORMS.iter() {
        if fits_in_bits(a, f.bits) && fits_in_bits(b, f.bits) {
            return f;
        }
    }
    // Unreachable for any delta inside the quantization domain.
    panic!("delta [{a}, {b}] does not fit any wire form");
}

#[inline]
fn sign_extend(raw: i64, bits: u32) -> i64 {
    let shift = 64 - bits;
    (raw << shift) >> shift
}

/// Encode `[dx, dy]` deltas into packed bytes.
pub fn encode_vectors(vectors: &[Pt]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(vectors.len() * 3);
    for v in vectors {
        let (a, b) = (v[0], v[1]);
        let form = select_form(a, b);
        match form.bytes {
            2 => {
                buf.push(form.tag | ((a >> 1) & 0x3F) as u8);
                buf.push((((a & 0x01) << 7) | (b & 0x7F)) as u8);
            }
            3 => {
                buf.push(form.tag | ((a >> 5) & 0x3F) as u8);
                buf.push((((a << 3) & 0xF8) | ((b >> 8) & 0x07)) as u8);
                buf.push((b & 0xFF) as u8);
            }
            5 => {
                buf.push(form.tag | ((a >> 13) & 0x3F) as u8);
                buf.push(((a >> 5) & 0xFF) as u8);
                buf.push((((a << 3) & 0xF8) | ((b >> 16) & 0x07)) as u8);
                buf.push(((b >> 8) & 0xFF) as u8);
                buf.push((b & 0xFF) as u8);
            }
            6 => {
                buf.push(form.tag | ((a >> 17) & 0x3F) as u8);
                buf.push(((a >> 9) & 0xFF) as u8);
                buf.push(((a >> 1) & 0xFF) as u8);
                buf.push((((a & 0x01) << 7) | ((b >> 16) & 0x7F)) as u8);
                buf.push(((b >> 8) & 0xFF) as u8);
                buf.push((b & 0xFF) as u8);
            }
            _ => unreachable!(),
        }
    }
    buf
}

/// Decode packed bytes back into `[dx, dy]` deltas.
pub fn decode_vectors(bytes: &[u8]) -> Vec<Pt> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < bytes.len() {
        let b0 = bytes[i] as i64;
        let tag = b0 & 0xC0;
        let (a, b);
        if tag == 0xC0 {
            let b1 = bytes[i + 1] as i64;
            a = sign_extend(((b0 & 0x3F) << 1) | (b1 >> 7), 7);
            b = sign_extend(b1 & 0x7F, 7);
            i += 2;
        } else if tag == 0x80 {
            let b1 = bytes[i + 1] as i64;
            let b2 = bytes[i + 2] as i64;
            a = sign_extend(((b0 & 0x3F) << 5) | (b1 >> 3), 11);
            b = sign_extend(((b1 & 0x07) << 8) | b2, 11);
            i += 3;
        } else if tag == 0x00 {
            let b1 = bytes[i + 1] as i64;
            let b2 = bytes[i + 2] as i64;
            let b3 = bytes[i + 3] as i64;
            let b4 = bytes[i + 4] as i64;
            a = sign_extend(((b0 & 0x3F) << 13) | (b1 << 5) | (b2 >> 3), 19);
            b = sign_extend(((b2 & 0x07) << 16) | (b3 << 8) | b4, 19);
            i += 5;
        } else {
            let b1 = bytes[i + 1] as i64;
            let b2 = bytes[i + 2] as i64;
            let b3 = bytes[i + 3] as i64;
            let b4 = bytes[i + 4] as i64;
            let b5 = bytes[i + 5] as i64;
            a = sign_extend(((b0 & 0x3F) << 17) | (b1 << 9) | (b2 << 1) | (b3 >> 7), 23);
            b = sign_extend(((b3 & 0x7F) << 16) | (b4 << 8) | b5, 23);
            i += 6;
        }
        out.push([a, b]);
    }
    out
}

/// Encode a polygon into `(origin, packed_bytes)`.  The encode is verified by
/// decoding it again before returning, so a codec bug can never silently reach
/// the artifact.
pub fn encode_polygon(poly: &[Pt]) -> (Pt, Vec<u8>) {
    if poly.is_empty() {
        return ([0, 0], Vec::new());
    }
    let origin = poly[0];
    let mut vectors = Vec::with_capacity(poly.len().saturating_sub(1));
    for i in 1..poly.len() {
        vectors.push([poly[i][0] - poly[i - 1][0], poly[i][1] - poly[i - 1][1]]);
    }
    let bytes = encode_vectors(&vectors);

    let check = decode_polygon(origin, &bytes);
    assert_eq!(check.len(), poly.len(), "encode_polygon round-trip changed vertex count");
    for (i, (&got, &want)) in check.iter().zip(poly.iter()).enumerate() {
        assert_eq!(got, want, "encode_polygon round-trip mismatch at vertex {i}");
    }
    (origin, bytes)
}

/// Rebuild absolute vertices from an origin and a packed delta stream.
pub fn decode_polygon(origin: Pt, bytes: &[u8]) -> Vec<Pt> {
    let vectors = decode_vectors(bytes);
    let mut poly = Vec::with_capacity(vectors.len() + 1);
    let (mut x, mut y) = (origin[0], origin[1]);
    poly.push([x, y]);
    for v in vectors {
        x += v[0];
        y += v[1];
        poly.push([x, y]);
    }
    poly
}

// --- standard base64 (RFC 4648, '+/' alphabet, '=' padding) ---

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

pub fn base64_encode(data: &[u8]) -> String {
    let mut s = String::with_capacity((data.len() + 2) / 3 * 4);
    for chunk in data.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = *chunk.get(1).unwrap_or(&0) as u32;
        let b2 = *chunk.get(2).unwrap_or(&0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        s.push(B64[((n >> 18) & 0x3F) as usize] as char);
        s.push(B64[((n >> 12) & 0x3F) as usize] as char);
        s.push(if chunk.len() > 1 { B64[((n >> 6) & 0x3F) as usize] as char } else { '=' });
        s.push(if chunk.len() > 2 { B64[(n & 0x3F) as usize] as char } else { '=' });
    }
    s
}

#[allow(dead_code)]
fn b64_val(c: u8) -> i32 {
    match c {
        b'A'..=b'Z' => (c - b'A') as i32,
        b'a'..=b'z' => (c - b'a' + 26) as i32,
        b'0'..=b'9' => (c - b'0' + 52) as i32,
        b'+' => 62,
        b'/' => 63,
        _ => -1,
    }
}

/// Decode standard base64.  Part of the codec API (symmetry with encode) and
/// exercised by tests, though the compressor binary only encodes.
#[allow(dead_code)]
pub fn base64_decode(s: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    let mut acc = 0u32;
    let mut nbits = 0u32;
    for &c in s.as_bytes() {
        let v = b64_val(c);
        if v < 0 {
            continue; // skip '=' and any whitespace
        }
        acc = (acc << 6) | v as u32;
        nbits += 6;
        if nbits >= 8 {
            nbits -= 8;
            out.push((acc >> nbits) as u8);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_far_from_origin() {
        for poly in [
            vec![[10, 10], [20, 20], [30, 10]],
            vec![[400000, 100000], [400010, 100010]],
            vec![[-524288, -262144], [-524200, -262100]],
            vec![[10, 10], [10, -2000], [500, 500]],
        ] {
            let (o, p) = encode_polygon(&poly);
            assert_eq!(decode_polygon(o, &p), poly);
        }
    }

    #[test]
    fn form_selection() {
        assert_eq!(encode_vectors(&[[0, 0]]).len(), 2);
        assert_eq!(encode_vectors(&[[63, -64]]).len(), 2);
        assert_eq!(encode_vectors(&[[64, 0]]).len(), 3);
        assert_eq!(encode_vectors(&[[1024, 0]]).len(), 5);
        assert_eq!(encode_vectors(&[[262144, 0]]).len(), 6);
    }

    #[test]
    fn base64_roundtrip() {
        for n in [0usize, 1, 2, 3, 4, 5, 100, 255] {
            let data: Vec<u8> = (0..n).map(|i| (i * 37 % 256) as u8).collect();
            assert_eq!(base64_decode(&base64_encode(&data)), data);
        }
    }
}
