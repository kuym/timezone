//! The `q` variable-length integer code, shared by the `quad` and `binary`
//! serializers.
//!
//! Three forms, selected by the top bits of the first byte; the encoded quantity
//! `q` relates to the raw field `a` as:
//!
//! ```text
//!   A  0aaaaaaa                            q = a           →   0 ..   127   (1 byte)
//!   B  10aaaaaa aaaaaaaa                   q = a + 128      → 128 .. 16511   (2 bytes)
//!   C  11aaaaaa aaaaaaaa aaaaaaaa          q = 2a + 16514   → even, 16514.. (3 bytes)
//! ```
//!
//! `q` is exact below 16512; larger values must be even (round up with
//! `representable`).  The maximum encodable `q` is [`Q_MAX`].

/// Largest `q` the 3-byte C form can represent.
pub const Q_MAX: u64 = 2 * 0x3F_FFFF + 16514;

/// Smallest representable `q` >= n (identity below 16512; rounds up to the next
/// even value in the form-C range).
pub fn representable(n: u64) -> u64 {
    if n <= 16511 {
        n
    } else {
        let r = n.max(16514);
        r + (r & 1) // make even
    }
}

pub fn write_q(buf: &mut Vec<u8>, q: u64) {
    if q <= 127 {
        buf.push(q as u8); // A: 0aaaaaaa
    } else if q <= 16511 {
        let a = q - 128; // 14 bits
        buf.push(0x80 | (a >> 8) as u8); // B: 10aaaaaa aaaaaaaa
        buf.push((a & 0xFF) as u8);
    } else {
        let a = (q - 16514) / 2; // 22 bits, even q only
        buf.push(0xC0 | (a >> 16) as u8); // C: 11aaaaaa aaaaaaaa aaaaaaaa
        buf.push(((a >> 8) & 0xFF) as u8);
        buf.push((a & 0xFF) as u8);
    }
}

pub fn read_q(bytes: &[u8], pos: &mut usize) -> u64 {
    let b0 = bytes[*pos] as u64;
    *pos += 1;
    if b0 < 0x80 {
        b0
    } else if b0 < 0xC0 {
        let a = ((b0 & 0x3F) << 8) | bytes[*pos] as u64;
        *pos += 1;
        a + 128
    } else {
        let a = ((b0 & 0x3F) << 16) | ((bytes[*pos] as u64) << 8) | bytes[*pos + 1] as u64;
        *pos += 2;
        2 * a + 16514
    }
}

/// ZigZag-map a signed value to an unsigned one (small magnitudes stay small),
/// so `write_q` can carry negatives (e.g. winding numbers).
pub fn zigzag(n: i64) -> u64 {
    ((n << 1) ^ (n >> 63)) as u64
}

pub fn unzigzag(z: u64) -> i64 {
    ((z >> 1) as i64) ^ -((z & 1) as i64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q_roundtrip() {
        for q in [0u64, 1, 127, 128, 129, 16511, 16514, 16516, 100000, Q_MAX] {
            let mut buf = Vec::new();
            write_q(&mut buf, q);
            let mut pos = 0;
            assert_eq!(read_q(&buf, &mut pos), q, "q={q}");
            assert_eq!(pos, buf.len());
        }
    }

    #[test]
    fn q_form_sizes() {
        let sz = |q| {
            let mut b = Vec::new();
            write_q(&mut b, q);
            b.len()
        };
        assert_eq!(sz(0), 1);
        assert_eq!(sz(127), 1);
        assert_eq!(sz(128), 2);
        assert_eq!(sz(16511), 2);
        assert_eq!(sz(16514), 3);
    }

    #[test]
    fn representable_is_exact_below_c() {
        for n in [0u64, 1, 8, 127, 128, 16511] {
            assert_eq!(representable(n), n);
        }
        assert_eq!(representable(16512), 16514);
        assert_eq!(representable(16515), 16516);
    }

    #[test]
    fn zigzag_roundtrip() {
        for n in [0i64, 1, -1, 2, -2, 100, -100, i32::MAX as i64, i32::MIN as i64] {
            assert_eq!(unzigzag(zigzag(n)), n, "n={n}");
        }
    }
}
