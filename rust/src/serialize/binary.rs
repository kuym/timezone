//! Binary serializer — STUB.
//!
//! The compact, mmap-friendly binary format is not designed yet (see
//! ANALYSIS.md §6 for the sketch). This module exists so the binary path shares
//! the `Serializer` interface end-to-end: format selection, the build pipeline,
//! and file writing all work today. What it emits is only a self-describing
//! header; the node / zone / geometry sections are `TODO`.
//!
//! `is_complete()` returns false so the driver prints a prominent warning and
//! never lets a stub file masquerade as a real artifact.

use super::Serializer;
use crate::build::Output;
use crate::quant;

pub struct BinarySerializer;

// Bump when the on-disk layout changes.
const MAGIC: &[u8; 4] = b"TZQT";
const VERSION: u16 = 0; // 0 = pre-release stub

impl Serializer for BinarySerializer {
    fn serialize(&self, out: &Output) -> Result<Vec<u8>, String> {
        let mut w = Writer::new();

        // --- header (this much is real) ---
        w.bytes(MAGIC);
        w.u16(VERSION);

        // Self-describing quantization block, mirroring the JSON `quant` object,
        // so a future reader needs no compiled-in constants.
        w.i32(quant::X_MIN as i32);
        w.i32(quant::X_MAX as i32);
        w.i32(quant::Y_MIN as i32);
        w.i32(quant::Y_MAX as i32);
        w.f64(quant::X_SCALE);
        w.f64(quant::Y_SCALE);
        w.u16(quant::MAX_DEPTH as u16);

        // Counts, so a reader can size its sections.
        w.u32(out.zones.len() as u32);
        w.u32(out.tz.len() as u32);
        w.u32(out.arena.len() as u32);

        // --- sections (NOT YET IMPLEMENTED) ---
        // TODO: quadtree nodes  (breadth-first, child offsets or a rank/select
        //       bitmap; eref/ref lists as varint-delta zone-id runs)
        // TODO: zones           (fixed-size records: origin, aabb, tzid, area,
        //       offset+len into the geometry blob)
        // TODO: geometry blob   (the packed delta streams, with periodic absolute
        //       anchors so a candidate's edge runs can be seeked without decoding
        //       the whole ring — see §5b / §6)
        // TODO: string table    (timezone names, offset-indexed)
        // TODO: trailer         (section offset table + CRC)

        Ok(w.into_bytes())
    }

    fn format_name(&self) -> &'static str {
        "binary"
    }

    fn is_complete(&self) -> bool {
        false
    }
}

/// Little-endian byte writer — the primitive scaffolding the real binary
/// serializer will build on.
struct Writer {
    buf: Vec<u8>,
}
impl Writer {
    fn new() -> Writer {
        Writer { buf: Vec::new() }
    }
    fn bytes(&mut self, b: &[u8]) {
        self.buf.extend_from_slice(b);
    }
    fn u16(&mut self, v: u16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn into_bytes(self) -> Vec<u8> {
        self.buf
    }
}
