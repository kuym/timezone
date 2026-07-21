//! Output serialization behind a common interface.
//!
//! Both the JSON writer (complete) and the binary writer (a stub, until the
//! binary format is designed) implement the same `Serializer` trait, so the
//! build driver is agnostic to the output format and a new format is a matter of
//! adding one impl.

use crate::build::Output;

pub mod binary;
pub mod json;

/// Turns a built `Output` into a byte stream in some concrete format.
pub trait Serializer {
    /// Serialize the artifact, or return a human-readable error.
    fn serialize(&self, out: &Output) -> Result<Vec<u8>, String>;
    /// Short format name, for logging.
    fn format_name(&self) -> &'static str;
    /// Whether this serializer produces a complete artifact (vs. a stub).
    fn is_complete(&self) -> bool {
        true
    }
}

/// Select a serializer by name (`"json"` or `"binary"`).
pub fn for_format(format: &str) -> Result<Box<dyn Serializer>, String> {
    match format {
        "json" => Ok(Box::new(json::JsonSerializer)),
        "binary" | "bin" => Ok(Box::new(binary::BinarySerializer)),
        other => Err(format!("unknown output format '{other}' (expected 'json' or 'binary')")),
    }
}
