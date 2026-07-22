//! Output serialization behind a common interface.
//!
//! Both the JSON writer (complete) and the binary writer (a stub, until the
//! binary format is designed) implement the same `Serializer` trait, so the
//! build driver is agnostic to the output format and a new format is a matter of
//! adding one impl.

use crate::build::Output;

pub mod binary;
pub mod json;
pub mod quad;

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
    /// Whether this format needs the cost-model quadtree (subdivide/annotate) and
    /// packed geometry.  Formats that build their own tree return false so the
    /// driver can skip that work.
    fn uses_cost_tree(&self) -> bool {
        true
    }
}

/// Select a serializer by name.  `leaf_meters` configures the `quad` format's
/// leaf-size threshold; other formats ignore it.
pub fn for_format(format: &str, leaf_meters: f64) -> Result<Box<dyn Serializer>, String> {
    match format {
        "json" => Ok(Box::new(json::JsonSerializer)),
        "binary" | "bin" => Ok(Box::new(binary::BinarySerializer)),
        "quad" => Ok(Box::new(quad::QuadtreeSerializer { leaf_meters })),
        other => {
            Err(format!("unknown output format '{other}' (expected 'json', 'binary', or 'quad')"))
        }
    }
}
