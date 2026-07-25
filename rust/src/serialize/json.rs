//! JSON serializer — produces the `quadtree.json` format documented in
//! quadtree.md, byte-for-byte compatible with the JS build's schema (compact,
//! no whitespace; object keys in the same order the JS writer emits).

use super::Serializer;
use crate::build::{ArcRun, Cand, Node, Output, Zone};
use crate::polycodec;
use crate::quant;
use crate::topology::ArcRef;

pub struct JsonSerializer;

impl Serializer for JsonSerializer {
    fn serialize(&self, out: &Output) -> Result<Vec<u8>, String> {
        let mut s = String::with_capacity(out.zones.len() * 256);
        s.push('{');

        // quant
        s.push_str("\"quant\":{");
        s.push_str(&format!("\"xMin\":{},\"xMax\":{},", quant::X_MIN, quant::X_MAX));
        s.push_str(&format!("\"yMin\":{},\"yMax\":{},", quant::Y_MIN, quant::Y_MAX));
        s.push_str(&format!("\"xScale\":{},\"yScale\":{},", fnum(quant::X_SCALE), fnum(quant::Y_SCALE)));
        s.push_str(&format!("\"maxDepth\":{}}},", quant::MAX_DEPTH));

        // rootCell
        s.push_str(&format!(
            "\"rootCell\":[[{},{}],[{},{}]],",
            quant::X_MIN, quant::Y_MIN, quant::X_MAX, quant::Y_MAX
        ));

        // quadtree
        s.push_str("\"quadtree\":");
        write_node(&mut s, out, out.root);
        s.push(',');

        // arcs (shared polygon boundaries) — origin + base64 delta stream each
        s.push_str("\"arcs\":[");
        for (i, (o, p)) in out.arc_packed.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&format!("{{\"o\":[{},{}],\"p\":", o[0], o[1]));
            s.push_str(&json_str(&polycodec::base64_encode(p)));
            s.push('}');
        }
        s.push_str("],");

        // zones
        s.push_str("\"zones\":[");
        for (i, z) in out.zones.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            write_zone(&mut s, z);
        }
        s.push_str("],");

        // tz (keyed by name, in id order)
        s.push_str("\"tz\":{");
        for (i, tz) in out.tz.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&json_str(&tz.n));
            s.push_str(&format!(":{{\"id\":{},\"n\":", tz.id));
            s.push_str(&json_str(&tz.n));
            s.push_str(",\"ref\":[");
            for (j, r) in tz.refs.iter().enumerate() {
                if j > 0 {
                    s.push(',');
                }
                s.push_str(&r.to_string());
            }
            s.push_str("]}");
        }
        s.push('}');

        s.push('}');
        Ok(s.into_bytes())
    }

    fn format_name(&self) -> &'static str {
        "json"
    }
}

// Node key order matches JS newNode(): q, eref, ref.  Empty members are omitted.
fn write_node(s: &mut String, out: &Output, idx: usize) {
    let node: &Node = &out.arena[idx];
    s.push('{');
    let mut first = true;

    if let Some(children) = node.q {
        s.push_str("\"q\":[");
        for (i, &c) in children.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            write_node(s, out, c);
        }
        s.push(']');
        first = false;
    }
    if !node.eref.is_empty() {
        if !first {
            s.push(',');
        }
        s.push_str("\"eref\":[");
        for (i, r) in node.eref.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&r.to_string());
        }
        s.push(']');
        first = false;
    }
    if !node.cands.is_empty() {
        if !first {
            s.push(',');
        }
        s.push_str("\"ref\":[");
        for (i, c) in node.cands.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            write_cand(s, c);
        }
        s.push(']');
    }
    s.push('}');
}

// Candidate key order: z, w, e, h.  `e` is a list of crossing arcs, each
// `[arcIndex, [[first,last],...]]` where arcIndex indexes the ring's arc-ref list.
fn write_cand(s: &mut String, c: &Cand) {
    s.push_str(&format!("{{\"z\":{},\"w\":{},\"e\":", c.z, c.w));
    write_arc_runs(s, &c.e);
    if !c.h.is_empty() {
        s.push_str(",\"h\":[");
        for (i, hc) in c.h.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&format!("{{\"i\":{},\"w\":{},\"e\":", hc.i, hc.w));
            write_arc_runs(s, &hc.e);
            s.push('}');
        }
        s.push(']');
    }
    s.push('}');
}

fn write_arc_runs(s: &mut String, e: &[ArcRun]) {
    s.push('[');
    for (i, ar) in e.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&format!("[{},", ar.a));
        write_runs(s, &ar.runs);
        s.push(']');
    }
    s.push(']');
}

fn write_runs(s: &mut String, runs: &[(u32, u32)]) {
    s.push('[');
    for (i, r) in runs.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&format!("[{},{}]", r.0, r.1));
    }
    s.push(']');
}

// Zone key order: id, aabb, a, tzid, outer (arc refs), h (hole arc refs).
// An arc reference is a signed integer: i for arc i forward, -i-1 reversed.
fn write_zone(s: &mut String, z: &Zone) {
    s.push_str(&format!("{{\"id\":{},\"aabb\":[[{},{}],[{},{}]],\"a\":{},\"tzid\":{},",
        z.id, z.aabb[0][0], z.aabb[0][1], z.aabb[1][0], z.aabb[1][1], z.a, z.tzid));
    s.push_str("\"outer\":");
    write_arc_refs(s, &z.outer_refs);
    if !z.hole_refs.is_empty() {
        s.push_str(",\"h\":[");
        for (i, refs) in z.hole_refs.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            write_arc_refs(s, refs);
        }
        s.push(']');
    }
    s.push('}');
}

fn write_arc_refs(s: &mut String, refs: &[ArcRef]) {
    s.push('[');
    for (i, r) in refs.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let v: i64 = if r.rev { -(r.arc as i64) - 1 } else { r.arc as i64 };
        s.push_str(&v.to_string());
    }
    s.push(']');
}

// Shortest round-trippable f64 text (matches JS Number->string for these values).
fn fnum(x: f64) -> String {
    let s = format!("{x}");
    if s.contains('.') || s.contains('e') || s.contains('E') {
        s
    } else {
        format!("{s}.0")
    }
}

// Minimal JSON string escaping.  Timezone names and base64 contain no control
// characters, but escape the JSON-significant ones for safety.
fn json_str(v: &str) -> String {
    let mut out = String::with_capacity(v.len() + 2);
    out.push('"');
    for c in v.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}
