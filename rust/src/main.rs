//! Timezone quadtree compressor — Rust port of tzconvert.js.
//!
//! Reads a GeoJSON FeatureCollection of timezone polygons, quantizes and
//! simplifies them, builds a cost-model-driven quadtree, verifies it against
//! brute force, and serializes it (json, binary, or quad).

mod build;
mod geom;
mod lookup;
mod polycodec;
mod quant;
mod serialize;

use serde::Deserialize;
use std::fs::File;
use std::io::{BufReader, Write};
use std::process::ExitCode;

// --- GeoJSON input model (parsed with serde_json) ---

#[derive(Deserialize)]
struct FeatureCollection {
    features: Vec<Feature>,
}

#[derive(Deserialize)]
struct Feature {
    properties: Props,
    geometry: Geometry,
}

#[derive(Deserialize)]
struct Props {
    tzid: String,
}

#[derive(Deserialize)]
struct Geometry {
    // Untagged coordinates disambiguate Polygon vs MultiPolygon by nesting depth;
    // the `type` field is available too but not needed.
    coordinates: Coords,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Coords {
    Polygon(Vec<Vec<[f64; 2]>>),
    MultiPolygon(Vec<Vec<Vec<[f64; 2]>>>),
}

struct Options {
    input: String,
    output: String,
    format: String,
    epsilon: f64,
    max_ops: i64,
    max_splits: Option<usize>,
    verify: usize,
    leaf_km: f64,
}

const HELP: &str = "\
tzconvert — timezone quadtree compressor (Rust port of tzconvert.js)

USAGE:
    tzconvert [OPTIONS] [OUTPUT]

    OUTPUT   Output file path (positional).            [default: quadtree.json]

OPTIONS:
    --input=PATH       Input GeoJSON FeatureCollection. [default: data/combined.json]
    --format=FMT       Output format: json | binary | quad.          [default: json]
    --epsilon=N        RDP simplification tolerance, in quantized units (~38 m
                       each).                                            [default: 8]
    --max-ops=N        Per-leaf lookup cost budget (json/binary): a leaf over this
                       many ops is split.                              [default: 500]
    --max-splits=N     Hard cap on the number of leaf splits; TAKES PRECEDENCE over
                       --max-ops, trading artifact size for lookup speed.
                                                                 [default: unlimited]
    --leaf-km=N        (quad format) stop splitting a cell once its longest edge is
                       under N km; the majority-area tzid wins.        [default: 10]
    --verify=N         Cross-check N random points against brute force (0 disables).
                                                                     [default: 3000]
    -h, --help         Print this help and exit.

FORMATS:
    json     Full quadtree + packed polygons; the quadtree.json schema (see
             quadtree.md). Byte-for-byte identical to the JavaScript build.
    binary   Compact binary of the full artifact (quadtree + packed polygons,
             no base64); ~42% smaller than json. See rust/README.md.
    quad     Experimental. A tiny quadtree with NO polygons: every leaf resolves
             to one tzid (majority area at the --leaf-km limit). See rust/README.md.

EXAMPLES:
    tzconvert --input=data/combined.json quadtree.json
    tzconvert --max-ops=250 --max-splits=2000 quadtree.json
    tzconvert --format=quad --leaf-km=10 tz.quad

Run from the project root so the default --input path resolves.";

fn parse_args() -> Result<Options, String> {
    let mut opt = Options {
        input: "data/combined.json".to_string(),
        output: "quadtree.json".to_string(),
        format: "json".to_string(),
        epsilon: 8.0,
        max_ops: 500,
        max_splits: None,
        verify: 3000,
        leaf_km: 10.0,
    };
    for arg in std::env::args().skip(1) {
        if arg == "--help" || arg == "-h" {
            println!("{HELP}");
            std::process::exit(0);
        } else if let Some(v) = arg.strip_prefix("--input=") {
            opt.input = v.to_string();
        } else if let Some(v) = arg.strip_prefix("--format=") {
            opt.format = v.to_string();
        } else if let Some(v) = arg.strip_prefix("--epsilon=") {
            opt.epsilon = v.parse().map_err(|_| format!("bad --epsilon: {v}"))?;
        } else if let Some(v) = arg.strip_prefix("--max-ops=") {
            opt.max_ops = v.parse().map_err(|_| format!("bad --max-ops: {v}"))?;
        } else if let Some(v) = arg.strip_prefix("--max-splits=") {
            opt.max_splits = Some(v.parse().map_err(|_| format!("bad --max-splits: {v}"))?);
        } else if let Some(v) = arg.strip_prefix("--verify=") {
            opt.verify = v.parse().map_err(|_| format!("bad --verify: {v}"))?;
        } else if let Some(v) = arg.strip_prefix("--leaf-km=") {
            opt.leaf_km = v.parse().map_err(|_| format!("bad --leaf-km: {v}"))?;
        } else if arg.starts_with("--") {
            return Err(format!("unknown option: {arg}"));
        } else {
            opt.output = arg;
        }
    }
    Ok(opt)
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let opt = parse_args()?;
    eprintln!(
        "options: max-ops={}, max-splits={}, epsilon={}, verify={}, format={}, out={}",
        opt.max_ops,
        opt.max_splits.map(|n| n.to_string()).unwrap_or_else(|| "unlimited".into()),
        opt.epsilon,
        opt.verify,
        opt.format,
        opt.output
    );

    let serializer = serialize::for_format(&opt.format, opt.leaf_km * 1000.0)?;

    eprintln!("loading {} ...", opt.input);
    let file = File::open(&opt.input).map_err(|e| format!("opening {}: {e}", opt.input))?;
    let fc: FeatureCollection = serde_json::from_reader(BufReader::new(file))
        .map_err(|e| format!("parsing {}: {e}", opt.input))?;
    eprintln!("processing {} features, epsilon={}", fc.features.len(), opt.epsilon);

    let mut out = build::Output::new();
    let (mut rings_in, mut holes_in, mut skipped, mut wrapped) = (0usize, 0usize, 0usize, 0usize);

    let mut insert_polygon = |out: &mut build::Output, rings: &[Vec<[f64; 2]>], name: &str| {
        let outer = quant::simplify_rdp(&quant::quantize(&rings[0]), opt.epsilon);
        let mut holes = Vec::new();
        for r in &rings[1..] {
            let hole = quant::simplify_rdp(&quant::quantize(r), opt.epsilon);
            if hole.len() >= 3 {
                holes.push(hole);
                holes_in += 1;
            }
        }
        rings_in += 1;

        // Antimeridian check: a ring spanning more than half the world in
        // longitude is not handled (no source ring should, save Antarctica).
        let aabb = geom::poly_aabb(&outer);
        if (aabb[1][0] - aabb[0][0]) > (quant::X_MAX - quant::X_MIN) / 2 {
            wrapped += 1;
            eprintln!(
                "WARNING: ring for {name} spans more than half the world in longitude \
                 (x {}..{}); antimeridian wrapping is not handled.",
                aabb[0][0], aabb[1][0]
            );
        }

        if out.append_zone(outer, holes, name).is_none() {
            skipped += 1;
        }
    };

    for (n, feature) in fc.features.iter().enumerate() {
        let name = &feature.properties.tzid;
        match &feature.geometry.coordinates {
            Coords::Polygon(rings) => insert_polygon(&mut out, rings, name),
            Coords::MultiPolygon(polys) => {
                for rings in polys {
                    insert_polygon(&mut out, rings, name);
                }
            }
        }
        if n % 25 == 0 {
            eprintln!("  {}/{} {}", n, fc.features.len(), name);
        }
    }
    eprintln!(
        "exterior rings: {rings_in}, holes: {holes_in}, skipped: {skipped}, antimeridian-wrapped: {wrapped}"
    );

    // The cost-model quadtree + packed geometry are only needed by formats that
    // serialize them (json, binary).  Formats that build their own tree (quad)
    // skip this whole phase.
    if serializer.uses_cost_tree() {
        eprintln!(
            "subdividing (max-ops={}, max-splits={}) ...",
            opt.max_ops,
            opt.max_splits.map(|n| n.to_string()).unwrap_or_else(|| "unlimited".into())
        );
        let sub = out.subdivide(opt.max_ops, opt.max_splits);
        eprintln!(
            "subdivision: {{\"splits\":{},\"atMaxDepth\":{},\"overLimit\":{}}}",
            sub.splits, sub.at_max_depth, sub.over_limit
        );

        // Cost stats before annotate (which consumes the candidate zone lists).
        let st = out.tree_stats(Some(opt.max_ops));
        eprintln!(
            "quadtree: {{\"nodes\":{},\"leaves\":{},\"refs\":{},\"erefs\":{},\"maxDepth\":{},\
             \"maxLeafRefs\":{},\"maxLeafCost\":{},\"leavesOverLimit\":{}}}",
            st.nodes, st.leaves, st.refs, st.erefs, st.max_depth, st.max_leaf_refs, st.max_leaf_cost,
            st.leaves_over_limit.unwrap_or(0)
        );

        out.annotate();

        if opt.verify > 0 {
            eprintln!("verifying {} random points against brute force ...", opt.verify);
            let failures = out.verify(opt.verify);
            if failures > 0 {
                return Err(format!("{failures}/{} lookups disagreed with brute force", opt.verify));
            }
            eprintln!("  all {} lookups agree", opt.verify);
        }

        out.pack();
    }

    if !serializer.is_complete() {
        eprintln!(
            "WARNING: the '{}' serializer is INCOMPLETE — some sections (polygon geometry) are \
             not yet implemented, so the file is not a usable standalone artifact.",
            serializer.format_name()
        );
    }
    let bytes = serializer.serialize(&out)?;
    let mut f = File::create(&opt.output).map_err(|e| format!("creating {}: {e}", opt.output))?;
    f.write_all(&bytes).map_err(|e| format!("writing {}: {e}", opt.output))?;
    eprintln!("wrote {} ({} bytes, {})", opt.output, bytes.len(), serializer.format_name());
    Ok(())
}
