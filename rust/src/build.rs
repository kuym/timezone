//! Build pipeline — port of tzconvert.js: classification, the lookup cost model,
//! greedy cost-driven subdivision, leaf annotation, geometry packing, and the
//! brute-force verifier.

use std::collections::HashMap;

use crate::geom::{self, Aabb, Pt};
use crate::lookup::{cell_center, quadrant_for_point, segment_crossings, split_cell};
use crate::polycodec;
use crate::quant::{MAX_DEPTH, ROOT_CELL};
use crate::topology::{self, ArcRef};

// Lookup cost model:
//   1 op per quadtree level descended;
//   1 op per arc vertex decoded to rebuild the candidate rings the localized test
//     needs (a reader decodes the whole of every arc that crosses the cell —
//     shared-arc geometry cannot be decoded partway);
//   2 ops per ring edge evaluated in the localized point-in-polygon test.
// Counting only the arcs that cross the cell (not a candidate's whole ring) keeps
// the cost subdivision-reducible: a smaller cell is crossed by fewer arcs, so
// `--max-ops` can actually drive it down.
pub const TRAVERSAL_OP: i64 = 1;
pub const RECON_OP: i64 = 1;
pub const VERTEX_OP: i64 = 2;

/// How a zone relates to a cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rel {
    Disjoint,
    Candidate,
    Definite,
}

/// A polygon record.  Carries build-time geometry (`outer`/`holes`) used by the
/// verifier and quadtree, plus the topological references (`outer_refs`/
/// `hole_refs`) into `Output::arcs` filled by `build_topology`.  The rings are
/// reconstructed from those arcs, so `outer`/`holes` always equal what a reader
/// rebuilds from the serialized arcs.
pub struct Zone {
    pub id: usize,
    pub tzid: usize,
    pub aabb: Aabb,
    pub a: i64,
    pub outer: Vec<Pt>,
    pub holes: Vec<Vec<Pt>>,
    pub hole_aabbs: Vec<Aabb>,
    pub outer_refs: Vec<ArcRef>,
    pub hole_refs: Vec<Vec<ArcRef>>,
}

/// A timezone (a named group of zones).
pub struct Tz {
    pub id: usize,
    pub n: String,
    pub refs: Vec<usize>,
}

/// The edges of ONE arc that cross a leaf cell: `a` indexes the ring's arc-ref
/// list (`zone.outer_refs`, or `zone.hole_refs[i]`), and `runs` are inclusive
/// `[first, last]` local edge ranges within that arc, in ring orientation.  A
/// reader decodes only the arcs named here, not the candidate's whole ring.
pub struct ArcRun {
    pub a: u32,
    pub runs: Vec<(u32, u32)>,
}

/// Per-leaf localization for one hole of a candidate.
pub struct HoleCand {
    pub i: usize,
    pub w: i64,
    pub e: Vec<ArcRun>,
}

/// A leaf candidate: which zone, the winding of the cell centre w.r.t. its full
/// outer ring, and the crossing arcs (plus crossing holes) in this cell.
pub struct Cand {
    pub z: usize,
    pub w: i64,
    pub e: Vec<ArcRun>,
    pub h: Vec<HoleCand>,
}

/// A quadtree node in the arena.  `refz` holds candidate zone ids until
/// `annotate` replaces them with `cands`.
pub struct Node {
    pub q: Option<[usize; 4]>,
    pub eref: Vec<usize>,
    pub refz: Vec<usize>,
    pub cands: Vec<Cand>,
}

impl Node {
    fn leaf() -> Node {
        Node { q: None, eref: Vec::new(), refz: Vec::new(), cands: Vec::new() }
    }
}

/// The whole built artifact.  `arcs` holds the shared polygon boundary arcs
/// (topology); zones reference them.  `arc_packed` is the origin+delta packing of
/// each arc, filled by `pack`.
pub struct Output {
    pub arena: Vec<Node>,
    pub root: usize,
    pub zones: Vec<Zone>,
    pub tz: Vec<Tz>,
    pub arcs: Vec<Vec<Pt>>,
    pub arc_packed: Vec<(Pt, Vec<u8>)>,
    tz_index: HashMap<String, usize>,
}

pub struct SubStats {
    pub splits: usize,
    pub at_max_depth: usize,
    pub over_limit: usize,
}

pub struct Stats {
    pub nodes: usize,
    pub leaves: usize,
    pub refs: usize,
    pub erefs: usize,
    pub max_depth: usize,
    pub max_leaf_refs: usize,
    pub max_leaf_cost: i64,
    pub max_leaf_true_cost: i64,
    pub leaves_over_limit: Option<usize>,
}

impl Output {
    pub fn new() -> Output {
        Output {
            arena: vec![Node::leaf()],
            root: 0,
            zones: Vec::new(),
            tz: Vec::new(),
            arcs: Vec::new(),
            arc_packed: Vec::new(),
            tz_index: HashMap::new(),
        }
    }

    /// Add one polygon (exterior ring + holes, already quantized and simplified)
    /// under timezone `name`, as a candidate of the root leaf.
    pub fn append_zone(&mut self, outer: Vec<Pt>, holes: Vec<Vec<Pt>>, name: &str) -> Option<usize> {
        if outer.len() < 3 {
            return None;
        }
        let tzid = match self.tz_index.get(name) {
            Some(&id) => id,
            None => {
                let id = self.tz.len();
                self.tz.push(Tz { id, n: name.to_string(), refs: Vec::new() });
                self.tz_index.insert(name.to_string(), id);
                id
            }
        };
        let id = self.zones.len();
        let aabb = geom::poly_aabb(&outer);
        let hole_aabbs: Vec<Aabb> = holes.iter().map(|h| geom::poly_aabb(h)).collect();
        let a = geom::ring_area2(&outer).abs();
        self.zones.push(Zone {
            id,
            tzid,
            aabb,
            a,
            outer,
            holes,
            hole_aabbs,
            outer_refs: Vec::new(),
            hole_refs: Vec::new(),
        });
        self.tz[tzid].refs.push(id);
        self.arena[self.root].refz.push(id);
        Some(id)
    }

    /// Factor every zone's rings into shared arcs (topology), optionally
    /// Visvalingam–Whyatt-simplifying each arc (`vw` < 1.0 = lossy), then rebuild
    /// each zone's rings from the arcs.  Must run before `subdivide`: it replaces
    /// `outer`/`holes` (and recomputes `aabb`/`a`) with the reconstructed geometry
    /// the quadtree and the readers both work from.  Returns (arc count, total arc
    /// vertices) for reporting.
    pub fn build_topology(&mut self, vw: f64) -> (usize, usize) {
        // Flatten all rings (each zone's outer, then its holes) in a fixed order.
        let mut rings: Vec<Vec<Pt>> = Vec::new();
        for z in &self.zones {
            rings.push(z.outer.clone());
            for h in &z.holes {
                rings.push(h.clone());
            }
        }

        let (arcs_full, ring_refs) = topology::build(&rings);
        let mut arcs: Vec<Vec<Pt>> = arcs_full.clone();
        // VW simplification is applied to the shared arcs (after topology), so a
        // shared border simplifies once and stays identical for both zones.
        topology::vw_simplify_all(&mut arcs, vw);

        // Map flat ring indices back to (zone, outer|hole).
        let mut flat = 0usize;
        let mut zone_outer: Vec<usize> = Vec::with_capacity(self.zones.len());
        let mut zone_holes: Vec<Vec<usize>> = Vec::with_capacity(self.zones.len());
        for z in &self.zones {
            zone_outer.push(flat);
            flat += 1;
            let mut hs = Vec::with_capacity(z.holes.len());
            for _ in &z.holes {
                hs.push(flat);
                flat += 1;
            }
            zone_holes.push(hs);
        }

        // Lossy simplification can collapse an outer ring below 3 vertices; if so,
        // restore (to full resolution) the arcs it uses and retry, so the stored
        // arcs always rebuild to a valid ring.  Monotonic → terminates.
        loop {
            let mut changed = false;
            for oi in &zone_outer {
                if topology::reconstruct(&arcs, &ring_refs[*oi]).len() < 3 {
                    for r in &ring_refs[*oi] {
                        if arcs[r.arc].len() < arcs_full[r.arc].len() {
                            arcs[r.arc] = arcs_full[r.arc].clone();
                            changed = true;
                        }
                    }
                }
            }
            if !changed {
                break;
            }
        }

        // Rebuild each zone from the (possibly restored) arcs.  Degenerate holes
        // (< 3 vertices after simplification) are dropped — a reader rebuilding
        // from the same arcs drops them identically.
        for (zi, z) in self.zones.iter_mut().enumerate() {
            let outer_refs = ring_refs[zone_outer[zi]].clone();
            z.outer = topology::reconstruct(&arcs, &outer_refs);
            z.outer_refs = outer_refs;

            let mut holes = Vec::new();
            let mut hole_aabbs = Vec::new();
            let mut hole_refs = Vec::new();
            for &hi in &zone_holes[zi] {
                let refs = ring_refs[hi].clone();
                let hole = topology::reconstruct(&arcs, &refs);
                if hole.len() >= 3 {
                    hole_aabbs.push(geom::poly_aabb(&hole));
                    holes.push(hole);
                    hole_refs.push(refs);
                }
            }
            z.holes = holes;
            z.hole_aabbs = hole_aabbs;
            z.hole_refs = hole_refs;
            z.aabb = geom::poly_aabb(&z.outer);
            z.a = geom::ring_area2(&z.outer).abs();
        }

        let arc_verts = arcs.iter().map(|a| a.len()).sum();
        self.arcs = arcs;
        (self.arcs.len(), arc_verts)
    }

    /// Subdivide the flat root into a quadtree driven by the cost model (see
    /// ANALYSIS.md §5c).  Greedy: repeatedly split the most expensive leaf until
    /// every leaf is within `op_limit`, or the split budget (`max_splits`, which
    /// takes precedence) runs out.
    pub fn subdivide(&mut self, op_limit: i64, max_splits: Option<usize>) -> SubStats {
        let mut heap = JsHeap::new();
        let root_cost = leaf_cost(&self.arena[self.root].refz, &self.zones, &self.arcs, &ROOT_CELL, 0);
        heap.push(Entry { cost: root_cost, idx: self.root, cell: ROOT_CELL, depth: 0 });

        let (mut splits, mut at_max_depth, mut over_limit) = (0usize, 0usize, 0usize);

        while let Some(top) = heap.peek() {
            if top.cost <= op_limit {
                break; // worst leaf is within budget
            }
            if let Some(ms) = max_splits {
                if splits >= ms {
                    break; // budget exhausted (takes precedence)
                }
            }
            let leaf = heap.pop().unwrap();
            if leaf.depth >= MAX_DEPTH {
                at_max_depth += 1;
                continue;
            }
            // Edgeless leaf: splitting only adds depth, never helps.
            if self.arena[leaf.idx].refz.is_empty() {
                over_limit += 1;
                continue;
            }

            let refz = std::mem::take(&mut self.arena[leaf.idx].refz);
            let base = self.arena.len();
            let mut entries: Vec<Entry> = Vec::with_capacity(4);
            for i in 0..4 {
                let cc = split_cell(&leaf.cell, i);
                let mut child = Node::leaf();
                for &zi in &refz {
                    match classify(&cc, &self.zones[zi]) {
                        Rel::Definite => child.eref.push(zi),
                        Rel::Candidate => child.refz.push(zi),
                        Rel::Disjoint => {}
                    }
                }
                let cost = leaf_cost(&child.refz, &self.zones, &self.arcs, &cc, leaf.depth + 1);
                self.arena.push(child);
                entries.push(Entry { cost, idx: base + i, cell: cc, depth: leaf.depth + 1 });
            }
            self.arena[leaf.idx].q = Some([base, base + 1, base + 2, base + 3]);
            splits += 1;
            for e in entries {
                heap.push(e);
            }
        }

        // Leaves still over budget when we stopped (split budget ran out).
        while let Some(e) = heap.pop() {
            if e.cost > op_limit {
                over_limit += 1;
            }
        }

        SubStats { splits, at_max_depth, over_limit }
    }

    /// Attach per-leaf edge subsets so lookups test only the ring edges that
    /// pass through each leaf cell.
    pub fn annotate(&mut self) {
        let mut stack: Vec<(usize, Aabb)> = vec![(self.root, ROOT_CELL)];
        while let Some((idx, cell)) = stack.pop() {
            if let Some(children) = self.arena[idx].q {
                for i in 0..4 {
                    stack.push((children[i], split_cell(&cell, i)));
                }
                continue;
            }
            let refz = std::mem::take(&mut self.arena[idx].refz);
            let center = cell_center(&cell);
            let mut cands = Vec::with_capacity(refz.len());
            for zi in refz {
                let zone = &self.zones[zi];
                // Winding stays w.r.t. the FULL ring; only the crossing arcs are
                // localized, so the reader rebuilds just those.
                let e = arc_runs(&zone.outer_refs, &self.arcs, &cell);
                let w = geom::poly_winding(&zone.outer, center);
                let mut h = Vec::new();
                for (hi, hole) in zone.holes.iter().enumerate() {
                    if !geom::aabbs_overlap(&cell, &zone.hole_aabbs[hi]) {
                        continue;
                    }
                    let he = arc_runs(&zone.hole_refs[hi], &self.arcs, &cell);
                    if he.is_empty() {
                        continue;
                    }
                    h.push(HoleCand { i: hi, w: geom::poly_winding(hole, center), e: he });
                }
                cands.push(Cand { z: zi, w, e, h });
            }
            self.arena[idx].cands = cands;
        }
    }

    /// Pack every shared arc into origin + delta stream (the serializers add
    /// base64 or store raw as needed).
    pub fn pack(&mut self) {
        self.arc_packed = self.arcs.iter().map(|arc| polycodec::encode_polygon(arc)).collect();
    }

    /// Tree + cost statistics.  Cost fields require `refz` to still be populated,
    /// so call this before `annotate`.
    pub fn tree_stats(&self, op_limit: Option<i64>) -> Stats {
        let (mut nodes, mut leaves, mut refs, mut erefs) = (0usize, 0usize, 0usize, 0usize);
        let (mut max_depth, mut max_leaf_refs, mut max_cost, mut over) = (0usize, 0usize, 0i64, 0usize);
        let mut max_true_cost = 0i64;
        let mut stack: Vec<(usize, Aabb, usize)> = vec![(self.root, ROOT_CELL, 0)];
        while let Some((idx, cell, d)) = stack.pop() {
            nodes += 1;
            max_depth = max_depth.max(d);
            let node = &self.arena[idx];
            refs += node.refz.len();
            erefs += node.eref.len();
            match node.q {
                Some(children) => {
                    for i in 0..4 {
                        stack.push((children[i], split_cell(&cell, i), d + 1));
                    }
                }
                None => {
                    leaves += 1;
                    max_leaf_refs = max_leaf_refs.max(node.refz.len());
                    let cost = leaf_cost(&node.refz, &self.zones, &self.arcs, &cell, d);
                    max_cost = max_cost.max(cost);
                    max_true_cost = max_true_cost.max(leaf_true_cost(&node.refz, &self.zones, &self.arcs, &cell, d));
                    if let Some(limit) = op_limit {
                        if cost > limit {
                            over += 1;
                        }
                    }
                }
            }
        }
        Stats {
            nodes,
            leaves,
            refs,
            erefs,
            max_depth,
            max_leaf_refs,
            max_leaf_cost: max_cost,
            max_leaf_true_cost: max_true_cost,
            leaves_over_limit: op_limit.map(|_| over),
        }
    }

    // --- reader side, used by the verifier ---

    fn resolve(&self, point: Pt) -> Option<usize> {
        let mut node = self.root;
        let mut cell = ROOT_CELL;
        let mut definite: Vec<usize> = Vec::new();
        loop {
            definite.extend_from_slice(&self.arena[node].eref);
            match self.arena[node].q {
                Some(children) => {
                    let qi = quadrant_for_point(&cell, point);
                    cell = split_cell(&cell, qi);
                    node = children[qi];
                }
                None => break,
            }
        }

        let mut best: Option<usize> = None;
        for &zi in &definite {
            if best.map_or(true, |b| self.zones[zi].a < self.zones[b].a) {
                best = Some(zi);
            }
        }
        let center = cell_center(&cell);
        for cand in &self.arena[node].cands {
            if self.local_contains(cand, center, point) {
                let zid = cand.z;
                if best.map_or(true, |b| self.zones[zid].a < self.zones[b].a) {
                    best = Some(zid);
                }
            }
        }
        best
    }

    fn local_contains(&self, cand: &Cand, center: Pt, point: Pt) -> bool {
        let zone = &self.zones[cand.z];
        // Sum ray crossings over just the crossing arcs of the outer ring.
        match arc_crossings(&cand.e, &zone.outer_refs, &self.arcs, center, point) {
            None => return geom::rings_contain_point(&zone.outer, &zone.holes, point),
            Some(d) => {
                if cand.w + d == 0 {
                    return false;
                }
            }
        }
        for hc in &cand.h {
            match arc_crossings(&hc.e, &zone.hole_refs[hc.i], &self.arcs, center, point) {
                None => return geom::rings_contain_point(&zone.outer, &zone.holes, point),
                Some(d) => {
                    if hc.w + d != 0 {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Brute-force smallest-area zone containing `point` (or `None`).
    fn brute(&self, point: Pt) -> Option<usize> {
        let mut best: Option<usize> = None;
        for z in &self.zones {
            if geom::rings_contain_point(&z.outer, &z.holes, point) {
                if best.map_or(true, |b| z.a < self.zones[b].a) {
                    best = Some(z.id);
                }
            }
        }
        best
    }

    /// Cross-check the tree against brute force over `samples` random points.
    /// Returns the number of disagreements.  Uses its own deterministic PRNG
    /// (the sequence need not match the JS verifier — it independently proves the
    /// Rust build correct).
    pub fn verify(&self, samples: usize) -> usize {
        let mut failures = 0usize;
        let mut rng = Lcg::new(0x2545F4914F6CDD1D);
        let (xspan, yspan) = ((ROOT_CELL[1][0] - ROOT_CELL[0][0]) as f64, (ROOT_CELL[1][1] - ROOT_CELL[0][1]) as f64);
        for _ in 0..samples {
            let x = ROOT_CELL[0][0] + (rng.f64() * xspan).floor() as i64;
            let y = ROOT_CELL[0][1] + (rng.f64() * yspan).floor() as i64;
            let point = [x, y];
            let expected = self.brute(point);
            let actual = self.resolve(point);
            let ok = match (expected, actual) {
                (None, None) => true,
                (Some(e), Some(a)) => {
                    self.zones[a].a == self.zones[e].a && self.zones[a].tzid == self.zones[e].tzid
                }
                _ => false,
            };
            if !ok {
                failures += 1;
            }
        }
        failures
    }
}

// Classify a zone against a cell, accounting for holes.
pub fn classify(cell: &Aabb, zone: &Zone) -> Rel {
    if !geom::aabbs_overlap(cell, &zone.aabb) {
        return Rel::Disjoint;
    }
    match geom::aabb_intersects_poly(cell, &zone.outer) {
        geom::AabbPoly::Disjoint => Rel::Disjoint,
        geom::AabbPoly::Crosses => Rel::Candidate,
        geom::AabbPoly::Contains => {
            let mut rel = Rel::Definite;
            for (i, hole) in zone.holes.iter().enumerate() {
                if !geom::aabbs_overlap(cell, &zone.hole_aabbs[i]) {
                    continue;
                }
                match geom::aabb_intersects_poly(cell, hole) {
                    geom::AabbPoly::Contains => return Rel::Disjoint, // cell inside a hole
                    geom::AabbPoly::Crosses => rel = Rel::Candidate,
                    geom::AabbPoly::Disjoint => {}
                }
            }
            rel
        }
    }
}

// Edges of one arc (an open polyline, not a closed ring) that cross the cell.
fn arc_edges_in_cell(arc: &[Pt], cell: &Aabb) -> usize {
    (0..arc.len().saturating_sub(1)).filter(|&i| geom::edge_touches_aabb(cell, arc[i], arc[i + 1])).count()
}

/// Cost of testing one candidate zone in one cell, as (arc vertices a reader
/// decodes, ring edges it then evaluates).  With arc-localized candidates a
/// reader rebuilds only the arcs that cross the cell — so this counts the full
/// vertex length of each crossing arc, and the localized edges within them.
pub fn candidate_cost(zone: &Zone, cell: &Aabb, arcs: &[Vec<Pt>]) -> (i64, i64) {
    let (mut recon, mut edges) = (0i64, 0i64);
    let mut acc = |r: &ArcRef| {
        let c = arc_edges_in_cell(&arcs[r.arc], cell);
        if c > 0 {
            recon += arcs[r.arc].len() as i64;
            edges += c as i64;
        }
    };
    for r in &zone.outer_refs {
        acc(r);
    }
    for (hi, hrefs) in zone.hole_refs.iter().enumerate() {
        if !geom::aabbs_overlap(cell, &zone.hole_aabbs[hi]) {
            continue;
        }
        for r in hrefs {
            acc(r);
        }
    }
    (recon, edges)
}

/// The **subdivision-reducible** lookup cost of a leaf: traversal + the localized
/// point-in-polygon edge tests.  This is what `--max-ops` governs, because it is
/// what splitting a cell can actually lower.  Arc reconstruction is excluded —
/// even localized to crossing arcs it has an irreducible floor (a single long arc
/// crossing a cell must be decoded in full no matter how small the cell), so
/// counting it here would make the budget unsatisfiable and split uselessly to
/// MAX_DEPTH.  See `leaf_true_cost` for the full cost a reader actually pays.
pub fn leaf_cost(refz: &[usize], zones: &[Zone], arcs: &[Vec<Pt>], cell: &Aabb, depth: usize) -> i64 {
    let mut edges = 0i64;
    for &zi in refz {
        edges += candidate_cost(&zones[zi], cell, arcs).1;
    }
    depth as i64 * TRAVERSAL_OP + edges * VERTEX_OP
}

/// The full worst-case lookup cost a reader pays for this leaf, INCLUDING arc
/// reconstruction — the same accounting `tzlookup_binary.js` reports.  Now that
/// readers rebuild only the crossing arcs, this counts just those arcs' vertices
/// (not the whole ring), so it is far lower than before.  Reported, not used to
/// drive subdivision (see `leaf_cost`); `--vw` is the lever that lowers it.
pub fn leaf_true_cost(refz: &[usize], zones: &[Zone], arcs: &[Vec<Pt>], cell: &Aabb, depth: usize) -> i64 {
    let mut cost = depth as i64 * TRAVERSAL_OP;
    for &zi in refz {
        let (recon, edges) = candidate_cost(&zones[zi], cell, arcs);
        cost += recon * RECON_OP + edges * VERTEX_OP;
    }
    cost
}

// For a ring (its arc-ref list), the arcs that cross `cell`, each with the local
// edge runs (in ring orientation) that cross.  Only these arcs need decoding.
fn arc_runs(refs: &[ArcRef], arcs: &[Vec<Pt>], cell: &Aabb) -> Vec<ArcRun> {
    let mut out = Vec::new();
    for (ai, r) in refs.iter().enumerate() {
        let runs = arc_edge_runs(&arcs[r.arc], r.rev, cell);
        if !runs.is_empty() {
            out.push(ArcRun { a: ai as u32, runs });
        }
    }
    out
}

// Edge indices of one arc (open polyline, oriented as it appears in the ring —
// reversed when `rev`) that cross `cell`, as inclusive [first, last] runs.
fn arc_edge_runs(arc: &[Pt], rev: bool, cell: &Aabb) -> Vec<(u32, u32)> {
    let ring_arc: Vec<Pt> = if rev { arc.iter().rev().cloned().collect() } else { arc.to_vec() };
    let mut runs: Vec<(u32, u32)> = Vec::new();
    for i in 0..ring_arc.len().saturating_sub(1) {
        if geom::edge_touches_aabb(cell, ring_arc[i], ring_arc[i + 1]) {
            let iu = i as u32;
            if let Some(last) = runs.last_mut() {
                if last.1 + 1 == iu {
                    last.1 = iu;
                    continue;
                }
            }
            runs.push((iu, iu));
        }
    }
    runs
}

// One arc's vertices as they appear in the ring (reversed when `rev`).
fn ring_arc(arcs: &[Vec<Pt>], r: &ArcRef) -> Vec<Pt> {
    if r.rev {
        arcs[r.arc].iter().rev().cloned().collect()
    } else {
        arcs[r.arc].clone()
    }
}

// Sum of signed center->point ray crossings over a ring's localized arc runs
// (decoding only those arcs), or None if any arc's test is degenerate.
fn arc_crossings(
    runs: &[ArcRun],
    refs: &[ArcRef],
    arcs: &[Vec<Pt>],
    center: Pt,
    point: Pt,
) -> Option<i64> {
    let mut total = 0i64;
    for ar in runs {
        let arc = ring_arc(arcs, &refs[ar.a as usize]);
        total += segment_crossings(center, point, &arc, &ar.runs)?;
    }
    Some(total)
}

struct Entry {
    cost: i64,
    idx: usize,
    cell: Aabb,
    depth: usize,
}

// A binary max-heap keyed by cost, replicating tzconvert.js's MaxHeap array
// operations exactly (sift-up keeps the parent on ties; sift-down prefers the
// strictly larger child).  Matching it byte-for-byte means the Rust and JS
// builds pop equal-cost leaves in the same order, so even a limited --max-splits
// budget produces the identical tree.
struct JsHeap {
    a: Vec<Entry>,
}
impl JsHeap {
    fn new() -> JsHeap {
        JsHeap { a: Vec::new() }
    }
    fn peek(&self) -> Option<&Entry> {
        self.a.first()
    }
    fn push(&mut self, e: Entry) {
        self.a.push(e);
        let mut i = self.a.len() - 1;
        while i > 0 {
            let p = (i - 1) >> 1;
            if self.a[p].cost >= self.a[i].cost {
                break;
            }
            self.a.swap(p, i);
            i = p;
        }
    }
    fn pop(&mut self) -> Option<Entry> {
        if self.a.is_empty() {
            return None;
        }
        let last = self.a.pop().unwrap();
        if self.a.is_empty() {
            return Some(last); // heap had a single element (the top)
        }
        let top = std::mem::replace(&mut self.a[0], last);
        let mut i = 0usize;
        loop {
            let (l, r) = (2 * i + 1, 2 * i + 2);
            let mut m = i;
            if l < self.a.len() && self.a[l].cost > self.a[m].cost {
                m = l;
            }
            if r < self.a.len() && self.a[r].cost > self.a[m].cost {
                m = r;
            }
            if m == i {
                break;
            }
            self.a.swap(m, i);
            i = m;
        }
        Some(top)
    }
}

// Small deterministic PRNG for the verifier's sample points.
struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Lcg {
        Lcg(seed)
    }
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }
    fn f64(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}
