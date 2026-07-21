//! Build pipeline — port of tzconvert.js: classification, the lookup cost model,
//! greedy cost-driven subdivision, leaf annotation, geometry packing, and the
//! brute-force verifier.

use std::collections::HashMap;

use crate::geom::{self, Aabb, Pt};
use crate::lookup::{cell_center, quadrant_for_point, segment_crossings, split_cell};
use crate::polycodec;
use crate::quant::{MAX_DEPTH, ROOT_CELL};

// Lookup cost model: 1 op per quadtree level descended, 2 ops per ring edge
// evaluated in the point-in-polygon test.
pub const TRAVERSAL_OP: i64 = 1;
pub const VERTEX_OP: i64 = 2;

/// How a zone relates to a cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rel {
    Disjoint,
    Candidate,
    Definite,
}

/// A polygon record.  Carries build-time geometry (`outer`/`holes`) used by the
/// verifier and packer, plus the packed fields (`o`/`p`/`h`) filled by `pack`.
pub struct Zone {
    pub id: usize,
    pub tzid: usize,
    pub aabb: Aabb,
    pub a: i64,
    pub outer: Vec<Pt>,
    pub holes: Vec<Vec<Pt>>,
    pub hole_aabbs: Vec<Aabb>,
    pub o: Pt,
    pub p: String,
    pub h: Vec<(Pt, String)>,
}

/// A timezone (a named group of zones).
pub struct Tz {
    pub id: usize,
    pub n: String,
    pub refs: Vec<usize>,
}

/// Per-leaf localization for one hole of a candidate.
pub struct HoleCand {
    pub i: usize,
    pub w: i64,
    pub e: Vec<(u32, u32)>,
}

/// A leaf candidate: which zone, the winding of the cell centre w.r.t. its full
/// outer ring, and the outer edge runs (plus crossing holes) in this cell.
pub struct Cand {
    pub z: usize,
    pub w: i64,
    pub e: Vec<(u32, u32)>,
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

/// The whole built artifact.
pub struct Output {
    pub arena: Vec<Node>,
    pub root: usize,
    pub zones: Vec<Zone>,
    pub tz: Vec<Tz>,
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
    pub leaves_over_limit: Option<usize>,
}

impl Output {
    pub fn new() -> Output {
        Output {
            arena: vec![Node::leaf()],
            root: 0,
            zones: Vec::new(),
            tz: Vec::new(),
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
            o: [0, 0],
            p: String::new(),
            h: Vec::new(),
        });
        self.tz[tzid].refs.push(id);
        self.arena[self.root].refz.push(id);
        Some(id)
    }

    /// Subdivide the flat root into a quadtree driven by the cost model (see
    /// ANALYSIS.md §5c).  Greedy: repeatedly split the most expensive leaf until
    /// every leaf is within `op_limit`, or the split budget (`max_splits`, which
    /// takes precedence) runs out.
    pub fn subdivide(&mut self, op_limit: i64, max_splits: Option<usize>) -> SubStats {
        let mut heap = JsHeap::new();
        let root_cost = leaf_cost(&self.arena[self.root].refz, &self.zones, &ROOT_CELL, 0);
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
                let cost = leaf_cost(&child.refz, &self.zones, &cc, leaf.depth + 1);
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
                let e = edge_runs(&zone.outer, &cell);
                let w = geom::poly_winding(&zone.outer, center);
                let mut h = Vec::new();
                for (hi, hole) in zone.holes.iter().enumerate() {
                    if !geom::aabbs_overlap(&cell, &zone.hole_aabbs[hi]) {
                        continue;
                    }
                    let he = edge_runs(hole, &cell);
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

    /// Pack every zone's geometry into origin + base64 delta stream.
    pub fn pack(&mut self) {
        for zone in &mut self.zones {
            let (o, p) = polycodec::encode_polygon(&zone.outer);
            zone.o = o;
            zone.p = polycodec::base64_encode(&p);
            let mut packed_holes = Vec::with_capacity(zone.holes.len());
            for hole in &zone.holes {
                let (ho, hp) = polycodec::encode_polygon(hole);
                packed_holes.push((ho, polycodec::base64_encode(&hp)));
            }
            zone.h = packed_holes;
        }
    }

    /// Tree + cost statistics.  Cost fields require `refz` to still be populated,
    /// so call this before `annotate`.
    pub fn tree_stats(&self, op_limit: Option<i64>) -> Stats {
        let (mut nodes, mut leaves, mut refs, mut erefs) = (0usize, 0usize, 0usize, 0usize);
        let (mut max_depth, mut max_leaf_refs, mut max_cost, mut over) = (0usize, 0usize, 0i64, 0usize);
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
                    let cost = leaf_cost(&node.refz, &self.zones, &cell, d);
                    max_cost = max_cost.max(cost);
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
        match segment_crossings(center, point, &zone.outer, &cand.e) {
            None => return geom::rings_contain_point(&zone.outer, &zone.holes, point),
            Some(d) => {
                if cand.w + d == 0 {
                    return false;
                }
            }
        }
        for hc in &cand.h {
            let hole = &zone.holes[hc.i];
            match segment_crossings(center, point, hole, &hc.e) {
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

fn count_edges_in_cell(ring: &[Pt], cell: &Aabb) -> usize {
    let n = ring.len();
    (0..n).filter(|&i| geom::edge_touches_aabb(cell, ring[i], ring[(i + 1) % n])).count()
}

/// Vertex comparisons a lookup performs for one candidate in one cell.
pub fn localized_edge_count(zone: &Zone, cell: &Aabb) -> usize {
    let mut n = count_edges_in_cell(&zone.outer, cell);
    for (hi, hole) in zone.holes.iter().enumerate() {
        if geom::aabbs_overlap(cell, &zone.hole_aabbs[hi]) {
            n += count_edges_in_cell(hole, cell);
        }
    }
    n
}

/// Worst-case lookup cost, in ops, for a leaf holding `refz` at `cell`/`depth`.
pub fn leaf_cost(refz: &[usize], zones: &[Zone], cell: &Aabb, depth: usize) -> i64 {
    let mut edges = 0usize;
    for &zi in refz {
        edges += localized_edge_count(&zones[zi], cell);
    }
    depth as i64 * TRAVERSAL_OP + edges as i64 * VERTEX_OP
}

// Compress edge indices intersecting `cell` into inclusive [first, last] runs.
fn edge_runs(ring: &[Pt], cell: &Aabb) -> Vec<(u32, u32)> {
    let mut runs: Vec<(u32, u32)> = Vec::new();
    let n = ring.len();
    for i in 0..n {
        if geom::edge_touches_aabb(cell, ring[i], ring[(i + 1) % n]) {
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
