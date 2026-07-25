# `quadtree.json` schema

The output artifact produced by `tzconvert.js`: the world's timezone polygons,
quantized to a fixed-point integer grid, packed into a quadtree for fast
`(latitude, longitude) → timezone` lookup. It is plain JSON (UTF-8), produced by
`node tzconvert.js quadtree.json` and consumed by `tzlookup.js` (and the
`tzview.html` demo).

This document describes the on-disk format so a reader can be written in any
language. The reference reader is `tzlookup.js`; the reference writer is
`tzconvert.js` (`annotateLeaves` + `purge`).

---

## 1. Coordinate system

All geometry is stored as **fixed-point integers**, never floating-point
degrees. Longitude maps to `x`, latitude maps to `y`:

```
x = round(longitude * xScale)      xScale = 524288 / 180  ≈ 2912.711
y = round(latitude  * yScale)      yScale = 262144 /  90  ≈ 2912.711
```

Both axes share the same scale, so one unit ≈ 0.000343° ≈ 38 m at the equator.
The domain is a half-open box, `x ∈ [-524288, 524288)`, `y ∈ [-262144, 262144)`
— i.e. `[-2^19, 2^19) × [-2^18, 2^18)`. To convert back:

```
longitude = x / xScale             latitude = y / yScale
```

The exact constants are stored in the file (see `quant`), so a reader needs no
compiled-in values.

---

## 2. Top level

```jsonc
{
  "quant":    { ... },   // quantization parameters (§3)
  "rootCell": [ ... ],   // bounds of the whole world (§4)
  "quadtree": { ... },   // the spatial index (§5)
  "arcs":     [ ... ],   // shared polygon boundary arcs (§6)
  "zones":    [ ... ],   // polygon records (reference arcs), indexed by zone id (§6)
  "tz":       { ... }    // timezone records, keyed by tzid name (§7)
}
```

There are two distinct id spaces, and it is important not to conflate them:

- **zone id** — an index into the `zones` array. One *polygon* (one ring plus
  its holes). A timezone made of several disjoint landmasses has several zones.
- **tzid** — an index into the timezones, and the `id` field of a `tz` record.
  The `zones[z].tzid` field maps a zone to the timezone it belongs to.

For the current dataset: 1195 zones across 426 timezones.

---

## 3. `quant`

Self-describing quantization parameters. A reader should use these rather than
hard-coding constants.

```json
{
  "xMin": -524288, "xMax": 524288,
  "yMin": -262144, "yMax": 262144,
  "xScale": 2912.711111111111,
  "yScale": 2912.711111111111,
  "maxDepth": 16
}
```

| field | meaning |
|---|---|
| `xMin`, `xMax` | half-open longitude range in quantized units, `[xMin, xMax)` |
| `yMin`, `yMax` | half-open latitude range, `[yMin, yMax)` |
| `xScale`, `yScale` | units per degree; `x = round(lon*xScale)`, `y = round(lat*yScale)` |
| `maxDepth` | maximum quadtree subdivision depth (see §5) |

---

## 4. `rootCell`

The bounding box of the whole quadtree, as `[[xMin, yMin], [xMax, yMax]]`:

```json
[[-524288, -262144], [524288, 262144]]
```

A **cell** is always `[[xLo, yLo], [xHi, yHi]]` and is **half-open**: it covers
`xLo ≤ x < xHi` and `yLo ≤ y < yHi`. Because the extents are powers of two, every
cell bound is a multiple of that cell's size, so a cell's midpoint
`(lo + hi) / 2` is always an exact integer — `(lo + hi) >> 1`, `floor`, and
truncate-toward-zero all agree. A port may use an arithmetic right shift with no
special cases.

### Subdividing a cell

A cell splits into four child cells. Quadrants use standard math numbering
(counter-clockwise from the top-right):

```
        yHi
    +---------+---------+
    |    1    |    0    |     0 = NE  (+x, +y)
    |  (NW)   |  (NE)   |     1 = NW  (-x, +y)
 my +---------+---------+     2 = SW  (-x, -y)
    |    2    |    3    |     3 = SE  (+x, -y)
    |  (SW)   |  (SE)   |
    +---------+---------+
   xLo       mx        xHi
```

with `mx = (xLo + xHi) >> 1`, `my = (yLo + yHi) >> 1`:

```
child 0 (NE): [[mx,  my ], [xHi, yHi]]
child 1 (NW): [[xLo, my ], [mx,  yHi]]
child 2 (SW): [[xLo, yLo], [mx,  my ]]
child 3 (SE): [[mx,  yLo], [xHi, my ]]
```

This is the single source of truth for the split; it must match
`tzlookup.splitCell` exactly. To descend toward a point, pick the child whose
half-open bounds contain it: `q = (x >= mx ? (y >= my ? 0 : 3) : (y >= my ? 1 : 2))`.

---

## 5. `quadtree`

A tree of **nodes**. The root node is `quadtree`; children are reached through
`q`. Every node is an object with up to three optional members — **an absent
member means an empty list**, which is how the file stays compact (most nodes
are `{}` or carry just one of these):

```jsonc
{
  "q":    [ node, node, node, node ],   // 4 children, indexed as in §4; absent at a leaf
  "eref": [ zoneId, ... ],              // zones that fully cover this cell (definitive)
  "ref":  [ candidate, ... ]            // zones that partially overlap (need a test)
}
```

- A node with a `q` array is **internal**; a node without one is a **leaf**.
  When present, `q` always has exactly four entries (an empty child is `{}`).
- **`eref`** ("enclosing ref") lists zones whose polygon **provably covers the
  entire cell**. Any point in the cell is inside that zone with no geometry test.
  `eref` may appear on internal *or* leaf nodes and at any depth.
- **`ref`** lists **candidate** zones whose polygon only partially overlaps the
  cell. These require a point-in-polygon test (§5.2). Candidates live **only on
  leaves**.

Subdivision rule (from the writer): a leaf is split whenever its **lookup cost**
exceeds a configurable op budget, so the shape of the tree is not fixed — it
depends on the `--max-ops` / `--max-splits` the artifact was built with. The
cost of resolving any point in a leaf is modelled as

```
cost = depth * 1  +  localizedEdges * 2
       \_______/     \_______________/
       traversal      vertex comparisons in the point-in-polygon test
       (1 op/level)   (2 ops/edge, summed over all candidates in the leaf)
```

where `localizedEdges` is the total number of ring edges the candidates
contribute to this cell (the lengths of their `e` runs — see §5.1). A leaf may
hold many candidates as long as they are collectively cheap to test; a leaf with
one edge-heavy candidate may be split hard. Splitting stops at `maxDepth` (16)
or when the split budget runs out, so a few leaves can remain over budget (the
build reports `leavesOverLimit`). `eref` zones need no test and do not count
toward the cost.

Example leaf with one candidate:

```json
{ "ref": [ { "z": 629, "w": 0, "e": [[8, 10], [17, 40]] } ] }
```

Example leaf fully covered by zone 509:

```json
{ "eref": [509] }
```

Example: a cell where two zones both enclose it (overlapping timezones do
occur — see §5.2):

```json
{ "eref": [559, 621] }
```

### 5.1 The `candidate` object (an entry of `ref`)

```jsonc
{
  "z": 629,                       // zone id (index into `zones`)
  "w": 0,                         // winding number of the CELL CENTER vs the full outer ring
  "e": [[8, 10], [17, 40]],       // outer-ring edge runs that intersect this cell
  "h": [                          // OPTIONAL: only holes that cross this cell
    { "i": 1, "w": 0, "e": [[8, 9]] }
  ]
}
```

| field | meaning |
|---|---|
| `z` | zone id — index into `zones` |
| `w` | signed winding number of this **cell's center** with respect to the zone's **full outer ring** (usually `0`, `+1`, or `-1`) |
| `e` | list of `[firstEdge, lastEdge]` inclusive index runs into the outer ring; these are the edges that pass through this cell. Edge `i` connects outer vertex `i` to vertex `i+1` (mod n). May be empty `[]` if only a hole crosses the cell |
| `h` | present only if one or more holes cross this cell; array of `{i, w, e}` where `i` is the hole index (into `zones[z].h` / the decoded holes), and `w`/`e` mirror the outer fields for that hole ring |

`w` and `e` together enable the **localized point-in-polygon test** (§5.2):
storing only the local edges plus a precomputed winding number lets a lookup test
a handful of edges instead of the whole (possibly thousands-of-vertices) ring.

Holes that do **not** cross the cell are omitted from `h` — they cannot change
the answer for any point inside the cell.

### 5.2 How a lookup uses the tree

Given a quantized query point `p = [x, y]` (reference: `tzlookup.resolve`):

1. **Descend.** Start at `rootCell` / `quadtree`. At each internal node, pick the
   child quadrant containing `p` (§4), recurse into `q[quadrant]` with the child
   cell. Collect every `eref` seen along the path. Stop at a leaf; collect its
   `ref` candidates.
2. **Definite matches.** Every `eref` zone contains `p` by construction — no test
   needed.
3. **Candidate tests.** For each `ref` candidate, test whether `p` is inside the
   zone using the localized method:

   ```
   center = cellCenter(leafCell)
   wn = candidate.w + signedCrossings(segment center→p, outer edges in candidate.e)
   inside_outer = (wn != 0)
   ```

   then subtract holes: for each `h` entry, the same formula with `h.w` / `h.e`;
   if the point is inside any hole, it is outside the zone. The segment
   `center→p` lies inside the (convex) cell, so it can only cross edges that
   intersect the cell — exactly the stored subset, which is why the local edges
   suffice. This yields the same result as a winding-number test over the full
   ring.
4. **Pick the smallest.** Zones can genuinely overlap — disputed regions (e.g.
   Kashmir) and the enclave-as-hole representation (Vatican inside Rome). Every
   zone containing `p` is either an `eref` on the path or a passing `ref`
   candidate; the answer is the zone with the **smallest area** (`zones[z].a`)
   among them. If the set is empty, `p` is in no mapped timezone (ocean).

A reader that does not care about the optimization can ignore `w`/`e`/`h` on the
candidates, decode each candidate zone's full geometry (§6), and run an ordinary
winding-number point-in-polygon test — the answer is identical.

---

## 6. `arcs` and `zones`

Adjacent zones share ~80% of their boundary edges (the border between two
timezones is one line, not two). The geometry is therefore stored **once** as a
set of shared **arcs** (the TopoJSON model), and each zone references the arcs
that make up its rings rather than repeating the vertices.

### 6.0 `arcs`

An array of shared boundary polylines, indexed by **arc id** (the array index).
Each arc is stored exactly like a ring used to be — an origin `o` plus a base64
packed delta stream `p` (§6.1) — decoding to a list of absolute vertices:

```jsonc
{ "o": [-25054, 18948], "p": "1OzNEIKX8t3W…" }
```

### 6.1a `zones`

An array of polygon records, indexed by **zone id** (`zones[id].id == id`). A
ring is a list of **signed arc references** instead of vertices:

```jsonc
{
  "id":    0,
  "aabb":  [[-25054, 12123], [-7261, 31283]],  // bounding box, quantized units
  "a":     463079127,                           // area (unsigned, doubled) — tie-break key
  "tzid":  0,                                   // which timezone (index into `tz`)
  "outer": [ 12, -6, 40 ],                      // outer ring: arc refs (see below)
  "h":     [ [ 41, -13 ], ... ]                 // OPTIONAL: holes, each a list of arc refs
}
```

| field | meaning |
|---|---|
| `id` | zone id; equal to the array index |
| `aabb` | axis-aligned bounding box `[[xLo,yLo],[xHi,yHi]]` in quantized units; a cheap reject before decoding |
| `a` | unsigned doubled polygon area of the outer ring, in quantized units²; used only as the smallest-wins tie-break (§5.2) |
| `tzid` | the timezone this polygon belongs to — an index into the `tz` records (`tz[name].id`) |
| `outer` | the outer ring as a list of **arc references**: `i` means `arcs[i]` forwards, `-i-1` means `arcs[i]` reversed |
| `h` | present only if the polygon has holes; each hole is its own list of arc references |

**Rebuilding a ring.** Concatenate the referenced arcs in order (reversing an
arc's vertices when its ref is negative), dropping the vertex each arc shares
with the previous one, then drop the final vertex (the closure back to the
start). Reference: `tzlookup.reconstructRing` (JS) / `topology::reconstruct`
(Rust). Because a shared arc is simplified once, both zones that use it stay
seam-consistent.

Zones with holes carry `h`; the sample dataset has zones with up to 11 holes.
Holes are the interior boundaries (lakes, enclaves) subtracted from the outer
ring.

### 6.1 Decoding an arc's `o` + `p` into vertices

An arc is stored as an explicit integer **origin** (`o`) plus a **delta stream**
(`p`): base64 → bytes → a sequence of signed `(dx, dy)` deltas between
consecutive vertices. Reconstruct absolute vertices by running sum:

```
v[0] = o
v[i] = v[i-1] + delta[i-1]
```

The origin is stored separately (not as the first delta) so that no stream value
ever has to hold a full-range absolute coordinate — every delta is small.
Reference: `polycodec.decodePolygon` / `polycodec.decodeVectors`.

**Wire format.** After base64-decoding `p`, read deltas one at a time. The top
two bits of the first byte of each delta select one of four variable-width forms;
both components are two's-complement signed and must be sign-extended on decode:

| tag (top 2 bits) | bytes | bits per component | layout |
|---|---|---|---|
| `11` | 2 | 7  | `11aaaaaa abbbbbbb` |
| `10` | 3 | 11 | `10aaaaaa aaaaabbb bbbbbbbb` |
| `00` | 5 | 19 | `00aaaaaa aaaaaaaa aaaaabbb bbbbbbbb bbbbbbbb` |
| `01` | 6 | 23 | `01aaaaaa aaaaaaaa aaaaaaaa aaabbbbb bbbbbbbb bbbbbbbb` |

`a` is `dx`, `b` is `dy`. The encoder always picks the smallest form that fits
both components. The 23-bit form is a guaranteed fallback (the whole domain spans
2^20 units, so no delta can exceed it). A ring's vertices do **not** repeat the
first vertex at the end — closure is implicit.

---

## 7. `tz`

An object keyed by IANA timezone name, one entry per timezone:

```json
"America/New_York": { "id": 151, "n": "America/New_York", "ref": [321, 322, 323] }
```

| field | meaning |
|---|---|
| key | the IANA `tzid` string (also duplicated in `n`) |
| `id` | the timezone's numeric id; this is what `zones[z].tzid` refers to |
| `n` | the IANA name (same as the key) |
| `ref` | the zone ids belonging to this timezone (its disjoint polygons) |

To go from a resolved zone back to a human-readable name: build a reverse map
`tzById[tz[name].id] = tz[name]` once, then `tzById[zones[z].tzid].n`.

---

## 8. Worked example

Resolve longitude −0.1276, latitude 51.5074 (London):

1. Quantize: `x = round(-0.1276 * 2912.711) = -372`, `y = round(51.5074 * 2912.711) = 150026`.
2. Descend the quadtree from `rootCell`, choosing a quadrant per level, to a leaf
   at depth 6. No `eref` was collected on the path; the leaf has one `ref`
   candidate for the zone whose `tzid` names `Europe/London`.
3. Localized test: `candidate.w` plus the crossings of the ~58 stored outer edges
   (out of the full ring's ~1115) gives a non-zero winding number → inside.
4. No other match → the answer is `Europe/London`.

Points in the open ocean descend to a leaf with no `eref` and no passing
candidate, and resolve to "no timezone".

---

## 9. Relationship to the source files

| file | role |
|---|---|
| `tzconvert.js` | builds the artifact: quantize → simplify → quadtree insert → `annotateLeaves` (adds `w`/`e`) → `purge` (packs geometry, drops build-only fields) |
| `polycodec.js` | the `o`/`p` codec (§6.1); shared by writer and reader |
| `tzlookup.js` | the reader: `splitCell`, tree descent, localized point-in-polygon, smallest-wins resolution |
| `tzmap.js` | quantization domain and the `quant` constants |
| `tzview.html` | browser demo that loads and queries the artifact |

For the design rationale, the bug history, and notes toward a compact binary
format for a C/Rust port, see `ANALYSIS.md`.
