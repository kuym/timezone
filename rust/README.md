# tzconvert (Rust)

A Rust port of the timezone quadtree compressor (`tzconvert.js` and its
dependencies). It reads the GeoJSON timezone dataset, quantizes and simplifies
the polygons, builds a cost-model-driven quadtree, verifies it against brute
force, and serializes it.

**It is byte-for-byte identical to the JavaScript compressor.** For any given
set of options the `quadtree.json` it emits is exactly the same file
`tzconvert.js` produces (verified across op-limit, split-budget, and epsilon
variations). See "Parity" below.

## Build & run

```sh
cd rust
cargo build --release            # offline if the crates are cached

# run from the PROJECT ROOT so the default --input path resolves
cd ..
./rust/target/release/tzconvert [options] quadtree.json
```

### Options (identical semantics to tzconvert.js)

| flag | default | meaning |
|---|---|---|
| `--max-ops=N` | 500 | per-leaf lookup cost budget; leaves over it split |
| `--max-splits=N` | unlimited | hard cap on splits; **takes precedence** over `--max-ops` |
| `--epsilon=N` | 8 | RDP simplification tolerance, in quantized units |
| `--verify=N` | 3000 | random points cross-checked against brute force (0 disables) |
| `--format=json\|binary` | json | output serializer (binary is a **stub**, see below) |
| `--input=PATH` | data/combined.json | GeoJSON FeatureCollection |
| positional | quadtree.json | output path |

```sh
cargo test --release             # 12 unit tests (geom, quant, codec, cells)
```

## Module map (mirrors the JS files)

| Rust | JS | role |
|---|---|---|
| `geom.rs` | geom.js | AABB / winding / segment-in-box / area, winding-order independent |
| `quant.rs` | tzmap.js | quantization domain, `quantize` (JS-matching rounding), `simplify_rdp` |
| `polycodec.rs` | polycodec.js | 2/3/5/6-byte delta codec with round-trip assert, base64 |
| `lookup.rs` | tzlookup.js | cell split / center / quadrant, localized segment-crossing test |
| `build.rs` | tzconvert.js | classify, cost model, greedy `subdivide`, `annotate`, `pack`, `verify` |
| `serialize/` | (the writer) | `Serializer` trait + JSON impl + binary stub |
| `main.rs` | the CLI driver | arg parsing, GeoJSON input, orchestration |

## Serializers

Both formats implement one `Serializer` trait (`serialize/mod.rs`), so the
driver is format-agnostic and a new format is one `impl`:

- **`json`** — complete; produces the exact `quadtree.json` schema (see
  `../quadtree.md`), compact and with the same key ordering as the JS writer.
- **`binary`** — full-artifact compact format, **complete** (`is_complete()` is
  true). ~42% smaller than `json` (1.08 MB vs 1.87 MB) — no base64, plus the
  quadtree optimizations. Four sections after a fixed header (magic `TZQT`,
  version, quant block, counts):

  1. **quadtree** — reuses the `quad` format's primitives (the `q` varint in
     `serialize/qvarint.rs`, plus recursive length-prefixed skippable nodes) but
     encodes the real cost-model tree: each node's `eref` list and, at leaves, the
     `ref` candidates with their localized point-in-polygon data (winding, edge
     runs, holes). Four encodings squeeze it 30% (104,637 → 73,077 bytes):
     - **P1** packs each candidate's winding, run count and hole presence into one
       `q` (winding is only ever −1/0/+1, holes almost always absent);
     - **P2** folds a node's internal/eref flags and leaf ref-count into one `q`;
     - **P4/P5** renumber zones by descending reference count (**rank**) so the
       busiest get 1-byte ids, and delta-code the sorted candidate ranks in a leaf.
     - *(Not done, to keep the decoder fast/low-memory: dropping the per-leaf
       length prefix — parse-to-skip; and omitting the localized data to recompute
       it at load — O(ring) per candidate.)*
  2. **zones** — one record per zone in rank order: `tzid`, area, aabb, then each
     ring as an origin + the **same packed delta stream as `json` (`polycodec`),
     but stored raw (no base64) behind a length prefix**.
  3. **tz names** — length-prefixed UTF-8, indexed by tzid.

  Two integer encodings appear: the `q` varint (small/paddable values — tree
  structure) and, in the zones section, an **exact** varint (`q` byte-count + that
  many LE bytes) for polygon lengths, coordinates and areas — these are large and
  of arbitrary parity, which the `q` 3-byte form (even-only ≥16512) cannot
  represent. Every section is validated by a decode round-trip against the source.
  The full layout is documented at the top of `serialize/binary.rs`.

  The reference reader is `../tzlookup_binary.js` (Node + browser): it parses the
  binary and resolves points by reusing `tzlookup.js`'s descent + localized
  point-in-polygon (agreeing 100% with the JSON reader). Run
  `node tzlookup_binary.js tz.bin 38.5,-98.5` (arguments are `lat,lon`).

- **`quad`** (experimental, Rust-only) — a very compact quadtree with **no
  polygons**. It builds its own rough tree from the zone geometry: split until a
  cell is homogeneous (one tzid) or its longest edge (in metres, at the cell's
  latitude) drops below `--leaf-km` (default 10 km); at that limit the tzid with
  the most area in the cell wins (8×8 area sampling via the localized winding
  test). Every leaf resolves to exactly one tzid, so lookups are pure integer
  cell descent with no geometry.

  ```
  ./rust/target/release/tzconvert --input=data/combined.json --format=quad --leaf-km=10 tz.quad
  ```

  Size vs accuracy (agreement with exact smallest-area lookup; misses are within
  ~`leaf-km` of a border):

  | --leaf-km | size | accuracy |
  |---|---|---|
  | 50 | 133 KB | 98.7% |
  | 20 | 316 KB | 99.5% |
  | **10** | **685 KB** | **99.7%** |
  | 5 | 1.4 MB | 99.95% |

  **File layout** (see `serialize/quad.rs`):

  ```
  magic "TZQ3"                          4 bytes
  section 1 — quadtree: one recursive node
      node := len:q
              len == 0  -> ocean leaf: no timezone, nothing follows (1 byte)
              len == 1  -> leaf:       tzid:q follows
              len >= 2  -> internal:   len bytes = 4 child nodes (quadrants 0..3)
  section 2 — tzid table:
      count:q, then count × ( nameLen:q, UTF-8 name )   — indexed by tzid
  ```

  `q` is a 3-form integer code selected by the top bits of the first byte:

  ```
    A  0aaaaaaa                     q = a           →   0 ..   127   (1 byte)
    B  10aaaaaa aaaaaaaa            q = a + 128      → 128 .. 16511   (2 bytes)
    C  11aaaaaa aaaaaaaa aaaaaaaa   q = 2a + 16514   → even, 16514.. (3 bytes)
  ```

  (equivalently the "value" `(q+1)*2` is `(a+1)*2` / `a*2+258` / `a*4+33030`).
  `q` is exact below 16512, so tzids, name lengths, and most node lengths pad
  nothing; only node payloads ≥ 16512 bytes round up to the next even value.
  Node lengths reach form C; tzids only ever use A or B. Max encodable `q` is
  8_404_120, capping the tree at ~8 MB (use a larger `--leaf-km` if exceeded).

  **ocean & tzid sorting**: open ocean is the single most common leaf, so it gets
  a dedicated 1-byte encoding (`len == 0`, no tzid, no table slot). The remaining
  real tzids are renumbered by descending reference count, so the most-used
  timezone gets id 0 and encodes in form A (1 byte). A leaf of length 0 is ocean;
  a leaf of length 1 carries its tzid.

  The internal node length prefix lets a reader skip whole subtrees, so lookup is
  O(depth) directly on the bytes. The reference reader is `../tzlookup_quad.js`
  (Node + browser), which also traces the byte offsets each lookup seeks to.

## Parity with the JS compressor

Two details make the output bit-identical rather than merely equivalent:

1. **Rounding.** `quantize` uses `floor(x + 0.5)` to match JavaScript's
   `Math.round` (round half toward +∞), not Rust's round-half-away-from-zero.
2. **RDP reciprocal.** `vdist_to_line` multiplies by `1/|ab|` rather than
   dividing by `|ab|`, matching `geom.js` in the last ULP — otherwise a rare
   rounding difference flips an RDP keep/drop decision and shifts edge indices.
3. **Heap order.** `JsHeap` replicates `tzconvert.js`'s `MaxHeap` array
   operations exactly, so equal-cost leaves pop in the same order and even a
   limited `--max-splits` budget yields the identical tree.

Notes:
- `serde_json` is used only to parse the input GeoJSON; the output is
  hand-written.
- The `--verify` sampler uses its own PRNG (its point sequence need not match
  the JS verifier — it independently proves the Rust build correct).
