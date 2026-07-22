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
- **`binary`** — a **stub**. It writes only a self-describing header (magic
  `TZQT`, version, quant block, counts); the node/zone/geometry sections are
  `TODO`, pending the format design in `../ANALYSIS.md` §6. `is_complete()`
  returns false, so the driver prints a loud warning and never presents the stub
  file as a real artifact. The scaffolding (trait wiring, CLI dispatch, LE
  `Writer` helpers, header layout) is in place for the eventual implementation.

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
  | 50 | 151 KB | 98.7% |
  | 20 | 356 KB | 99.5% |
  | **10** | **767 KB** | **99.7%** |
  | 5 | 1.6 MB | 99.95% |

  **File layout** (see `serialize/quad.rs`):

  ```
  magic "TZQ2"                          4 bytes
  section 1 — quadtree: one recursive node
      node := len:q
              len == 0  -> leaf:     tzid:q
              len  > 0  -> internal: len bytes = 4 child nodes (quadrants 0..3)
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

  **tzid sorting**: before encoding, tzids are renumbered by descending
  reference count, so the most-used timezones (and open ocean, usually the
  single commonest value) get the lowest ids and encode in form A (1 byte). This
  is worth ~17% over an unsorted UTF-8-style varint. A table entry with an empty
  name means "no timezone" (ocean).

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
