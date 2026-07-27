// Cross-check two timezone artifacts — typically a lossless build (`--vw=1.0`)
// against a lossy one (`--vw < 1.0`) — by resolving a dense lon/lat grid through
// both and reporting where they disagree.
//
// The build's own `--verify` only checks the quadtree against brute force on the
// SAME geometry, so it cannot see a region that lossy simplification dropped
// entirely (both tree and brute force then agree it is "ocean").  This tool
// compares the *candidate* artifact against a *reference* one, so a hole shows up
// as a cluster of "reference had a timezone, candidate says ocean" for one zone.
//
// Usage:
//   node tzverify.js <reference> <candidate> [--grid=DEG] [--max-error=PCT]
//
//   <reference>  lossless (or baseline) artifact — JSON or binary (auto-detected)
//   <candidate>  the artifact to check against it
//   --grid=DEG   grid step in degrees                              [default: 0.25]
//   --max-error=PCT  fail (exit 1) if disagreements exceed this % of land samples
//                                                                  [default: 0.5]
//
// Both files must share the same quantization domain (they always do).  Timezones
// are compared by NAME, so the binary format's remapped tzids don't matter.

"use strict";

const fs = require("fs");
const tzlookup = require("./tzlookup");
const { BinaryReader } = require("./tzlookup_binary.js");

// Return { kind, resolve(lon, lat) -> tzName | null } for a JSON or binary file.
function makeResolver(path) {
  const buf = fs.readFileSync(path);
  const isBinary = buf.length >= 4 &&
    buf[0] === 0x54 && buf[1] === 0x5A && buf[2] === 0x51 && buf[3] === 0x54; // "TZQT"
  if (isBinary) {
    const br = new BinaryReader(buf);
    return { kind: "binary", resolve: function (lon, lat) { return br.resolve(lon, lat).name; } };
  }
  const db = JSON.parse(buf.toString("utf8"));
  db.tzById = {};
  for (const name in db.tz) { db.tzById[db.tz[name].id] = db.tz[name]; }
  const sx = db.quant.xScale, sy = db.quant.yScale;
  return {
    kind: "json",
    resolve: function (lon, lat) {
      const z = tzlookup.resolve(db, [Math.round(sx * lon), Math.round(sy * lat)]).zone;
      return z ? db.tzById[z.tzid].n : null;
    }
  };
}

function main() {
  const args = process.argv.slice(2);
  const files = [];
  let grid = 0.25, maxError = 0.5;
  for (const a of args) {
    if (a.startsWith("--grid=")) { grid = parseFloat(a.slice(7)); }
    else if (a.startsWith("--max-error=")) { maxError = parseFloat(a.slice(12)); }
    else if (a === "-h" || a === "--help") { files.length = 0; break; }
    else { files.push(a); }
  }
  if (files.length !== 2 || !(grid > 0)) {
    console.error("Usage: node tzverify.js <reference> <candidate> [--grid=DEG] [--max-error=PCT]");
    process.exit(2);
  }

  const ref = makeResolver(files[0]);
  const cand = makeResolver(files[1]);
  console.log("reference: " + ref.kind + " " + files[0]);
  console.log("candidate: " + cand.kind + " " + files[1]);
  console.log("grid: " + grid + "°");

  // Sample the world.  Longitude is half-open [-180, 180); latitude [-90, 90).
  const nLon = Math.round(360 / grid), nLat = Math.round(180 / grid);
  let samples = 0, land = 0, disagree = 0;
  let toOcean = 0, toLand = 0, toOther = 0;
  // Per reference-zone: how many samples changed, split by kind, with an example.
  const byZone = new Map();
  const note = function (zone, kind, lon, lat, other) {
    let e = byZone.get(zone);
    if (!e) { e = { ocean: 0, other: 0, example: null }; byZone.set(zone, e); }
    e[kind]++;
    if (!e.example) { e.example = { lon: lon, lat: lat, other: other }; }
  };

  for (let i = 0; i < nLat; i++) {
    const lat = -90 + (i + 0.5) * grid;
    for (let j = 0; j < nLon; j++) {
      const lon = -180 + (j + 0.5) * grid;
      const a = ref.resolve(lon, lat);
      const b = cand.resolve(lon, lat);
      samples++;
      if (a !== null) { land++; }
      if (a === b) { continue; }
      disagree++;
      if (a !== null && b === null) { toOcean++; note(a, "ocean", lon, lat, null); }
      else if (a === null && b !== null) { toOther++; } // ocean -> land (candidate gained area)
      else { toLand++; note(a, "other", lon, lat, b); } // land -> different land
    }
  }

  console.log("\nsamples: " + samples + "   land: " + land +
    "   disagreements: " + disagree + " (" + pct(disagree, samples) + " of all, " +
    pct(disagree, land) + " of land)");
  console.log("  reference-tz → ocean: " + toOcean +
    "     → different tz: " + toLand + "     ocean → tz: " + toOther);

  // The land->ocean / land->other clusters are the tell-tale of a bad simplification.
  const zones = Array.from(byZone.entries())
    .sort(function (x, y) { return (y[1].ocean + y[1].other) - (x[1].ocean + x[1].other); });
  if (zones.length) {
    console.log("\nmost-affected timezones (reference name):");
    for (const [name, e] of zones.slice(0, 15)) {
      const ex = e.example;
      console.log("  " + name.padEnd(28) + " " + (e.ocean + e.other) +
        "  (→ocean " + e.ocean + ", →other " + e.other + ")" +
        "  e.g. " + fmt(ex.lon) + "," + fmt(ex.lat) +
        (ex.other ? " → " + ex.other : " → ocean"));
    }
  }

  const errPct = land ? (100 * disagree / land) : 0;
  // A cluster of land->ocean for a single zone is the strongest hole signal —
  // flagged even when the overall error rate is under threshold.
  const worstOcean = zones.reduce(function (m, z) { return Math.max(m, z[1].ocean); }, 0);
  const holeSuspect = worstOcean >= 10;
  console.log("\nVERDICT: " + (errPct <= maxError ? "PASS" : "FAIL") +
    "  (land error " + errPct.toFixed(3) + "%, threshold " + maxError + "%)" +
    (holeSuspect ? "  ⚠ possible dropped region (see land→ocean clusters above)" : ""));
  process.exit(errPct <= maxError && !holeSuspect ? 0 : 1);
}

function pct(n, d) { return d ? (100 * n / d).toFixed(3) + "%" : "0%"; }
function fmt(v) { return v.toFixed(2); }

main();
