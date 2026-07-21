// STATUS: abandoned prototype, kept as a design sketch for the binary format.
// NOT part of the build or the test suite.
//
// It now loads again (quadtree.json is real JSON rather than a `const
// quadtree=` JS file), but the flattening itself is still broken: off() emits
// each jump offset as exactly two hex digits, so any subtree longer than 255
// units overflows the field.  pad() does not catch it either -- for a 3-digit
// value the substr() index goes negative-ish and yields an empty pad, so the
// field silently becomes variable-width and the whole stream loses framing.
// Every offset near the root of the real tree exceeds 255.
//
// It also predates the current artifact: zone geometry now lives in `zones[].o`
// and `zones[].p` (origin plus packed deltas), which this does not emit at all,
// so the output describes tree structure only.
//
// The mmap-friendly binary format planned for the Rust encoder supersedes this;
// see ANALYSIS.md section 5.
const qt = require("./quadtree.json").quadtree;

function pad(v, len) {
	return "0000000000000000".substr(16 - (len - v.length)) + v;
}
function com(arr) {
	return pad(arr.length.toString(16), 2) + arr.map(function(v) {
			return pad(v.toString(16), 4);
		}).join("");
}
function off(v) {
	return pad(v.toString(16), 2);
}
function out(node) {
	let str = "";
	if(node.eref) {
		str += com(node.eref);
	}
	if(node.ref) {
		str += com(node.ref);
	}
	if(node.q) {
		const q = node.q.map(out);
		str += off(q[0].length + 4) + off(q[0].length + q[1].length + 2) + off(q[0].length + q[1].length + q[2].length) + q.join("");
	}
	return str;
}

process.stdout.write(out(qt));
