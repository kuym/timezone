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
