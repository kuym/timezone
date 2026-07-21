// Trivial static file server for the demo.  Run `node server.js` then open
// http://localhost:8000/tzview.html.  Serves files from this directory only.

const http = require("http");
const fs = require("fs");
const path = require("path");

const ROOT = __dirname;
const PORT = process.env.PORT || 8000;

const TYPES = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".css": "text/css; charset=utf-8"
};

http.createServer(function(req, res) {
  // Strip the query string and default "/" to the viewer.
  let rel = decodeURIComponent(req.url.split("?")[0]);
  if (rel == "/") { rel = "/tzview.html"; }

  // Resolve within ROOT and reject anything that escapes it (e.g. "../").
  const file = path.join(ROOT, rel);
  if (file != ROOT && !file.startsWith(ROOT + path.sep)) {
    res.writeHead(403).end("Forbidden");
    return;
  }

  fs.readFile(file, function(err, data) {
    if (err) {
      res.writeHead(err.code == "ENOENT" ? 404 : 500).end(err.code || "Error");
      return;
    }
    res.writeHead(200, {"Content-Type": TYPES[path.extname(file)] || "application/octet-stream"});
    res.end(data);
  });
}).listen(PORT, function() {
  console.log("serving " + ROOT + " at http://localhost:" + PORT + "/tzview.html");
});
