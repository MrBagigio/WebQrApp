// secure-server.js
// Minimal static file server with Basic HTTP auth (no external deps required)
const http = require('http');
const fs = require('fs');
const path = require('path');
const url = require('url');

const HOST = '0.0.0.0';
const PORT = process.env.PORT ? Number(process.env.PORT) : 8080;

// Credentials (as requested)
const USER = 'expear';
const PASS = '123';
const AUTH_HEADER = 'Basic ' + Buffer.from(`${USER}:${PASS}`).toString('base64');

function send401(res) {
  res.writeHead(401, { 'WWW-Authenticate': 'Basic realm="Protected"', 'Content-Type': 'text/plain' });
  res.end('Unauthorized');
}

function basicAuth(req, res) {
  const h = req.headers['authorization'];
  if (!h || h !== AUTH_HEADER) {
    send401(res);
    return false;
  }
  return true;
}

function serveFile(req, res) {
  try {
    let parsed = url.parse(req.url || '/');
    let pathname = decodeURIComponent(parsed.pathname || '/');
    if (pathname === '/') pathname = '/index.html';
    const safeSuffix = path.normalize(pathname).replace(/^\.+/, '');
    const fileLoc = path.join(process.cwd(), safeSuffix);

    if (!fileLoc.startsWith(process.cwd())) {
      res.writeHead(403);
      res.end('Forbidden');
      return;
    }

    fs.stat(fileLoc, (err, stats) => {
      if (err) {
        res.writeHead(404);
        res.end('Not found');
        return;
      }

      if (stats.isDirectory()) {
        // try index.html
        const idx = path.join(fileLoc, 'index.html');
        if (fs.existsSync(idx)) {
          streamFile(idx, res);
        } else {
          res.writeHead(403);
          res.end('Forbidden');
        }
        return;
      }

      streamFile(fileLoc, res, stats);
    });
  } catch (e) {
    res.writeHead(500);
    res.end('Server error');
  }
}

function streamFile(filePath, res, stats) {
  const ext = path.extname(filePath).toLowerCase();
  const map = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'application/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.wasm': 'application/wasm',
    '.fbx': 'application/octet-stream'
  };
  const contentType = map[ext] || 'application/octet-stream';
  const stat = stats || fs.statSync(filePath);

  res.writeHead(200, {
    'Content-Type': contentType,
    'Content-Length': stat.size,
    'Cache-Control': 'public, max-age=3600'
  });
  const stream = fs.createReadStream(filePath);
  stream.pipe(res);
  stream.on('error', () => {
    if (!res.headersSent) { res.writeHead(500); }
    res.end();
  });
}

const server = http.createServer((req, res) => {
  // enforce basic auth for all routes
  if (!basicAuth(req, res)) return;
  serveFile(req, res);
});

server.listen(PORT, HOST, () => {
  console.log(`Secure static server running on http://${HOST}:${PORT} (user: ${USER})`);
});

process.on('SIGINT', () => process.exit(0));
