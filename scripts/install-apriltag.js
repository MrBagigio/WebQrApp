const https = require('https');
const fs = require('fs');
const path = require('path');

const outDir = path.join(__dirname, '..', 'vendor', 'apriltag');
if (!fs.existsSync(outDir)) fs.mkdirSync(outDir, { recursive: true });

const candidates = [
  // arena XR demo distribution (prebuilt WASM + JS)
  'https://arenaxr.github.io/apriltag-js-standalone/apriltag_wasm.js',
  'https://arenaxr.github.io/apriltag-js-standalone/apriltag_wasm.wasm',
  // fallback CDN hints (may or may not exist)
  'https://cdn.jsdelivr.net/gh/arenaxr/apriltag-js-standalone@main/html/apriltag_wasm.js',
  'https://cdn.jsdelivr.net/gh/arenaxr/apriltag-js-standalone@main/html/apriltag_wasm.wasm'
];

function download(url, dest) {
  return new Promise((resolve, reject) => {
    const file = fs.createWriteStream(dest);
    https.get(url, (res) => {
      if (res.statusCode !== 200) {
        file.close();
        fs.unlinkSync(dest, { force: true });
        return reject(new Error(`HTTP ${res.statusCode}`));
      }
      res.pipe(file);
      file.on('finish', () => file.close(resolve));
    }).on('error', (err) => {
      try { fs.unlinkSync(dest); } catch (_) {}
      reject(err);
    });
  });
}

(async () => {
  console.log('Install AprilTag: trying to download prebuilt WASM/JS...');
  const mapping = [
    { url: candidates[0], name: 'apriltag_wasm.js' },
    { url: candidates[1], name: 'apriltag_wasm.wasm' }
  ];

  for (const item of mapping) {
    const dest = path.join(outDir, item.name);
    try {
      console.log('Downloading', item.url);
      await download(item.url, dest);
      console.log('Saved', dest);
    } catch (err) {
      console.warn('Failed to download', item.url, err.message);
    }
  }

  // If files are missing, write a small note and exit gracefully
  const jsExists = fs.existsSync(path.join(outDir, 'apriltag_wasm.js'));
  const wasmExists = fs.existsSync(path.join(outDir, 'apriltag_wasm.wasm'));
  if (!jsExists || !wasmExists) {
    console.log('\nCould not download a prebuilt AprilTag bundle automatically.');
    console.log('Please download a prebuilt apriltag_wasm.js + apriltag_wasm.wasm and place them in', outDir);
    process.exit(jsExists && wasmExists ? 0 : 2);
  }

  console.log('\nAprilTag build installed. Worker will use it when you enable AprilTag in the UI.');
})();
