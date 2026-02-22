const fs = require('fs');
const path = require('path');

test('vendor/apriltag has a usable build (JS or wasm)', () => {
  const dir = path.join(__dirname, '../../vendor/apriltag');
  const hasJs = fs.existsSync(path.join(dir, 'apriltag_wasm.js')) || fs.existsSync(path.join(dir, 'apriltag.js'));
  const hasWasm = fs.existsSync(path.join(dir, 'apriltag_wasm.wasm')) || fs.existsSync(path.join(dir, 'apriltag.wasm'));
  // JS is required for worker loading; wasm optional depending on build
  expect(hasJs).toBe(true);
  // if wasm is missing that's acceptable (some bundles are JS-only), so just log
  // but assert at least one of the files exists in the vendor dir
  expect(hasJs || hasWasm).toBe(true);
});