AprilTag (vendor)

This folder may contain a production AprilTag JS/WASM build (optional).

Recommended file layout when providing a real implementation:

  vendor/apriltag/
    ├─ apriltag.js       # JS glue that exposes `apriltag.detect(gray, w, h)`
    └─ apriltag.wasm     # optional WASM file used by the JS glue

If no real AprilTag library is present, the project includes a small fallback shim
that proxies detection to `js-aruco2` so the worker API stays functional. To replace
with a real AprilTag build, add a compatible `apriltag.js` (for example from
`apriltag-js`) that exposes `apriltag.detect(gray, width, height)` or sets
`self.AprilTag` / `self.apriltag`.

Sources / builds you can use:
- apriltag-js (search npm / unpkg / jsdelivr for prebuilt bundles)

Notes:
- The fallback is *not* a real AprilTag implementation — it keeps behavior
  compatible so the worker and UI do not fail if AprilTag is unavailable.
- If you want, I can fetch and add a canonical AprilTag JS+WASM build into this
  folder for you.

Quick install (automatic): run `npm run install:apriltag` from the project root —
this will try a few known prebuilt bundles and place `apriltag_wasm.js` +
`apriltag_wasm.wasm` into `vendor/apriltag/`.

Manual install: place `apriltag_wasm.js` and `apriltag_wasm.wasm` (or a
compatible `apriltag.js`) into this folder. The worker will automatically
load the local build when `Use AprilTag` is enabled in the UI.
