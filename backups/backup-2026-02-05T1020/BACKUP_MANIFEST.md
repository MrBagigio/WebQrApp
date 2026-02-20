Backup Manifest - ExpeAR Pro (Zenith V14)
Timestamp: 2026-02-05T10:20

Files backed up in this folder:
- index.html.bak (full HTML, pre-POC state)
- script.js.bak (full JS before ArUco POC)
- styles.css.bak (full CSS)
- sw.js.bak (service worker)
- i18n.json.bak (i18n file backup)
- package.json.bak (package.json state)
- playwright.config.js.bak (Playwright config)
- tests_e2e.spec.js.bak (E2E tests)

Summary of changes made in the working copy (already applied before this backup):
- Removed duplicate `initP5` and p5 dependency; replaced start background with CSS
- Added robust loop control (start/stop), visibility handling and pagehide cleanup
- Improved AR event listener registration and added `destroyAR()` cleanup
- Hardened localStorage access and AR start error handling
- Added achievements, rank system, notifications and share button
- Added Playwright E2E tests, config and scripts, and SW registration

Next step: create AR solid-object POC (ArUco markers + OpenCV.js + Three.js) in the working tree.

NOTE: POC files created: `aruco-poc.html`, `aruco.js` (live detection + Three.js overlay), a printable generator `aruco-markers.html`, and a dev link added to `index.html`.

New features:
- Upload a `.glb`/`.gltf` model in the POC (via the "Carica modello glTF" control) to replace placeholder box.
- Toggle smoothing (exponential/Slerp) for pose stability.
- Toggle occlusion (depth-only occluder created from uploaded model) to mask AR model behind physical object when aligned.
- Printable markers generator (`aruco-markers.html`) to print ArUco IDs for testing.

If you need the entire project zipped, tell me and I will create an archive with the same timestamp.