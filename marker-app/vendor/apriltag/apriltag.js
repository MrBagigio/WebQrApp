/*
  apriltag.js (loader + fallback shim)

  - Tries to load a real AprilTag JS/WASM build from known CDNs (if available).
  - If not found, provides a *fallback shim* that reuses js-aruco2 as a best-effort detector
    and exposes a minimal API compatible with the worker's expectations:
      - detect(grayUint8, width, height) -> [ { id, corners: [{x,y}, ...] }, ... ]

  Place a real apriltag.js / apriltag.wasm in this folder to use the native detector.
*/
(function () {
  // Prefer local prebuilt (apriltag_wasm.js / apriltag.js) under this folder if present
  const localCandidates = [
    'apriltag_wasm.js',
    'apriltag.js'
  ];

  let loaded = false;

  // try local files first (vendor/apriltag/apriltag_wasm.js or apriltag.js)
  for (const local of localCandidates) {
    try {
      importScripts(local);
      if (typeof self.apriltag !== 'undefined' || typeof self.AprilTag !== 'undefined' || typeof self.Apriltag !== 'undefined') {
        loaded = true;
        break;
      }
    } catch (e) {
      // ignore and try next
    }
  }

  // if no local build, try popular CDN bundles as a fallback
  if (!loaded) {
    // list of CDNs/paths to try (best-effort)
    const candidates = [
      'https://cdn.jsdelivr.net/npm/apriltag-js@0.0.6/dist/apriltag.min.js',
      'https://unpkg.com/apriltag-js@0.0.6/dist/apriltag.min.js'
    ];

    for (const url of candidates) {
      try {
        importScripts(url);
        // if the CDN script defines a global apriltag/AprilTag, consider it loaded
        if (typeof self.apriltag !== 'undefined' || typeof self.AprilTag !== 'undefined' || typeof self.Apriltag !== 'undefined') {
          loaded = true;
          break;
        }
      } catch (e) {
        // ignore and try next
      }
    }
  }

  if (!loaded) {
    // Provide a lightweight fallback that uses js-aruco2 for detection (NOT real AprilTag)
    // This keeps the worker path functional when no AprilTag implementation is available.
    self.apriltag = {
      detect: function (gray, width, height) {
        // Build an RGBA buffer required by js-aruco2.detect
        const rgba = new Uint8ClampedArray(width * height * 4);
        for (let i = 0, j = 0; i < gray.length; i++, j += 4) {
          const v = gray[i];
          rgba[j] = rgba[j + 1] = rgba[j + 2] = v;
          rgba[j + 3] = 255;
        }

        try {
          // AR.Detector is provided by vendor/js-aruco2.js (already loaded in the worker)
          const det = new AR.Detector({ dictionaryName: 'ARUCO_4X4_1000' });
          const found = det.detect({ width: width, height: height, data: rgba });
          // Normalize format to expected AprilTag-like output
          return found.map(m => ({ id: m.id, corners: m.corners.map(c => ({ x: c.x, y: c.y })) }));
        } catch (err) {
          return [];
        }
      }
    };

    // expose a convenience alias
    self.AprilTag = self.apriltag;
  }
})();
