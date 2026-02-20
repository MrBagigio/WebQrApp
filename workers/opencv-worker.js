// opencv-worker.js
// Worker that loads OpenCV.js (WASM) and performs ArUco detection off the main thread.
let cvLoaded = false;
let dictionary = null;
let detectorParams = null;
let markerLength = 0.05; // meters - default

self.onmessage = async (e) => {
  const msg = e.data;
  try {
    if (msg.type === 'init') {
      postMessage({ type: 'log', message: 'Inizio inizializzazione worker...' });
      if (!cvLoaded) {
        // Advanced mocks for stubborn builds
        self.window = self;
        const mockElt = () => ({
          style: {},
          appendChild: () => { },
          removeChild: () => { },
          setAttribute: () => { },
          getAttribute: () => '',
          getElementsByTagName: () => []
        });
        const headElt = mockElt();
        self.document = {
          createElement: mockElt,
          getElementsByTagName: (name) => (name === 'head' ? [headElt] : [mockElt()]),
          head: headElt,
          body: mockElt(),
          readyState: 'complete',
          documentElement: { style: {} }
        };
        self.HTMLVideoElement = function () { };
        self.HTMLCanvasElement = function () { };
        self.Image = function () { };
        self.navigator = { userAgent: 'Node.js' }; // Trick some builds

        // Module setup for WASM
        self.Module = {
          onRuntimeInitialized: function () {
            postMessage({ type: 'log', message: 'Runtime OpenCV inizializzato!' });
          }
        };
        if (msg.wasmPath) {
          self.Module.locateFile = (path) => path.endsWith('.wasm') ? msg.wasmPath : path;
        }

        const urls = [
          msg.localOpencvUrl,
          'https://cdn.jsdelivr.net/npm/@techstardm/opencv-js@4.7.0-dev.20221107/opencv.js', // ArUco build
          'https://docs.opencv.org/4.5.1/opencv.js'
        ].filter(Boolean);

        let success = false;
        for (const url of urls) {
          try {
            postMessage({ type: 'log', message: 'Caricamento: ' + url });
            importScripts(url);
            await waitForCv(40000); // 40s timeout for large files
            if (cv.aruco) {
              success = true;
              break;
            } else {
              postMessage({ type: 'log', message: 'OpenCV caricato ma senza ArUco, provo altro...' });
            }
          } catch (e) {
            postMessage({ type: 'log', message: 'Errore: ' + e.message });
          }
        }

        if (!success) {
          postMessage({ type: 'error', error: 'Impossibile caricare OpenCV da nessuna fonte.' });
          return;
        }
        cvLoaded = true;
      }

      markerLength = msg.markerLength || markerLength;

      try {
        if (cv.aruco) {
          // Robust constructor check with fallbacks
          if (cv.aruco_DetectorParameters) {
            detectorParams = new cv.aruco_DetectorParameters();
          } else if (cv.aruco.DetectorParameters) {
            detectorParams = new cv.aruco.DetectorParameters();
          } else if (cv.DetectorParameters) {
            detectorParams = new cv.DetectorParameters();
          } else {
            postMessage({ type: 'log', message: 'Uso parametri default (non trovo DetectorParameters)' });
            detectorParams = null; // Some builds don't expose it, detectMarkers might take null
          }

          if (cv.aruco.getPredefinedDictionary) {
            if (typeof cv.aruco.DICT_ARUCO_ORIGINAL !== 'undefined') {
              dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL);
              postMessage({ type: 'log', message: 'Pronto per la scansione (DICT_ARUCO_ORIGINAL).' });
            } else if (typeof cv.aruco.DICT_5X5_1000 !== 'undefined') {
              dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000);
              postMessage({ type: 'log', message: 'Pronto per la scansione (DICT_5X5_1000 fallback).' });
            } else {
              throw new Error('Nessun dizionario ARUCO compatibile disponibile.');
            }
          } else {
            throw new Error('getPredefinedDictionary non trovato.');
          }
        } else {
          throw new Error('API cv.aruco non trovata.');
        }
      } catch (err) {
        postMessage({ type: 'error', error: 'ArUco setup error: ' + err.message });
        return;
      }
      postMessage({ type: 'ready' });
    } else if (msg.type === 'config') {
      if (typeof msg.markerLength === 'number') markerLength = msg.markerLength;
      postMessage({ type: 'log', message: 'Worker config updated: markerLength=' + markerLength });
    } else if (msg.type === 'frame') {
      if (!cvLoaded || !dictionary) return;
      const imageBitmap = msg.bitmap;
      if (!imageBitmap) return;

      // Draw ImageBitmap to OffscreenCanvas
      const canvas = new OffscreenCanvas(imageBitmap.width, imageBitmap.height);
      const ctx = canvas.getContext('2d');
      ctx.drawImage(imageBitmap, 0, 0);
      // extract ImageData
      const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);

      const src = new cv.Mat(canvas.height, canvas.width, cv.CV_8UC4);
      src.data.set(imageData.data);

      let corners = new cv.MatVector();
      let ids = new cv.Mat();
      let rejected = new cv.MatVector();

      try {
        // Some builds use 5 params, some use 6
        if (detectorParams) {
          cv.aruco.detectMarkers(src, dictionary, corners, ids, detectorParams, rejected);
        } else {
          cv.aruco.detectMarkers(src, dictionary, corners, ids);
        }
      } catch (err) {
        // detection failed
        corners.delete(); ids.delete(); rejected.delete(); src.delete();
        return;
      }

      const markers = [];
      const rejectedItems = [];

      if (!ids.empty()) {
        for (let i = 0; i < corners.size(); i++) {
          const c = corners.get(i);
          const pts = [];
          for (let j = 0; j < 4; j++) pts.push([c.data32F[j * 2], c.data32F[j * 2 + 1]]);
          markers.push({ id: ids.data32S ? ids.data32S[i] : (ids.data64F ? ids.data64F[i] : -1), corners: pts });
        }

        // estimate pose for all markers if function available
        try {
          // Convert camera arrays to cv.Mat if present
          let cameraMatrix = null; let distCoeffs = null;
          if (msg.cameraMatrix && Array.isArray(msg.cameraMatrix)) {
            cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, msg.cameraMatrix);
          }
          if (msg.distCoeffs && Array.isArray(msg.distCoeffs)) {
            distCoeffs = cv.matFromArray(1, msg.distCoeffs.length, cv.CV_64F, msg.distCoeffs);
          } else {
            distCoeffs = cv.Mat.zeros(1, 5, cv.CV_64F);
          }

          let rvecs = new cv.Mat();
          let tvecs = new cv.Mat();
          // allow per-frame markerLength override (engine may downscale frames)
          const usedMarkerLength = (typeof msg.markerLength === 'number') ? msg.markerLength : markerLength;
          cv.aruco.estimatePoseSingleMarkers(corners, usedMarkerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
          for (let i = 0; i < rvecs.rows; i++) {
            markers[i].rvec = [rvecs.data64F[i * 3 + 0], rvecs.data64F[i * 3 + 1], rvecs.data64F[i * 3 + 2]];
            markers[i].tvec = [tvecs.data64F[i * 3 + 0], tvecs.data64F[i * 3 + 1], tvecs.data64F[i * 3 + 2]];
          }
          rvecs.delete(); tvecs.delete();
          if (cameraMatrix) cameraMatrix.delete();
          if (distCoeffs) distCoeffs.delete();
        } catch (err) {
          // ignore pose failure
        }
      }

      // Add some rejected candidates for visual feedback in debug
      if (rejected.size() > 0) {
        const numToReturn = Math.min(rejected.size(), 5);
        for (let i = 0; i < numToReturn; i++) {
          const c = rejected.get(i);
          const pts = [];
          for (let j = 0; j < 4; j++) pts.push([c.data32F[j * 2], c.data32F[j * 2 + 1]]);
          rejectedItems.push({ corners: pts });
        }
      }

      // cleanup
      corners.delete(); ids.delete(); rejected.delete(); src.delete();

      postMessage({ type: 'result', markers, rejected: rejectedItems, timestamp: Date.now() });
    }
  } catch (err) {
    postMessage({ type: 'error', error: (err && err.message) || String(err) });
  }
};

function waitForCv(timeoutMs = 15000) {
  return new Promise((resolve, reject) => {
    const max = Date.now() + timeoutMs;
    (function check() {
      if (typeof cv !== 'undefined' && cv && cv.Mat) return resolve();
      if (Date.now() > max) return reject(new Error('OpenCV non disponibile nel worker (timeout ' + timeoutMs + 'ms)'));
      setTimeout(check, 100);
    })();
  });
}
