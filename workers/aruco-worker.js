// ArUco detection worker using js-aruco2 (pure JavaScript — no OpenCV WASM)
// Detects markers from ARUCO dictionary (5x5) and estimates pose via POSIT/solvePnP.

importScripts('../vendor/js-aruco2.js');

let detector = null;
let detectors = null;
let detectorNames = [];
let detectorIndex = 0;
let markerLength = 0.1;  // meters (default 100mm)
let focalLength = 800;   // pixels (updated per frame)
let canvas = null;       // reusable OffscreenCanvas
let ctx = null;
const validMarkerIds = new Set([1]);

// Corner smoothing state (worker-side temporal filter to stabilise POSIT inputs)
// NOTE: Keep these values LOW — main-thread filters handle smoothing.
// Stacking too much smoothing causes visible lag.
let cornerSmoothing = 0; // 0..1 — disabled: let main-thread filters handle smoothing
const lastCornersById = {};
const lastCornersHistoryById = {}; // per-marker corner history for median filtering
const maxCornerHistory = 3;        // frames kept for median corner — REDUCED from 5

self.onmessage = (e) => {
    const msg = e.data;
    try {
        switch (msg.type) {
            case 'init': {
                // allow caller to specify dictionary (e.g. ARUCO, ARUCO_5X5_1000, ARUCO_4X4_1000, ARUCO_MIP_36h12, etc.)
                const dictNameRaw = msg.dictionaryName || 'ARUCO';
                const dictName = String(dictNameRaw);
                const dictMode = dictName.toUpperCase();
                detector = null;
                detectors = null;
                detectorNames = [];
                detectorIndex = 0;

                if (dictMode === 'AUTO') {
                    const candidates = ['ARUCO', 'ARUCO_4X4_50', 'ARUCO_5X5_1000', 'ARUCO_MIP_36H12'];
                    const built = [];
                    const names = [];
                    for (const name of candidates) {
                        try {
                            built.push(new AR.Detector({ dictionaryName: name }));
                            names.push(name);
                        } catch (_err) {
                            // ignore unsupported dictionaries in current build
                        }
                    }
                    if (built.length > 0) {
                        detectors = built;
                        detectorNames = names;
                    } else {
                        detector = new AR.Detector({ dictionaryName: 'ARUCO' });
                    }
                } else {
                    detector = new AR.Detector({ dictionaryName: dictName });
                }
                markerLength = msg.markerLength || markerLength;
                postMessage({ type: 'log', message: 'js-aruco2 pronto (' + dictName + ')' });
                postMessage({ type: 'ready' });
                break;
            }

            case 'config': {
                if (typeof msg.markerLength === 'number') markerLength = msg.markerLength;
                if (typeof msg.cornerSmoothing === 'number') cornerSmoothing = Math.max(0, Math.min(1, msg.cornerSmoothing));
                if (typeof msg.focalLength === 'number') focalLength = msg.focalLength;

                postMessage({ type: 'log', message: `worker config updated: markerLength=${markerLength}, cornerSmoothing=${cornerSmoothing.toFixed(2)}` });
                break;
            }

            case 'frame': {
                processFrame(msg);
                break;
            }
        }
    } catch (err) {
        postMessage({ type: 'error', error: (err && err.message) || String(err) });
    }
};

function processFrame(msg) {
    if (!detector && (!detectors || !detectors.length)) return;
    const bitmap = msg.bitmap;
    if (!bitmap) return;

    const w = bitmap.width;
    const h = bitmap.height;

    // Reuse OffscreenCanvas if same size, otherwise create new
    if (!canvas || canvas.width !== w || canvas.height !== h) {
        canvas = new OffscreenCanvas(w, h);
        ctx = canvas.getContext('2d', { willReadFrequently: true });
    }
    ctx.drawImage(bitmap, 0, 0);
    const imageData = ctx.getImageData(0, 0, w, h);

    // Scale factor: detection space → overlay space
    const scaleX = msg.overlayWidth ? (msg.overlayWidth / w) : 1;
    const scaleY = msg.overlayHeight ? (msg.overlayHeight / h) : 1;

    // Focal length from camera matrix
    if (msg.cameraMatrix && msg.cameraMatrix.length >= 5) {
        focalLength = msg.cameraMatrix[0]; // fx
    }

    let arDetected = [];

    // run fast ArUco detection
    try {
        if (detectors && detectors.length) {
            for (let i = 0; i < detectors.length; i++) {
                const idx = (detectorIndex + i) % detectors.length;
                const d = detectors[idx];
                const found = d.detect({ width: w, height: h, data: imageData.data }) || [];
                if (found.length > 0) {
                    arDetected = found;
                    detectorIndex = idx;
                    break;
                }
            }
        } else {
            arDetected = detector.detect({ width: w, height: h, data: imageData.data }) || [];
        }
    } catch (e) {
        arDetected = [];
    }

    const detected = [];
    for (const t of arDetected) {
        const id = Number(t.id);
        if (!Number.isFinite(id) || !validMarkerIds.has(id)) continue;
        const corners = (t.corners || []).map(c => ({ x: (c.x ?? c[0]), y: (c.y ?? c[1]) }));
        if (!corners || corners.length < 4) continue;
        detected.push({ id, corners, source: 'aruco' });
    }

    const markers = [];
    const cx = w / 2;
    const cy = h / 2;
    const usedLength = (typeof msg.markerLength === 'number') ? msg.markerLength : markerLength;

    for (const m of detected) {
        // if (!validMarkerIds.has(Number(m.id))) continue; // Removed filtering to allow main thread to decide
        if (!m.corners || m.corners.length < 4) continue;

        // Optionally smooth corners per-marker (temporal smoothing inside worker)
        if (cornerSmoothing > 0 && lastCornersById[m.id] && lastCornersById[m.id].length === m.corners.length) {
            for (let i = 0; i < m.corners.length; i++) {
                const prev = lastCornersById[m.id][i];
                m.corners[i].x = prev.x * (1 - cornerSmoothing) + m.corners[i].x * cornerSmoothing;
                m.corners[i].y = prev.y * (1 - cornerSmoothing) + m.corners[i].y * cornerSmoothing;
            }
        }

        // store copy for next frame (temporal smoothing reference)
        lastCornersById[m.id] = m.corners.map(c => ({ x: c.x, y: c.y }));

        // Median corner filtering: accumulate per-marker corner history and take per-axis median
        // This reduces single-frame jitter from detection noise
        if (!lastCornersHistoryById[m.id]) lastCornersHistoryById[m.id] = [];
        lastCornersHistoryById[m.id].push(m.corners.map(c => ({ x: c.x, y: c.y })));
        if (lastCornersHistoryById[m.id].length > maxCornerHistory) lastCornersHistoryById[m.id].shift();

        if (lastCornersHistoryById[m.id].length >= 3) {
            const hist = lastCornersHistoryById[m.id];
            for (let i = 0; i < m.corners.length; i++) {
                const xs = hist.map(h => h[i] ? h[i].x : m.corners[i].x).slice().sort((a, b) => a - b);
                const ys = hist.map(h => h[i] ? h[i].y : m.corners[i].y).slice().sort((a, b) => a - b);
                const mid = Math.floor(xs.length / 2);
                m.corners[i].x = xs[mid];
                m.corners[i].y = ys[mid];
            }
        }

        // Normalize corner order (top-left, top-right, bottom-right, bottom-left)
        // before pose estimation. Inconsistent order causes center/orientation drift.
        m.corners = normalizeMarkerCorners(m.corners);

        // Corners in overlay (full-resolution) space for drawing
        const corners = m.corners.map(c => [c.x * scaleX, c.y * scaleY]);
        const result = { id: m.id, corners };

        // Pose estimation: POSIT
        try {
            const centeredCorners = m.corners.map(c => ({ x: c.x - cx, y: -(c.y - cy) }));
            const positInst = new POS.Posit(usedLength, focalLength);
            const pose = positInst.pose(centeredCorners);
            if (pose && pose.bestRotation && pose.bestTranslation) {
                result.rvec = rotMatToRvec(pose.bestRotation);
                result.tvec = [pose.bestTranslation[0], pose.bestTranslation[1], pose.bestTranslation[2]];
                result.cameraAngleDeg = viewAngleDegFromRotationMatrix(pose.bestRotation);
                result.poseError = pose.bestError;
                result.source = 'posit';
                result.confidence = computePoseConfidence({
                    source: 'posit',
                    poseError: result.poseError,
                    corners: m.corners,
                    cameraAngleDeg: result.cameraAngleDeg
                });
            }
        } catch (e) {
            postMessage({ type: 'log', message: 'pose estimation error: ' + (e && e.message) });
        }

        markers.push(result);
    }

    // Clean stale corner history for markers not seen this frame (prevent unbounded memory growth)
    const seenIds = new Set(detected.map(d => d.id));
    for (const id in lastCornersById) {
        if (!seenIds.has(Number(id))) {
            delete lastCornersById[id];
            delete lastCornersHistoryById[id];
        }
    }

    postMessage({ type: 'result', markers, rejected: [], timestamp: Date.now() });
}

/**
 * Convert a 3×3 rotation matrix (array of 3 rows, each 3 elements) to a
 * Rodrigues rotation vector [rx, ry, rz] where |v| = angle.
 */
function rotMatToRvec(R) {
    const trace = R[0][0] + R[1][1] + R[2][2];
    const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
    const theta = Math.acos(cosTheta);

    if (theta < 1e-6) return [0, 0, 0];

    const sinTheta = Math.sin(theta);

    if (Math.abs(sinTheta) < 1e-6) {
        // theta ≈ π — extract axis from (R + I)
        const Rp = [
            [R[0][0] + 1, R[0][1],     R[0][2]    ],
            [R[1][0],     R[1][1] + 1,  R[1][2]    ],
            [R[2][0],     R[2][1],      R[2][2] + 1]
        ];
        let best = 0, bestNorm = 0;
        for (let i = 0; i < 3; i++) {
            const n = Rp[0][i] ** 2 + Rp[1][i] ** 2 + Rp[2][i] ** 2;
            if (n > bestNorm) { bestNorm = n; best = i; }
        }
        const len = Math.sqrt(bestNorm);
        if (len < 1e-10) return [theta, 0, 0];
        return [
            theta * Rp[0][best] / len,
            theta * Rp[1][best] / len,
            theta * Rp[2][best] / len
        ];
    }

    const k = theta / (2 * sinTheta);
    return [
        k * (R[2][1] - R[1][2]),
        k * (R[0][2] - R[2][0]),
        k * (R[1][0] - R[0][1])
    ];
}

function markerPerimeterPx(corners) {
    if (!Array.isArray(corners) || corners.length < 4) return 0;
    let p = 0;
    for (let i = 0; i < corners.length; i++) {
        const a = corners[i];
        const b = corners[(i + 1) % corners.length];
        const dx = (a.x - b.x);
        const dy = (a.y - b.y);
        p += Math.hypot(dx, dy);
    }
    return p;
}

function normalizeMarkerCorners(corners) {
    if (!Array.isArray(corners) || corners.length < 4) return corners;

    const pts = corners.map(c => ({ x: Number(c.x), y: Number(c.y) }));

    // 1) Sort by angle around centroid (stable circular order)
    const cx = pts.reduce((s, p) => s + p.x, 0) / pts.length;
    const cy = pts.reduce((s, p) => s + p.y, 0) / pts.length;
    pts.sort((a, b) => Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx));

    // 2) Ensure clockwise winding in image coordinates (y down)
    const signedArea = polygonSignedArea(pts);
    if (signedArea < 0) pts.reverse();

    // 3) Rotate list so first point is top-left (min x+y)
    let start = 0;
    let best = Number.POSITIVE_INFINITY;
    for (let i = 0; i < pts.length; i++) {
        const score = pts[i].x + pts[i].y;
        if (score < best) {
            best = score;
            start = i;
        }
    }

    return [
        pts[start],
        pts[(start + 1) % pts.length],
        pts[(start + 2) % pts.length],
        pts[(start + 3) % pts.length]
    ];
}

function polygonSignedArea(pts) {
    let area = 0;
    for (let i = 0; i < pts.length; i++) {
        const a = pts[i];
        const b = pts[(i + 1) % pts.length];
        area += (a.x * b.y) - (b.x * a.y);
    }
    return area * 0.5;
}

function clamp01(v) { return Math.max(0, Math.min(1, v)); }

function computePoseConfidence({ source, poseError, corners, cameraAngleDeg }) {
    const perim = markerPerimeterPx(corners);
    const perimN = clamp01((perim - 80) / 240);
    const viewDeg = Number.isFinite(cameraAngleDeg) ? cameraAngleDeg : 0;
    const obliqueN = clamp01((viewDeg - 20) / 60); // 20°..80° maps to 0..1
    const angleW = Math.max(0.08, 1 - obliqueN * obliqueN);

    // POSIT error has different scale; use softer mapping.
    const errN = Math.min(3.5, Math.max(0, poseError / 1.2));
    const base = Math.exp(-errN * 0.95);
    return Math.max(0.03, Math.min(0.92, (0.08 + 0.60 * base + 0.32 * perimN) * angleW));
}

function clampNegPos1(v) { return Math.max(-1, Math.min(1, v)); }

function viewAngleDegFromRotationMatrix(R) {
    // Marker normal in camera frame is R*[0,0,1] => 3rd column.
    const nz = R[2][2];
    const frontal = Math.abs(clampNegPos1(nz));
    return Math.acos(frontal) * 180 / Math.PI;
}
