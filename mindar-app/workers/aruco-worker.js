// ArUco detection worker using js-aruco2 (pure JavaScript — no OpenCV WASM)
// Detects markers from ARUCO dictionary (5x5) and estimates pose via POSIT/solvePnP.

importScripts('../vendor/js-aruco2.js');

let detector = null;
let markerLength = 0.1;  // meters (default 100mm)
let focalLength = 800;   // pixels (updated per frame)
let canvas = null;       // reusable OffscreenCanvas
let ctx = null;
const validMarkerIds = new Set([1, 2, 3, 4, 5, 6, 7, 8]);

// Corner smoothing state (worker-side temporal filter to stabilise POSIT inputs)
// NOTE: Keep these values LOW — main-thread filters handle smoothing.
// Stacking too much smoothing causes visible lag.
let cornerSmoothing = 0; // 0..1 — disabled: let main-thread filters handle smoothing
const lastCornersById = {};
const lastCornersHistoryById = {}; // per-marker corner history for median filtering
const maxCornerHistory = 3;        // frames kept for median corner — REDUCED from 5

// Optical-flow / template-tracking (lightweight stabilizer for corners)
let cornerFlowEnabled = false; // disabled: adds latency for marginal benefit
let cornerFlowRadius = 4;
let cornerFlowTemplate = 9;
let cornerFlowMaxNormalizedSSD = 40; // threshold — REDUCED from 60
let lastImageGray = null;        // Uint8Array of previous frame grayscale

// Optional OpenCV.js sub-pixel refinement and advanced CV ops
let useSubpixel = false;
let subpixWin = 5;           // half-window used by cornerSubPix
let subpixMaxIter = 30;
let subpixEPS = 0.1;
let cvReady = false;
let cvLoading = false;
// Optional higher-precision pose via solvePnP and LK optical-flow
let useSolvePnP = false;     // when true and cvReady -> prefer cv.solvePnP for pose
let usePyrLKFlow = false;    // when true and cvReady -> use calcOpticalFlowPyrLK for corner tracking
let _prevGrayMat = null;     // cached cv.Mat of previous gray frame for pyrLK
let _prevPtsMat = null;      // cached previous points Mat for pyrLK tracking


// AprilTag fallback support (scaffold: will be active if vendor/apriltag is provided)
let useAprilTag = false;
let aprilReady = false;
let aprilImpl = null; // holds detected implementation object if present

self.onmessage = (e) => {
    const msg = e.data;
    try {
        switch (msg.type) {
            case 'init':
                // allow caller to specify dictionary (e.g. ARUCO, ARUCO_5X5_1000, ARUCO_4X4_1000, ARUCO_MIP_36h12, etc.)
                const dictName = msg.dictionaryName || 'ARUCO';
                detector = new AR.Detector({ dictionaryName: dictName });
                markerLength = msg.markerLength || markerLength;
                postMessage({ type: 'log', message: 'js-aruco2 pronto (' + dictName + ')' });
                postMessage({ type: 'ready' });
                break;

            case 'config':
                if (typeof msg.markerLength === 'number') markerLength = msg.markerLength;
                if (typeof msg.cornerSmoothing === 'number') cornerSmoothing = Math.max(0, Math.min(1, msg.cornerSmoothing));
                if (typeof msg.focalLength === 'number') focalLength = msg.focalLength;
                if (typeof msg.cornerFlowEnabled === 'boolean') cornerFlowEnabled = !!msg.cornerFlowEnabled;
                if (typeof msg.cornerFlowRadius === 'number') cornerFlowRadius = Math.max(1, Math.min(32, Math.floor(msg.cornerFlowRadius)));
                if (typeof msg.cornerFlowTemplate === 'number') cornerFlowTemplate = Math.max(3, Math.min(33, Math.floor(msg.cornerFlowTemplate)));
                if (typeof msg.cornerFlowMaxNormalizedSSD === 'number') cornerFlowMaxNormalizedSSD = Math.max(1, Number(msg.cornerFlowMaxNormalizedSSD));

                // OpenCV sub-pixel config
                if (typeof msg.useSubpixel === 'boolean') {
                    useSubpixel = !!msg.useSubpixel;
                    if (useSubpixel && !cvReady && !cvLoading) {
                        try {
                            importScripts('../vendor/opencv/opencv.js');
                            cvLoading = true;
                            if (typeof cv !== 'undefined' && cv && cv.Mat) {
                                cvReady = true;
                                cvLoading = false;
                                postMessage({ type: 'log', message: 'OpenCV.js ready in worker' });
                            } else if (typeof cv !== 'undefined') {
                                cv['onRuntimeInitialized'] = () => { cvReady = true; cvLoading = false; postMessage({ type: 'log', message: 'OpenCV.js ready (async)' }); };
                            }
                        } catch (e) {
                            postMessage({ type: 'log', message: 'OpenCV.js load failed: ' + e.message });
                            useSubpixel = false;
                        }
                    }
                }
                if (typeof msg.subpixWin === 'number') subpixWin = Math.max(3, Math.min(31, Math.floor(msg.subpixWin)));
                if (typeof msg.subpixMaxIter === 'number') subpixMaxIter = Math.max(1, Math.min(200, Math.floor(msg.subpixMaxIter)));
                if (typeof msg.subpixEPS === 'number') subpixEPS = Math.max(0.0, Math.min(10.0, Number(msg.subpixEPS)));

                // advanced CV toggles
                if (typeof msg.useSolvePnP === 'boolean') useSolvePnP = !!msg.useSolvePnP;
                if (typeof msg.usePyrLKFlow === 'boolean') usePyrLKFlow = !!msg.usePyrLKFlow;
                if (typeof msg.cornerFlowMaxNormalizedSSD === 'number') cornerFlowMaxNormalizedSSD = Math.max(1, Number(msg.cornerFlowMaxNormalizedSSD));

                // AprilTag config: try to load vendor implementation when requested
                if (typeof msg.useAprilTag === 'boolean') {
                    useAprilTag = !!msg.useAprilTag;
                    if (useAprilTag && !aprilReady) {
                        try {
                            importScripts('../vendor/apriltag/apriltag.js');
                            // attempt to detect common exposed symbols
                            if (typeof self.apriltag !== 'undefined') { aprilImpl = self.apriltag; aprilReady = true; }
                            else if (typeof self.AprilTag !== 'undefined') { aprilImpl = self.AprilTag; aprilReady = true; }
                            else if (typeof self.Apriltag !== 'undefined') { aprilImpl = self.Apriltag; aprilReady = true; }
                            if (aprilReady) postMessage({ type: 'log', message: 'AprilTag library loaded in worker' });
                            else postMessage({ type: 'log', message: 'AprilTag library not found in vendor — will fallback to ArUco' });
                        } catch (err) {
                            postMessage({ type: 'log', message: 'AprilTag load failed: ' + (err && err.message) });
                            useAprilTag = false;
                            aprilReady = false;
                        }
                    }
                }

                postMessage({ type: 'log', message: `worker config updated: markerLength=${markerLength}, cornerSmoothing=${cornerSmoothing.toFixed(2)}, cornerFlow=${cornerFlowEnabled}, subpix=${useSubpixel}, april=${useAprilTag && aprilReady}` });
                break;

            case 'frame':
                processFrame(msg);
                break;
        }
    } catch (err) {
        postMessage({ type: 'error', error: (err && err.message) || String(err) });
    }
};

function processFrame(msg) {
    if (!detector) return;
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

    // convert current frame to grayscale for template tracking
    const gray = new Uint8ClampedArray(w * h);
    const src = imageData.data;
    for (let i = 0, j = 0; i < src.length; i += 4, j++) {
        // luminosity
        gray[j] = (0.2126 * src[i] + 0.7152 * src[i+1] + 0.0722 * src[i+2]) | 0;
    }

    // Scale factor: detection space → overlay space
    const scaleX = msg.overlayWidth ? (msg.overlayWidth / w) : 1;
    const scaleY = msg.overlayHeight ? (msg.overlayHeight / h) : 1;

    // Focal length from camera matrix
    if (msg.cameraMatrix && msg.cameraMatrix.length >= 5) {
        focalLength = msg.cameraMatrix[0]; // fx
    }

    // Run both detectors (if available) and merge by ID — prefer AprilTag corners when present
    let arDetected = [];
    let aprDetected = [];

    // try AprilTag first (best-effort)
    if (useAprilTag && aprilReady) {
        try {
            if (typeof aprilImpl.detect === 'function') {
                const res = aprilImpl.detect(gray, w, h);
                if (Array.isArray(res) && res.length) aprDetected = res.map(t => ({ id: Number(t.id), corners: (t.corners || t.corners2d || []).map(c => ({ x: c.x || c[0], y: c.y || c[1] })) }));
            } else if (typeof aprilImpl.detectTags === 'function') {
                const res = aprilImpl.detectTags(gray, w, h);
                if (Array.isArray(res) && res.length) aprDetected = res.map(t => ({ id: Number(t.id), corners: t.corners.map(c => ({ x: c.x, y: c.y })) }));
            } else if (typeof self.apriltag_detect === 'function') {
                const res = self.apriltag_detect(gray, w, h);
                if (Array.isArray(res) && res.length) aprDetected = res.map(t => ({ id: Number(t.id), corners: t.corners.map(c => ({ x: c.x, y: c.y })) }));
            }
        } catch (err) {
            postMessage({ type: 'log', message: 'AprilTag detect error: ' + (err && err.message) });
            aprDetected = [];
        }
    }

    // always run fast ArUco detection (complimentary)
    try {
        arDetected = detector.detect({ width: w, height: h, data: imageData.data }) || [];
    } catch (e) {
        arDetected = [];
    }

    // merge detections by id (prefer apriltag corners when available)
    const merged = new Map();
    const addDet = (arr, src) => {
        for (const t of arr) {
            const id = Number(t.id);
            if (!Number.isFinite(id)) continue; // Removed validMarkerIds check to allow debug of other markers
            const corners = (t.corners || []).map(c => ({ x: c.x || c[0], y: c.y || c[1] }));
            if (!corners || corners.length < 4) continue;
            if (src === 'apriltag') {
                if (!merged.has(id)) merged.set(id, { id, corners, sources: new Set([src]) });
                else {
                    const e = merged.get(id);
                    e.sources.add(src);
                    e.corners = corners; // prefer AprilTag corners
                }
            } else {
                if (!merged.has(id)) merged.set(id, { id, corners, sources: new Set([src]) });
                else {
                    const e = merged.get(id);
                    e.sources.add(src);
                }
            }
        }
    };
    addDet(aprDetected, 'apriltag');
    addDet(arDetected, 'aruco');

    const detected = Array.from(merged.values()).map(v => ({ id: v.id, corners: v.corners, source: (v.sources.has('apriltag') && !v.sources.has('aruco')) ? 'apriltag' : (v.sources.has('aruco') && !v.sources.has('apriltag') ? 'aruco' : 'mixed') }));

    const markers = [];
    const cx = w / 2;
    const cy = h / 2;
    const usedLength = (typeof msg.markerLength === 'number') ? msg.markerLength : markerLength;

    for (const m of detected) {
        // if (!validMarkerIds.has(Number(m.id))) continue; // Removed filtering to allow main thread to decide
        if (!m.corners || m.corners.length < 4) continue;

        // Optional: refine corners by template-based optical flow using previous frame
        if (cornerFlowEnabled && lastImageGray && lastCornersById[m.id] && lastCornersById[m.id].length === m.corners.length) {
            // Prefer OpenCV LK flow when available and enabled — it is faster and sub-pixel by construction
            if (cvReady && usePyrLKFlow) {
                let prevMat = null, currMat = null, prevPts = null, nextPts = null, statusM = null, errM = null;
                try {
                    prevMat = new cv.Mat(h, w, cv.CV_8UC1);
                    prevMat.data.set(lastImageGray);
                    currMat = new cv.Mat(h, w, cv.CV_8UC1);
                    currMat.data.set(gray);

                    prevPts = new cv.Mat(m.corners.length, 1, cv.CV_32FC2);
                    for (let i = 0; i < m.corners.length; i++) { prevPts.data32F[i*2] = lastCornersById[m.id][i].x; prevPts.data32F[i*2+1] = lastCornersById[m.id][i].y; }
                    nextPts = new cv.Mat();
                    statusM = new cv.Mat();
                    errM = new cv.Mat();

                    cv.calcOpticalFlowPyrLK(prevMat, currMat, prevPts, nextPts, statusM, errM, new cv.Size(21,21), 3);

                    for (let i = 0; i < m.corners.length; i++) {
                        if (statusM.data[i] === 1) {
                            const nx = nextPts.data32F[i*2];
                            const ny = nextPts.data32F[i*2+1];
                            const weight = 0.85;
                            m.corners[i].x = m.corners[i].x * (1 - weight) + nx * weight;
                            m.corners[i].y = m.corners[i].y * (1 - weight) + ny * weight;
                        }
                    }
                } catch (err) {
                    postMessage({ type: 'log', message: 'PyrLK flow failed: ' + (err && err.message) });
                } finally {
                    // Always release Mats to prevent memory leaks
                    try { if (prevMat) prevMat.delete(); } catch (_) {}
                    try { if (currMat) currMat.delete(); } catch (_) {}
                    try { if (prevPts) prevPts.delete(); } catch (_) {}
                    try { if (nextPts) nextPts.delete(); } catch (_) {}
                    try { if (statusM) statusM.delete(); } catch (_) {}
                    try { if (errM) errM.delete(); } catch (_) {}
                }
            } else {
                const halfT = Math.floor(Math.max(3, Math.min(21, cornerFlowTemplate)) / 2);
                const R = Math.max(1, Math.min(24, cornerFlowRadius));

                for (let i = 0; i < m.corners.length; i++) {
                    const prev = lastCornersById[m.id][i];
                    const tx = Math.round(prev.x);
                    const ty = Math.round(prev.y);

                    // ensure template inside previous image
                    if (tx - halfT < 0 || ty - halfT < 0 || tx + halfT >= w || ty + halfT >= h) continue;

                    // extract template from lastImageGray
                    let bestX = null, bestY = null, bestScore = Infinity;

                    for (let sy = -R; sy <= R; sy++) {
                        const cyc = ty + sy;
                        if (cyc - halfT < 0 || cyc + halfT >= h) continue;
                        for (let sx = -R; sx <= R; sx++) {
                            const cxc = tx + sx;
                            if (cxc - halfT < 0 || cxc + halfT >= w) continue;

                            // compute SSD between template (prev frame) and candidate (current gray)
                            let ssd = 0;
                            for (let yy = -halfT; yy <= halfT; yy++) {
                                const rowPrev = (ty + yy) * w;
                                const rowCurr = (cyc + yy) * w;
                                for (let xx = -halfT; xx <= halfT; xx++) {
                                    const tVal = lastImageGray[rowPrev + (tx + xx)];
                                    const cVal = gray[rowCurr + (cxc + xx)];
                                    const d = tVal - cVal;
                                    ssd += d * d;
                                }
                                if (ssd > bestScore) break; // early exit
                            }

                            if (ssd < bestScore) { bestScore = ssd; bestX = cxc; bestY = cyc; }
                        }
                    }

                    // normalize SSD by template area and require a reasonable threshold
                    const templateArea = (2 * halfT + 1) * (2 * halfT + 1);
                    if (bestX !== null && bestY !== null) {
                        const norm = bestScore / Math.max(1, templateArea);
                        if (norm < cornerFlowMaxNormalizedSSD) {
                            const weight = 0.85; // prefer matched location but keep detector stable
                            m.corners[i].x = m.corners[i].x * (1 - weight) + bestX * weight;
                            m.corners[i].y = m.corners[i].y * (1 - weight) + bestY * weight;
                        }
                    }
                }
            }
        }

        // Optionally smooth corners per-marker (temporal smoothing inside worker)
        if (cornerSmoothing > 0 && lastCornersById[m.id] && lastCornersById[m.id].length === m.corners.length) {
            for (let i = 0; i < m.corners.length; i++) {
                const prev = lastCornersById[m.id][i];
                m.corners[i].x = prev.x * (1 - cornerSmoothing) + m.corners[i].x * cornerSmoothing;
                m.corners[i].y = prev.y * (1 - cornerSmoothing) + m.corners[i].y * cornerSmoothing;
            }
        }

        // Optional: sub-pixel refinement using OpenCV.js (if loaded and enabled)
        if (useSubpixel && cvReady) {
            let srcMat = null, pts = null;
            try {
                srcMat = new cv.Mat(h, w, cv.CV_8UC1);
                srcMat.data.set(gray);
                pts = new cv.Mat(m.corners.length, 1, cv.CV_32FC2);
                for (let i = 0; i < m.corners.length; i++) {
                    pts.data32F[i * 2] = m.corners[i].x;
                    pts.data32F[i * 2 + 1] = m.corners[i].y;
                }
                const win = new cv.Size(subpixWin, subpixWin);
                const zero = new cv.Size(-1, -1);
                const criteria = new cv.TermCriteria(cv.TermCriteria_EPS + cv.TermCriteria_MAX_ITER, subpixMaxIter, subpixEPS);
                cv.cornerSubPix(srcMat, pts, win, zero, criteria);
                for (let i = 0; i < m.corners.length; i++) {
                    m.corners[i].x = pts.data32F[i * 2];
                    m.corners[i].y = pts.data32F[i * 2 + 1];
                }
            } catch (err) {
                postMessage({ type: 'log', message: 'subpixel refinement failed: ' + (err && err.message) });
            } finally {
                try { if (pts) pts.delete(); } catch (_) {}
                try { if (srcMat) srcMat.delete(); } catch (_) {}
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

        // Corners in overlay (full-resolution) space for drawing
        const corners = m.corners.map(c => [c.x * scaleX, c.y * scaleY]);
        const result = { id: m.id, corners };

        // Pose estimation: prefer OpenCV solvePnP when available (more robust than POSIT)
        try {
            let gotPose = false;

            if (cvReady && useSolvePnP && msg && msg.cameraMatrix && Array.isArray(msg.cameraMatrix) && msg.cameraMatrix.length >= 9) {
                let objPts = null, imgPts = null, camMat = null, dist = null, rvec = null, tvec = null, inliers = null, proj = null;
                try {
                    // prepare object points for a square marker (counter-clockwise, Z=0)
                    const half = usedLength / 2.0;
                    objPts = cv.matFromArray(4, 1, cv.CV_32FC3, [ -half,  half, 0,  half,  half, 0,  half, -half, 0,  -half, -half, 0 ]);
                    imgPts = cv.matFromArray(4, 1, cv.CV_32FC2, [ m.corners[0].x, m.corners[0].y, m.corners[1].x, m.corners[1].y, m.corners[2].x, m.corners[2].y, m.corners[3].x, m.corners[3].y ]);

                    camMat = cv.matFromArray(3, 3, cv.CV_64F, msg.cameraMatrix);
                    const distArr = (msg.distCoeffs && Array.isArray(msg.distCoeffs) && msg.distCoeffs.length > 0) ? msg.distCoeffs : [0,0,0,0,0];
                    dist = cv.matFromArray(distArr.length, 1, cv.CV_64F, distArr);

                    rvec = new cv.Mat();
                    tvec = new cv.Mat();
                    inliers = new cv.Mat();

                    // Use a planarity-aware method for square markers when available.
                    const pnpMethod = (typeof cv.SOLVEPNP_IPPE_SQUARE !== 'undefined')
                        ? cv.SOLVEPNP_IPPE_SQUARE
                        : cv.SOLVEPNP_ITERATIVE;
                    const ret = cv.solvePnPRansac(objPts, imgPts, camMat, dist, rvec, tvec, false, 100, 8.0, 0.99, inliers, pnpMethod);

                    if (ret) {
                        // Optional non-linear refinement pass (if supported by this OpenCV build)
                        try {
                            if (typeof cv.solvePnPRefineLM === 'function') {
                                cv.solvePnPRefineLM(objPts, imgPts, camMat, dist, rvec, tvec);
                            }
                        } catch (_) { /* refinement best-effort */ }

                        // compute reprojection error as poseError
                        proj = new cv.Mat();
                        cv.projectPoints(objPts, rvec, tvec, camMat, dist, proj);
                        let sumErr = 0;
                        for (let i = 0; i < 4; i++) {
                            const px = proj.data32F[i*2], py = proj.data32F[i*2+1];
                            const dx = px - m.corners[i].x, dy = py - m.corners[i].y;
                            sumErr += Math.hypot(dx, dy);
                        }
                        const rms = sumErr / 4.0;

                        result.rvec = [rvec.data64F[0], rvec.data64F[1], rvec.data64F[2]];
                        result.tvec = [tvec.data64F[0], tvec.data64F[1], tvec.data64F[2]];
                        result.cameraAngleDeg = viewAngleDegFromRvec(result.rvec);
                        result.poseError = rms;
                        result.source = result.source || 'opencv-pnp';
                        result.confidence = computePoseConfidence({
                            source: 'opencv-pnp',
                            poseError: rms,
                            corners: m.corners,
                            cameraAngleDeg: result.cameraAngleDeg
                        });
                        gotPose = true;
                    }
                } catch (err) {
                    postMessage({ type: 'log', message: 'solvePnP failed: ' + (err && err.message) });
                } finally {
                    try { if (objPts) objPts.delete(); } catch (_) {}
                    try { if (imgPts) imgPts.delete(); } catch (_) {}
                    try { if (camMat) camMat.delete(); } catch (_) {}
                    try { if (dist) dist.delete(); } catch (_) {}
                    try { if (rvec) rvec.delete(); } catch (_) {}
                    try { if (tvec) tvec.delete(); } catch (_) {}
                    try { if (inliers) inliers.delete(); } catch (_) {}
                    try { if (proj) proj.delete(); } catch (_) {}
                }
            }

            // fallback to POSIT if no solvePnP pose was obtained
            if (!gotPose) {
                const centeredCorners = m.corners.map(c => ({ x: c.x - cx, y: -(c.y - cy) }));
                const positInst = new POS.Posit(usedLength, focalLength);
                const pose = positInst.pose(centeredCorners);
                if (pose && pose.bestRotation && pose.bestTranslation) {
                    result.rvec = rotMatToRvec(pose.bestRotation);
                    result.tvec = [pose.bestTranslation[0], pose.bestTranslation[1], pose.bestTranslation[2]];
                    result.cameraAngleDeg = viewAngleDegFromRotationMatrix(pose.bestRotation);
                    result.poseError = pose.bestError;
                    result.source = result.source || 'posit';
                    result.confidence = result.confidence || computePoseConfidence({
                        source: 'posit',
                        poseError: result.poseError,
                        corners: m.corners,
                        cameraAngleDeg: result.cameraAngleDeg
                    });
                    gotPose = true;
                }
            }
        } catch (e) {
            postMessage({ type: 'log', message: 'pose estimation error: ' + (e && e.message) });
        }

        markers.push(result);
    }

    // save current gray for next-frame template tracking
    lastImageGray = gray;

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

function clamp01(v) { return Math.max(0, Math.min(1, v)); }

function computePoseConfidence({ source, poseError, corners, cameraAngleDeg }) {
    const perim = markerPerimeterPx(corners);
    const perimN = clamp01((perim - 80) / 240);
    const viewDeg = Number.isFinite(cameraAngleDeg) ? cameraAngleDeg : 0;
    const obliqueN = clamp01((viewDeg - 20) / 60); // 20°..80° maps to 0..1
    const angleW = Math.max(0.08, 1 - obliqueN * obliqueN);

    if (source === 'opencv-pnp') {
        // poseError is reprojection RMS in px
        const errN = Math.min(3, Math.max(0, poseError / 8.0));
        const base = Math.exp(-(errN * errN) * 0.9);
        return Math.max(0.04, Math.min(1.0, (0.10 + 0.62 * base + 0.28 * perimN) * angleW));
    }

    // POSIT error has different scale; use softer mapping.
    const errN = Math.min(3.5, Math.max(0, poseError / 1.2));
    const base = Math.exp(-errN * 0.95);
    return Math.max(0.03, Math.min(0.92, (0.08 + 0.60 * base + 0.32 * perimN) * angleW));
}

function clampNegPos1(v) { return Math.max(-1, Math.min(1, v)); }

function rotationMatrixFromRvec(rvec) {
    const rx = rvec[0], ry = rvec[1], rz = rvec[2];
    const theta = Math.hypot(rx, ry, rz);
    if (theta < 1e-9) {
        return [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ];
    }
    const kx = rx / theta, ky = ry / theta, kz = rz / theta;
    const c = Math.cos(theta), s = Math.sin(theta), v = 1 - c;
    return [
        [kx * kx * v + c,      kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c,      ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c]
    ];
}

function viewAngleDegFromRotationMatrix(R) {
    // Marker normal in camera frame is R*[0,0,1] => 3rd column.
    const nz = R[2][2];
    const frontal = Math.abs(clampNegPos1(nz));
    return Math.acos(frontal) * 180 / Math.PI;
}

function viewAngleDegFromRvec(rvec) {
    const R = rotationMatrixFromRvec(rvec);
    return viewAngleDegFromRotationMatrix(R);
}
