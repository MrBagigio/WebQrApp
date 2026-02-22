/**
 * RestorationEngine.js — AR engine for 3D house restoration with multi-marker tracking.
 *
 * Coordinate systems:
 *   POSIT:   Y-up, Z-forward  (marker Z points out of marker surface)
 *   Three.js: Y-up, Z-backward (camera looks along -Z)
 *
 * Conversion strategy:
 *   Rotation:  conjugate by F = diag(1,1,-1) → negate qz component
 *   Position:  negate tz
 *   Model:     modelGroup.scale.z = -1 compensates the Z-mirror on geometry
 *
 * Marker layout (table setup, viewed from above, NO side markers):
 *
 *   IDs used: 1,2,3,4,5,6,7,8 (9/10 currently disabled)
 *
 *      1      7      2
 *      5    [HOUSE]  6
 *      3      8      4
 */

export class RestorationEngine {

    // ── Constructor ──────────────────────────────────────────────────────────

    constructor() {
        this.video = null;
        this.overlay = null;
        this.overlayCtx = null;
        this.renderer = null;
        this.scene = null;
        this.camera = null;
        this.modelGroup = null;
        this.destroyedModel = null;
        this.restoredModel = null;
        this.worker = null;
        this._workerReady = false;
        this._workerBusy = false;
        this.isTracking = false;
        this._lastTrackingTime = 0;
        this._trackingTimeout = 800;
        this._framesWithoutDetection = 0;
        this.markerSizeMM = 40;   // 4cm markers (user's physical setup)
        this.houseSizeMM = 195;  // slight upscale for better visual match
        this.focal = 800;
        this._modelBBox = null;
        this._markerHelpers = {};
        this._showMarkerHelpers = false;
        this._markerPoseAxes = null;
        this._housePivotHelper = null;
        this._modelYOffset = 0; // meters, manual vertical tuning from UI
        this._modelXOffset = 0;
        this._modelZOffset = 0;
        this._modelScaleFactor = 1;
        this._targetModelSize = this.houseSizeMM / 1000;
        this._validMarkerIds = new Set([1]);
        this._singleMarkerMode = true;
        this._singleMarkerOffsetTemplate = null;
        this._detectionFps = 30;
        // Last raw markers received from worker (copy of data.markers)
        this._lastRawMarkers = [];
        this._lastRawMarkersTs = 0;
        this._debugOverlayHoldMs = 350;
        this._lastDetectionTime = 0;
        this._maxDetectSize = 768;
        this._detectionCanvas = null;
        this._detectionCanvasCtx = null;

        // Pose filters (initialized later in _initThree)
        this.posFilter = null;
        this.quatFilter = null;

        // Small history buffer for position median filtering (reduces spikes)
        this._positionHistory = [];
        this._positionHistorySize = 3; // robust median over short window (anti-jitter)

        // High-frequency render-space smoothing toward measurement target
        this._poseTargetPosition = null;
        this._poseTargetQuaternion = null;
        this._lastRenderTime = 0;

        this._debugOverlayEnabled = true;
        this._centerLockStrength = 0.35;
        this._centerLockMaxMeters = 0.08;

        // First-pose flag: snap to first detected pose, then smooth after
        this._hasFirstPose = false;

        // Runtime tuning / detection helpers
        this._minMarkerPerimeter = 30;    // px - ignore tiny detections

        // Optional camera calibration (cameraMatrix [9], distCoeffs [])
        this._cameraMatrix = null;
        this._distCoeffs = null;

        this.restorationLevel = 0;
        this.onLog = null;
        this._stopped = false;
        this._rafId = null;
    }

    // ── Pose helpers ───────────────────────────────────────────────────────
    /**
     * Return the most recently applied pose measurement (in world coords).
     * Useful for logging or exporting from browser console.
     */
    getLastPose() {
        return this._lastPoseInfo ? {
            pos: this._lastPoseInfo.position.toArray(),
            quat: this._lastPoseInfo.quaternion.toArray(),
            error: this._lastPoseInfo.error,
            confidence: this._lastPoseInfo.confidence
        } : null;
    }

    // ── Logging ──────────────────────────────────────────────────────────────

    log(msg, type = 'info') {
        if (this.onLog) this.onLog(msg, type);
        else console.log(`[Engine] ${msg}`);
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    async init() {
        this.video = document.getElementById('video');
        this.overlay = document.getElementById('overlay');
        this.overlayCtx = this.overlay.getContext('2d');

        try {
            this.log('Reset sistemi e camera...');
            this.stop();
            this._stopped = false;

            if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
                throw new Error('API Camera non trovata. Usa HTTPS.');
            }

            this.log('Richiesta accesso fotocamera...');
            let stream;
            try {
                stream = await navigator.mediaDevices.getUserMedia({
                    video: {
                        facingMode: { ideal: 'environment' },
                        width: { ideal: 1920 },
                        height: { ideal: 1080 }
                    }
                });
            } catch (err) {
                if (err.name === 'NotAllowedError') {
                    throw new Error('Permesso negato. Controlla impostazioni browser o usa HTTPS.');
                }
                
                // Fallback for local testing without HTTPS/Camera permissions
                if (location.hostname === 'localhost' || location.hostname === '127.0.0.1') {
                    this.log('Running on localhost without camera access. Using mock video stream.', 'warn');
                    const canvas = document.createElement('canvas');
                    canvas.width = 640;
                    canvas.height = 480;
                    const ctx = canvas.getContext('2d');
                    ctx.fillStyle = '#333';
                    ctx.fillRect(0, 0, canvas.width, canvas.height);
                    ctx.fillStyle = '#fff';
                    ctx.font = '20px Arial';
                    ctx.fillText('Mock Camera Stream', 50, 50);
                    stream = canvas.captureStream(30);
                } else {
                    throw new Error('Camera indisponibile. Chiudi altre app o usa HTTPS.');
                }
            }

            this.video.srcObject = stream;
            try { await this.video.play(); } catch (e) { this.log('Play: ' + e.message, 'warn'); }

            this.log('Attesa dimensioni video...');
            for (let i = 0; i < 30 && !this.video.videoWidth; i++) {
                await new Promise(r => setTimeout(r, 100));
            }

            const isIPhone = /iPhone/i.test(navigator.userAgent || '');
            if (isIPhone) {
                this._maxDetectSize = 960;
                this.log('iPhone mode: detection size 960');
            }

            this._setupCanvas();
            this._initThree();
            this._estimateFocal();
            this._syncProjection();
            this._initWorker();
            this._loop();

            // Auto-select preset per device
            try {
                const preset = isIPhone ? 'iphone13pro' : 'minimal';
                this.applyStabilityPreset(preset);
            } catch (e) { /* ignore */ }

            this.log('Tutti i sistemi inizializzati.');
        } catch (e) {
            this.log('ERRORE: ' + e.message, 'error');
            const el = document.getElementById('tracking-status');
            if (el) el.textContent = 'KO: ' + e.message;
        }
    }

    stop() {
        this._stopped = true;
        if (this._rafId) {
            cancelAnimationFrame(this._rafId);
            this._rafId = null;
        }
        if (this.video?.srcObject) {
            this.video.srcObject.getTracks().forEach(t => t.stop());
            this.video.srcObject = null;
        }
        if (this.worker) {
            this.worker.terminate();
            this.worker = null;
        }
    }

    // ── Canvas setup ─────────────────────────────────────────────────────────

    _setupCanvas() {
        this.overlay.width = this.video.videoWidth;
        this.overlay.height = this.video.videoHeight;
        this.overlay.style.width = '100%';
        this.overlay.style.height = '100%';
        this.overlay.style.objectFit = 'cover';

        // Small offscreen canvas for downscaled detection frames
        if (!this._detectionCanvas) this._detectionCanvas = document.createElement('canvas');
        const maxDim = this._maxDetectSize;
        const ratio = Math.min(1, maxDim / Math.max(this.overlay.width, this.overlay.height));
        this._detectionCanvas.width = Math.max(1, Math.round(this.overlay.width * ratio));
        this._detectionCanvas.height = Math.max(1, Math.round(this.overlay.height * ratio));
        this._detectionCanvasCtx = this._detectionCanvas.getContext('2d');
    }

    // ── Camera intrinsics ────────────────────────────────────────────────────

    _estimateFocal() {
        if (!this.video || !this.overlay) return;
        const w = this.overlay.width || 0;
        // height not needed for horizontal focal estimation
        
        // 1. Try to get hardware FOV
        let estimated = this._getHardwareFocal(w);

        // 2. Fallback if hardware FOV failed
        if (!estimated || estimated < 100) {
            const isIPhone = /iPhone/i.test(navigator.userAgent || '');
            // Fallback tuned for mobile rear cameras; previous 0.9*max() was too high in portrait.
            estimated = w * (isIPhone ? 0.73 : 0.78);
        }

        // 3. Clamp to plausible range
        const minF = Math.max(100, w * 0.55);
        const maxF = Math.max(minF + 1, w * 1.4);
        this.focal = Math.max(minF, Math.min(maxF, estimated));

        this.log(`Focale stimata: ${Math.round(this.focal)} px`);
    }

    _getHardwareFocal(width) {
        try {
            const track = this.video.srcObject?.getVideoTracks()[0];
            if (!track) return 0;
            
            const settings = track.getSettings ? track.getSettings() : {};
            let hFov = Number(settings.fov);
            const facingMode = settings.facingMode || '';
            const isIPhone = /iPhone/i.test(navigator.userAgent || '');

            if (!Number.isFinite(hFov) || hFov < 20 || hFov > 140) {
                // Heuristics based on device type
                if (facingMode === 'environment') {
                    hFov = isIPhone ? 69 : 67; 
                } else {
                    hFov = 60; // Default wide
                }
            }
            return (width / 2) / Math.tan((hFov * Math.PI / 180) / 2);
        } catch (e) {
            this.log('FOV detect: ' + e.message, 'warn');
            return 0;
        }
    }

    // Alias for the UI calibrate button
    updateFocalFromCamera() {
        this._estimateFocal();
        this._syncProjection();
    }

    _syncProjection() {
        if (!this.overlay || !this.overlay.width || !this.overlay.height) return;
        const w = this.overlay.width;
        const h = this.overlay.height;
        const fx = (this._cameraMatrix && this._cameraMatrix.length >= 9) ? this._cameraMatrix[0] : this.focal;
        const fy = (this._cameraMatrix && this._cameraMatrix.length >= 9) ? this._cameraMatrix[4] : this.focal;
        const cx = (this._cameraMatrix && this._cameraMatrix.length >= 9) ? this._cameraMatrix[2] : (w / 2);
        const cy = (this._cameraMatrix && this._cameraMatrix.length >= 9) ? this._cameraMatrix[5] : (h / 2);
        const near = 0.01, far = 100;

        // OpenCV intrinsics → Three.js NDC projection (column-major)
        const m = new THREE.Matrix4();
        m.set(
            2 * fx / w,  0,            -(2 * cx / w - 1),  0,
            0,           2 * fy / h,   -(2 * cy / h - 1),  0,
            0,           0,            -(far + near) / (far - near), -2 * far * near / (far - near),
            0,           0,            -1,                  0
        );
        this.camera.projectionMatrix.copy(m);
        this.camera.projectionMatrixInverse.copy(m).invert();

        const container = document.getElementById('three-container');
        if (container && this.renderer) { // both needed for setSize
            this.renderer.setSize(container.clientWidth, container.clientHeight);
        }
    }

    // ── Public setters ───────────────────────────────────────────────────────

    setMarkerSizeMM(mm) {
        this.markerSizeMM = Math.max(10, Number(mm) || 100);
        if (this.worker) {
            try { this.worker.postMessage({ type: 'config', markerLength: this.markerSizeMM / 1000 }); }
            catch (e) { this.log('Worker config: ' + e.message, 'warn'); }
        }
        this.log(`Marker: ${this.markerSizeMM} mm`);
    }

    setHouseSizeMM(mm) {
        this.houseSizeMM = Math.max(10, Number(mm) || 200);
        this._targetModelSize = this.houseSizeMM / 1000;
        const target = this._targetModelSize;
        if (this.restoredModel) {
            this._scaleModel(this.restoredModel, target);
            this.restoredModel.userData.baseScale = this.restoredModel.scale.clone();
            this.restoredModel.userData.basePosition = this.restoredModel.position.clone();
            this._applyModelScaleTransform(this.restoredModel);
        }
        if (this.destroyedModel) {
            this._scaleModel(this.destroyedModel, target);
            this.destroyedModel.userData.baseScale = this.destroyedModel.scale.clone();
            this.destroyedModel.userData.basePosition = this.destroyedModel.position.clone();
            this._applyModelScaleTransform(this.destroyedModel);
        }
        this._recomputeBBox();
        this._repositionMarkerHelpers();
        this.log(`Casa: ${this.houseSizeMM} mm → ${(this._targetModelSize).toFixed(3)} m (scale ${this._modelScaleFactor.toFixed(2)}x)`);
    }

    setModelYOffset(meters) {
        this._modelYOffset = Number.isFinite(Number(meters)) ? Number(meters) : this._modelYOffset;
        this._applyOffsets();
        this.log(`Model Y offset: ${(this._modelYOffset * 100).toFixed(1)} cm`);
    }

    setModelXOffset(meters) {
        this._modelXOffset = Number.isFinite(Number(meters)) ? Number(meters) : this._modelXOffset;
        this._applyOffsets();
        this.log(`Model X offset: ${(this._modelXOffset * 100).toFixed(1)} cm`);
    }

    setModelZOffset(meters) {
        this._modelZOffset = Number.isFinite(Number(meters)) ? Number(meters) : this._modelZOffset;
        this._applyOffsets();
        this.log(`Model Z offset: ${(this._modelZOffset * 100).toFixed(1)} cm`);
    }

    _applyOffsets() {
        if (this.restoredModel) {
            const basePos = this.restoredModel.userData?.basePosition;
            if (basePos) this.restoredModel.position.set(basePos.x + this._modelXOffset, basePos.y + this._modelYOffset, basePos.z + this._modelZOffset);
        }
        if (this.destroyedModel) {
            const basePos = this.destroyedModel.userData?.basePosition;
            if (basePos) this.destroyedModel.position.set(basePos.x + this._modelXOffset, basePos.y + this._modelYOffset, basePos.z + this._modelZOffset);
        }
    }

    updateDetectionFps(fps) {
        this._detectionFps = Math.max(1, Math.min(60, Number(fps) || 24));
    }

    setDebugOverlayEnabled(enable) {
        this._debugOverlayEnabled = !!enable;
        if (this._housePivotHelper) this._housePivotHelper.visible = this._debugOverlayEnabled;
        if (this._markerPoseAxes) this._markerPoseAxes.visible = this._debugOverlayEnabled && this.isTracking;
        if (!this._debugOverlayEnabled && this.overlayCtx && this.overlay) {
            this.overlayCtx.drawImage(this.video, 0, 0, this.overlay.width, this.overlay.height);
        }
        this.log('Debug overlay ' + (this._debugOverlayEnabled ? 'enabled' : 'disabled'));
    }

    // Position history buffer size (median filter)
    setPositionHistorySize(n) {
        this._positionHistorySize = Math.max(1, Math.min(15, Number(n) || this._positionHistorySize));
        this.log('Position history size: ' + this._positionHistorySize);
        if (this._positionHistory && this._positionHistory.length > this._positionHistorySize) {
            this._positionHistory = this._positionHistory.slice(-this._positionHistorySize);
        }
    }

    // Camera calibration input (cameraMatrix: 9 elements row-major, distCoeffs array optional)
    setCameraCalibration(cameraMatrix, distCoeffs) {
        try {
            if (!Array.isArray(cameraMatrix) || cameraMatrix.length < 9) throw new Error('cameraMatrix must be 9 numbers');
            this._cameraMatrix = cameraMatrix.slice(0,9);
            if (Array.isArray(distCoeffs)) this._distCoeffs = distCoeffs.slice();
            // adopt fx/cx/cy from provided matrix
            this.focal = this._cameraMatrix[0];
            this._syncProjection();
            localStorage.setItem('expear.cameraMatrix', JSON.stringify(this._cameraMatrix));
            if (this._distCoeffs) localStorage.setItem('expear.distCoeffs', JSON.stringify(this._distCoeffs));
            this.log('Camera calibration applied');
        } catch (e) { this.log('setCameraCalibration failed: ' + e.message, 'error'); }
    }

    setMinMarkerPerimeter(px) {
        this._minMarkerPerimeter = Math.max(4, Number(px) || this._minMarkerPerimeter);
        this.log('minMarkerPerimeter=' + this._minMarkerPerimeter);
    }

    // Apply preset tuned for a camera/device
    applyStabilityPreset(name = 'default') {
        const presets = {
            minimal: {
                filter: { positionSmoothing: 0.05, rotationTimeConstant: 0.04 },
                posHist: 1, maxJump: 0.25, minPeri: 30, anchorBoost: 1.0,
                ekf: false, anchorIds: null, autoLock: false, clearLock: true, lockEnabled: false,
                fps: 20,
                worker: {
                    cornerSmooth: 0.0, flow: false, subpix: false,
                    apriltag: false, pnp: false, lk: false,
                    outlier: 0.25, conf: 0.15
                }
            },
            mobile: {
                filter: { positionSmoothing: 0.08, rotationTimeConstant: 0.05 },
                posHist: 3, maxJump: 0.3, minPeri: 30, anchorBoost: 2.5,
                ekf: false, anchorIds: null, autoLock: false, clearLock: true, lockEnabled: false,
                fps: 60,
                worker: {
                    cornerSmooth: 0.3, flow: true, flowSSD: 40,
                    subpix: true, subpixParams: { win: 3, maxIter: 15, eps: 0.1 },
                    apriltag: true, pnp: true, lk: true,
                    outlier: 0.4, conf: 0.15
                }
            },
            desktop: {
                filter: { positionSmoothing: 0.05, rotationTimeConstant: 0.04 },
                posHist: 1, maxJump: 0.5, minPeri: 25,
                ekf: false, autoLock: false, clearLock: true,
                fps: 60,
                worker: {}
            }
        };
        // Aliases
        presets.iphone13pro = presets.mobile;

        const p = presets[name];
        if (!p) {
            this.log('Unknown preset: ' + name, 'warn');
            return;
        }

        // Apply
        if (p.filter) this.setFilterParams(p.filter);
        if (p.posHist !== undefined) this.setPositionHistorySize(p.posHist);
        if (p.maxJump !== undefined) this.setMaxPositionJump(p.maxJump);
        if (p.minPeri !== undefined) this.setMinMarkerPerimeter(p.minPeri);
        if (p.anchorBoost !== undefined) this.setAnchorBoost(p.anchorBoost);
        if (p.ekf !== undefined) this.setUseQuaternionEKF(p.ekf);
        if (p.anchorIds !== undefined) this.setAnchorIds(p.anchorIds);
        
        if (p.autoLock !== undefined) this.setAnchorAutoLockEnabled(p.autoLock);
        if (p.clearLock) this.clearAnchorLock({ persist: true });
        if (p.lockEnabled !== undefined) this.setAnchorLockEnabled(p.lockEnabled);
        
        if (p.fps) this.updateDetectionFps(p.fps);

        // Worker configs (safely applied)
        const w = p.worker || {};
        try {
            if (w.cornerSmooth !== undefined) this.setCornerSmoothing(w.cornerSmooth);
            if (w.flow !== undefined) this.setCornerFlowEnabled(w.flow);
            if (w.flowSSD !== undefined) this.setCornerFlowSSDThreshold(w.flowSSD);
            if (w.subpix !== undefined) {
                this.setUseSubpixel(w.subpix);
                if (w.subpixParams) this.setSubpixelParams(w.subpixParams);
            }
            if (w.apriltag !== undefined) this.setUseAprilTag(w.apriltag);
            if (w.pnp !== undefined) this.setUseSolvePnP(w.pnp);
            if (w.lk !== undefined) this.setUsePyrLKFlow(w.lk);
            if (w.outlier !== undefined) this.setMarkerOutlierDistanceMeters(w.outlier);
            if (w.conf !== undefined) this.setMarkerConfidenceThreshold(w.conf);
        } catch (e) { /* ignore worker config errors if methods missing */ }

        this.log(`Stability preset applied: ${name}`);
    }

    // Utility: compute polygon perimeter (px) from corners array [[x,y],...]
    _markerPerimeter(corners) {
        if (!corners || corners.length < 4) return 0;
        let p = 0; for (let i = 0; i < corners.length; i++) {
            const a = corners[i], b = corners[(i+1) % corners.length];
            const dx = a[0] - b[0], dy = a[1] - b[1]; p += Math.hypot(dx, dy);
        }
        return p;
    }

    // Runtime API: update filter bandwidths
    setFilterParams({ positionSmoothing, rotationTimeConstant } = {}) {
        if (typeof positionSmoothing === 'number') {
            // Map smoothing value to responsiveness: lower smoothing = higher responsiveness
            const responsiveness = Math.max(0.02, Math.min(0.99, 1.0 - positionSmoothing));
            
            if (this.posFilter && this.posFilter.responsiveness !== undefined) {
                this.posFilter.responsiveness = responsiveness;
                this.log(`Position filter updated: responsiveness=${responsiveness.toFixed(3)}`);
            } else {
                this.posFilter = new PoseFilters.PredictivePositionFilter({
                    responsiveness: responsiveness,
                    velocitySmoothing: 0.25,      // smoother velocity estimation for board setup
                    predictionFactor: 0.2,         // conservative prediction to reduce inter-frame drift
                    maxVelocity: 1.0,              // real camera movement is slow; reject fast spikes
                    maxPredictionDt: 0.033,
                    maxPredictionStep: 0.015,
                    velocityDamping: 0.9,
                    positionDeadband: 0.0015
                });
                this.log(`Position filter init: responsiveness=${responsiveness.toFixed(3)}`);
            }
        }
        if (typeof rotationTimeConstant === 'number') {
            if (this.quatFilter && this.quatFilter.timeConstant !== undefined) {
                this.quatFilter.timeConstant = rotationTimeConstant;
                this.log(`Rotation filter updated: tau=${rotationTimeConstant.toFixed(3)}`);
            } else {
                this.quatFilter = new PoseFilters.QuaternionFilter({ timeConstant: rotationTimeConstant });
                this.log(`Rotation timeConstant set: ${rotationTimeConstant}`);
            }
        }
    }

    // Set or override marker offsets at runtime. `map` expects id -> { pos: [x,y,z], quat?: [x,y,z,w] }
    setMarkerOffsets(map, { persist = true } = {}) {
        if (!map || typeof map !== 'object') return;
        this._markerOffsets = this._markerOffsets || {};
        for (const idStr of Object.keys(map)) {
            const id = Number(idStr);
            const v = map[idStr];
            const pos = (v.pos && v.pos.length === 3) ? new THREE.Vector3(v.pos[0], v.pos[1], v.pos[2]) : new THREE.Vector3(0,0,0);
            let quat = new THREE.Quaternion();
            if (v.quat && v.quat.length === 4) quat.copy(new THREE.Quaternion(v.quat[0], v.quat[1], v.quat[2], v.quat[3]));
            else if (v.euler && v.euler.length === 3) quat.setFromEuler(new THREE.Euler(v.euler[0], v.euler[1], v.euler[2]));
            else quat.identity();
            this._markerOffsets[id] = { positionOffset: pos, rotationOffset: quat };
        }
        this._repositionMarkerHelpers();
        if (persist) this._saveMarkerOffsetsToStorage();
        this.log('Marker offsets aggiornati');
    }

    // Return runtime marker offsets (meters, plain object) — useful for UI
    getMarkerOffsets() {
        const out = {};
        if (!this._markerOffsets) return out;
        for (const id in this._markerOffsets) {
            const o = this._markerOffsets[id];
            out[id] = {
                pos: [o.positionOffset.x, o.positionOffset.y, o.positionOffset.z],
                quat: [o.rotationOffset.x, o.rotationOffset.y, o.rotationOffset.z, o.rotationOffset.w]
            };
        }
        return out;
    }

    // Remove runtime override for a specific marker (optionally persist)
    clearMarkerOffset(id, { persist = true } = {}) {
        if (!this._markerOffsets || typeof id === 'undefined' || this._markerOffsets[id] == null) return;
        delete this._markerOffsets[id];
        this._repositionMarkerHelpers();
        if (persist) this._saveMarkerOffsetsToStorage();
        this.log('Marker offset cleared for id: ' + id);
    }

    // Limit instantaneous position jumps (meters)
    setMaxPositionJump(meters) {
        this._maxPositionJump = Math.max(0.01, Number(meters) || this._maxPositionJump);
        this.log(`Max position jump ${this._maxPositionJump} m`);
    }

    // Persist/load helper
    _saveMarkerOffsetsToStorage() {
        try {
            const plain = {};
            if (this._markerOffsets) {
                for (const id in this._markerOffsets) {
                    const o = this._markerOffsets[id];
                    plain[id] = { pos: [o.positionOffset.x, o.positionOffset.y, o.positionOffset.z], quat: [o.rotationOffset.x, o.rotationOffset.y, o.rotationOffset.z, o.rotationOffset.w] };
                }
            }
            localStorage.setItem(this._markerOffsetsStorageKey, JSON.stringify(plain));
            this.log('Marker offsets salvati in localStorage');
        } catch (e) { this.log('Salvataggio offsets fallito: ' + e.message, 'error'); }
    }

    _loadMarkerOffsetsFromStorage() {
        try {
            const raw = localStorage.getItem(this._markerOffsetsStorageKey);
            if (!raw) return;
            const parsed = JSON.parse(raw);
            const map = {};
            for (const id in parsed) map[id] = { pos: parsed[id].pos, quat: parsed[id].quat };
            this.setMarkerOffsets(map, { persist: false });
            this.log('Marker offsets caricati da localStorage');
        } catch (e) { /* ignore silently */ }
    }

    setRestorationLevel(level) {
        this.restorationLevel = level;
        if (this.restoredModel) {
            this.restoredModel.traverse(c => {
                if (c.isMesh) c.material.opacity = level;
            });
        }
        if (this.destroyedModel) {
            this.destroyedModel.traverse(c => {
                if (c.isMesh) c.material.opacity = (1 - level) * 0.5;
            });
        }
    }

    // ── Model scaling (safe for repeated calls) ──────────────────────────────

    setModelScale(scaleFactor) {
        const parsed = Number(scaleFactor);
        if (!Number.isFinite(parsed)) return;
        this._modelScaleFactor = Math.max(0.1, Math.min(3.0, parsed));
        if (this.restoredModel) {
            this._applyModelScaleTransform(this.restoredModel);
        }
        if (this.destroyedModel) {
            this._applyModelScaleTransform(this.destroyedModel);
        }
        this.log(`Model scale: ${this._modelScaleFactor.toFixed(2)}x`);
    }

    _applyModelScaleTransform(object) {
        if (!object) return;
        const baseScale = object.userData?.baseScale;
        const basePosition = object.userData?.basePosition;
        if (!(baseScale instanceof THREE.Vector3) || !(basePosition instanceof THREE.Vector3)) return;

        object.scale.copy(baseScale).multiplyScalar(this._modelScaleFactor);
        object.position.set(basePosition.x + this._modelXOffset, basePosition.y + this._modelYOffset, basePosition.z + this._modelZOffset);
        object.updateMatrixWorld(true);
    }

    /**
     * Scale a loaded FBX so its largest dimension equals `targetMeters`.
     * Resets scale/position first so repeated calls are idempotent.
     */
    _scaleModel(object, targetMeters) {
        if (!object || !Number.isFinite(targetMeters) || targetMeters <= 0) return;
        // Reset to original transform before re-computing
        object.scale.set(1, 1, 1);
        object.position.set(0, 0, 0);
        object.updateMatrixWorld(true);

        const box = new THREE.Box3().setFromObject(object);
        const size = new THREE.Vector3();
        box.getSize(size);
        const maxDim = Math.max(size.x, size.y, size.z);
        if (maxDim > 0) {
            const s = targetMeters / maxDim;
            object.scale.set(s, s, s);
        }

        // Re-centre: pivot at bottom-centre of the bounding box
        object.updateMatrixWorld(true);
        const box2 = new THREE.Box3().setFromObject(object);
        const centre = new THREE.Vector3();
        box2.getCenter(centre);
        
        // Sposta l'oggetto in modo che il centro (X, Z) sia a 0, e la base (min.y) sia a 0
        // Nota: la posizione dell'oggetto è relativa al suo parent (modelRoot).
        // Sottraendo box2.min.y, stiamo sollevando l'oggetto in modo che il suo punto più basso
        // coincida con y=0 del parent.
        
        // Convertiamo il centro globale in coordinate locali dell'oggetto
        const localCenter = object.worldToLocal(centre.clone());
        const localMinY = object.worldToLocal(new THREE.Vector3(0, box2.min.y, 0)).y;

        object.position.x -= localCenter.x * object.scale.x;
        object.position.z -= localCenter.z * object.scale.z;
        object.position.y -= localMinY * object.scale.y;
        
        // Aggiorna la matrice per assicurarsi che il pivot sia corretto
        object.updateMatrixWorld(true);
        
        // Forza l'aggiornamento del bounding box dopo lo spostamento
        const finalBox = new THREE.Box3().setFromObject(object);
        this.log(`Model centered. New min.y: ${finalBox.min.y.toFixed(4)}`);
    }

    /** Compute model bounding box from whichever model is loaded. */
    _recomputeBBox() {
        const model = this.restoredModel || this.destroyedModel;
        if (!model) return;
        model.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(model);
        const size = new THREE.Vector3();
        box.getSize(size);
        this._modelBBox = {
            halfW: size.x / 2,
            height: size.y,
            halfD: size.z / 2
        };
        this.log(`Bbox: ${size.x.toFixed(3)} × ${size.y.toFixed(3)} × ${size.z.toFixed(3)} m`);
    }

    // ── Marker offsets ───────────────────────────────────────────────────────
    //
    // Each marker is physically glued to a face of the printed house.
    // `positionOffset` = vector from house origin (bottom-centre) to marker centre,
    //                     in the house's local coordinate frame.
    // `rotationOffset` = inverse of (marker-frame → house-frame rotation).
    //   i.e.  Q_house = Q_marker × rotationOffset
    //
    // Marker-to-house rotation Q_m2h for each face:
    //   Front (+Z): identity        → offset = identity
    //   Back  (-Z): 180°Y           → offset = inv(180°Y) = 180°Y
    //   Right (+X): +90°Y           → offset = inv(+90°Y)  = -90°Y
    //   Left  (-X): -90°Y           → offset = inv(-90°Y)  = +90°Y

    _markerOffsetsForId(id) {
        // If runtime overrides exist (setMarkerOffsets), use them first
        if (this._markerOffsets && this._markerOffsets[id]) {
            const o = this._markerOffsets[id];
            return { rotationOffset: o.rotationOffset.clone(), positionOffset: o.positionOffset.clone() };
        }

        // In single-marker mode, default to neutral offset:
        // - no extra rotation (keeps the house perpendicular to the marker plane)
        // - no position translation (house origin sits on marker center)
        // A custom template can still override this behavior.
        if (this._singleMarkerMode && this._singleMarkerOffsetTemplate) {
            const t = this._singleMarkerOffsetTemplate;
            return { rotationOffset: t.rotationOffset.clone(), positionOffset: t.positionOffset.clone() };
        }
        if (this._singleMarkerMode) {
            return {
                rotationOffset: new THREE.Quaternion(),
                positionOffset: new THREE.Vector3(0, 0, 0)
            };
        }

        const rot = new THREE.Quaternion();
        const pos = new THREE.Vector3();

        // All markers are flat on the table (no side markers).
        // Base rotation aligns marker normal to house up.
        // NOTE: do NOT add extra 180° yaw here, otherwise center offset is mirrored.
        // The marker's Z axis points UP from the paper. The house's Y axis points UP.
        // We need to rotate the house so its Y axis aligns with the marker's Z axis.
        rot.setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0, 'XYZ'));

        // Marker positions (centers) in meters:
        // Layout from NewArUcoSetup.png (bottom side = house front):
        // back row:  1 - 7 - 2
        // middle:    5     6
        // front row: 3 - 8 - 4
        const xOffset = 0.14;
        const zOffset = 0.10;
        const yOffset = 0.0; // Flat on the table

        switch (id) {
            case 1: // Back-Left
                pos.set(-xOffset, yOffset, zOffset);
                break;
            case 7: // Back-Center
                pos.set(0, yOffset, zOffset);
                break;
            case 2: // Back-Right
                pos.set(xOffset, yOffset, zOffset);
                break;
            case 5: // Middle-Left
                pos.set(-xOffset, yOffset, 0);
                break;
            case 6: // Middle-Right
                pos.set(xOffset, yOffset, 0);
                break;
            case 3: // Front-Left
                pos.set(-xOffset, yOffset, -zOffset);
                break;
            case 8: // Front-Center
                pos.set(0, yOffset, -zOffset);
                break;
            case 4: // Front-Right
                pos.set(xOffset, yOffset, -zOffset);
                break;
            default:
                return null;
        }
        return { rotationOffset: rot, positionOffset: pos };
    }

    // ── Three.js scene ───────────────────────────────────────────────────────

    _initThree() {
        const container = document.getElementById('three-container');

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.2;
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.xr.enabled = true;
        container.appendChild(this.renderer.domElement);

        window.addEventListener('resize', () => this._syncProjection());

        // Scene
        this.scene = new THREE.Scene();

        // Camera (projection overwritten by _syncProjection)
        this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
        this.camera.position.set(0, 0, 0);
        this.camera.lookAt(0, 0, -1);
        this._syncProjection();

        // Lighting
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.5));

        const key = new THREE.DirectionalLight(0xfff5e6, 1.0);
        key.position.set(2, 4, 2);
        key.castShadow = true;
        key.shadow.mapSize.set(1024, 1024);
        key.shadow.camera.left = -0.5;
        key.shadow.camera.right = 0.5;
        key.shadow.camera.top = 0.5;
        key.shadow.camera.bottom = -0.5;
        key.shadow.bias = -0.001;
        this.scene.add(key);

        const fill = new THREE.DirectionalLight(0xb0c4ff, 0.4);
        fill.position.set(-2, 1, -1);
        this.scene.add(fill);

        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 0.3));

        // Model container
        this.modelGroup = new THREE.Group();
        this.modelGroup.scale.set(1, 1, 1); // Removed scale.z = -1 (was causing mirroring/inversion)
        this.modelGroup.visible = false;
        this.scene.add(this.modelGroup);

        // modelRoot: actual FBX objects are parented here so their local pivot
        // (bottom-center) remains stable while modelGroup is the tracked transform.
        this._modelRoot = new THREE.Group();
        this.modelGroup.add(this._modelRoot);

        // Debug helper: pivot della casetta (origine modelGroup)
        this._housePivotHelper = new THREE.Group();
        const houseAxes = new THREE.AxesHelper(0.12);
        const housePivotDot = new THREE.Mesh(
            new THREE.SphereGeometry(0.008, 16, 16),
            new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x222222, metalness: 0.1, roughness: 0.6 })
        );
        this._housePivotHelper.add(houseAxes);
        this._housePivotHelper.add(housePivotDot);
        this._housePivotHelper.visible = this._debugOverlayEnabled;
        this.modelGroup.add(this._housePivotHelper);

        // Debug helper: assi della posa marker rilevata (prima della correzione upright)
        this._markerPoseAxes = new THREE.AxesHelper(0.12);
        this._markerPoseAxes.visible = false;
        this.scene.add(this._markerPoseAxes);

        // Shadow ground plane
        const shadowPlane = new THREE.Mesh(
            new THREE.PlaneGeometry(2, 2),
            new THREE.ShadowMaterial({ opacity: 0.25 })
        );
        shadowPlane.rotation.x = -Math.PI / 2;
        shadowPlane.position.y = -0.001;
        shadowPlane.receiveShadow = true;
        this.modelGroup.add(shadowPlane);

        // Load FBX models
        this._loadModels();
    }

    // ── FBX loading ──────────────────────────────────────────────────────────

    _loadModels() {
        const loader = new THREE.FBXLoader();
        // global multiplier from UI slider (1.0 default)
        this._modelScaleFactor = Number.isFinite(this._modelScaleFactor) ? this._modelScaleFactor : 1.0;
        this._targetModelSize = this.houseSizeMM / 1000;
        const targetSize = this._targetModelSize;

        // Helper: shared material + scaling + bbox recompute
        const onLoaded = (object, opts) => {
            // Apply a fixed FBX-up correction ONCE at model load time.
            // This keeps pivot/bbox computations consistent with the final visible orientation.
            object.quaternion.premultiply(new THREE.Quaternion().setFromEuler(
                new THREE.Euler(Math.PI / 2, Math.PI, 0, 'XYZ')
            ));
            object.updateMatrixWorld(true);

            this._scaleModel(object, targetSize);
            object.userData.baseScale = object.scale.clone();
            object.userData.basePosition = object.position.clone();
            this._applyModelScaleTransform(object);
            object.traverse(child => {
                if (child.isMesh) {
                    child.castShadow = true;
                    if (opts.receiveShadow) child.receiveShadow = true;
                    child.material = new THREE.MeshStandardMaterial({
                        color: opts.color,
                        transparent: true,
                        opacity: opts.opacity,
                        metalness: opts.metalness || 0,
                        roughness: opts.roughness,
                        ...(opts.envMapIntensity != null && { envMapIntensity: opts.envMapIntensity })
                    });
                }
            });
            // add geometry under modelRoot so scaling/pivot remain at model bottom
            this._modelRoot.add(object);
            this._recomputeBBox();
            this._repositionMarkerHelpers();
            this._hideProgress();
        };

        const onProgress = (label) => (xhr) => {
            if (!xhr || !xhr.lengthComputable) return;
            const pct = Math.round((xhr.loaded / xhr.total) * 100);
            this.log(`${label} ${pct}%`);
            const el = document.getElementById('fbx-progress');
            const lbl = document.getElementById('fbx-progress-label');
            if (el) { el.value = pct; el.style.display = 'block'; }
            if (lbl) lbl.textContent = `${label}: ${pct}%`;
            const ct = document.getElementById('fbx-progress-container');
            if (ct) ct.style.display = 'flex';
        };

        const makeFallbackBox = (color, opacity) => {
            const mesh = new THREE.Mesh(
                new THREE.BoxGeometry(0.04, 0.04, 0.04),
                new THREE.MeshStandardMaterial({ color, transparent: true, opacity, wireframe: opacity < 1 })
            );
            mesh.position.y = 0.02;
            return mesh;
        };

        // Restored house
        this.log('Caricamento casa_restaurata.fbx...');
        loader.load(
            'assets/models/casa_restaurata.fbx',
            (obj) => {
                this.log('Casa restaurata caricata.');
                this.restoredModel = obj;
                onLoaded(obj, {
                    color: 0x8fd4a0, opacity: 0, metalness: 0.05,
                    roughness: 0.65, envMapIntensity: 0.5, receiveShadow: true
                });
            },
            onProgress('casa_restaurata'),
            (err) => {
                this.log('Errore casa_restaurata.fbx — uso fallback.', 'warn');
                console.warn(err);
                this.restoredModel = makeFallbackBox(0x00FF88, 0);
                this.modelGroup.add(this.restoredModel);
                this._hideProgress();
            }
        );

        // Destroyed house
        this.log('Caricamento casa_distrutta.fbx...');
        loader.load(
            'assets/models/casa_distrutta.fbx',
            (obj) => {
                this.log('Casa distrutta caricata.');
                this.destroyedModel = obj;
                onLoaded(obj, {
                    color: 0xcc6644, opacity: 0.5, metalness: 0,
                    roughness: 0.9, receiveShadow: false
                });
            },
            onProgress('casa_distrutta'),
            (err) => {
                console.warn(err);
                this.destroyedModel = makeFallbackBox(0xff4444, 0.5);
                this.modelGroup.add(this.destroyedModel);
                this._hideProgress();
            }
        );
    }

    _hideProgress() {
        const ct = document.getElementById('fbx-progress-container');
        if (ct) setTimeout(() => ct.style.display = 'none', 800);
    }

    // ── Marker debug helpers ─────────────────────────────────────────────────

    _createMarkerHelpers() {
        const size = (this.markerSizeMM / 1000) * 0.15;
        const colors = [0x00ff88, 0x00ccff, 0xffaa00, 0xff44aa, 0xaa00ff, 0xffff00, 0x00ffff, 0xff00ff];
        const markerIds = [1];

        for (let index = 0; index < markerIds.length; index++) {
            const id = markerIds[index];
            const geo = new THREE.BoxGeometry(size, size, size * 0.3);
            const mat = new THREE.MeshStandardMaterial({
                color: colors[index % colors.length], transparent: true, opacity: 0.85,
                metalness: 0.1, roughness: 0.6
            });
            const helper = new THREE.Mesh(geo, mat);
            const off = this._markerOffsetsForId(id);
            if (!off) continue;
            helper.position.copy(off.positionOffset);
            helper.quaternion.copy(off.rotationOffset);
            helper.userData = { markerId: id, label: `ID ${id}` };
            helper.visible = false;
            this._markerHelpers[id] = helper;
            this.modelGroup.add(helper);
        }
    }

    _repositionMarkerHelpers() {
        const markerIds = [1];
        for (const id of markerIds) {
            const h = this._markerHelpers[id];
            if (!h) continue;
            const off = this._markerOffsetsForId(id);
            if (!off) continue;
            h.position.copy(off.positionOffset);
            h.quaternion.copy(off.rotationOffset);
            // keep helper label in sync with logical mapping
            if (h.userData) h.userData.label = `ID ${id}`;
        }
    }

    showMarkerHelpers(enable = true) {
        this._showMarkerHelpers = !!enable;
        for (const id in this._markerHelpers) {
            this._markerHelpers[id].visible = this._showMarkerHelpers;
        }
        this.log(`Marker helpers ${this._showMarkerHelpers ? 'ON' : 'OFF'}`);
    }

    // ── Worker ───────────────────────────────────────────────────────────────

    _initWorker() {
        this.worker = new Worker('workers/aruco-worker.js?v=5');
        // allow dictionary selection via query string for GitHub Pages convenience
        let dict = this._dictionaryName || 'ARUCO';
        try {
            const params = new URLSearchParams(window.location.search);
            const dq = params.get('dict');
            if (dq) dict = dq;
        } catch (_err) {}
        this.worker.postMessage({ type: 'init', markerLength: this.markerSizeMM / 1000, dictionaryName: dict });
        // sync worker-side config
        try { 
            this.worker.postMessage({ 
                type: 'config', 
                cornerSmoothing: 0, 
                cornerFlowEnabled: false, 
                useSolvePnP: !!this._useSolvePnP, 
                usePyrLKFlow: !!this._usePyrLKFlow,
                useAprilTag: false
            }); 
        } catch (e) { /* ignore */ }

        this._workerReady = false;
        this.worker.onmessage = (e) => {
            const d = e.data;
            if (d.type === 'ready') {
                this._workerBusy = false;
                this._workerReady = true;
                this.log('TUTTI I SISTEMI PRONTI. Inquadra il marker!');
            } else if (d.type === 'log') {
                this.log('[Worker] ' + d.message);
            } else if (d.type === 'error') {
                this._workerBusy = false;
                this._workerReady = false;
                this.log('[Worker ERRORE] ' + d.error, 'error');
            } else if (d.type === 'result') {
                this._workerBusy = false;
                this._handleTrackingResult(d);
            }
        };
    }

    // ── Pose conversion ──────────────────────────────────────────────────────

    /**
     * Convert rvec/tvec → Three.js position + quaternion.
     * Handles both POSIT and OpenCV coordinate systems.
     */
    _poseToThreeJs(rvec, tvec, source = 'posit') {
        const angle = Math.hypot(rvec[0], rvec[1], rvec[2]);
        const q = new THREE.Quaternion();
        if (angle >= 1e-6) {
            q.setFromAxisAngle(
                new THREE.Vector3(rvec[0] / angle, rvec[1] / angle, rvec[2] / angle),
                angle
            );
        }

        if (source === 'opencv-pnp') {
            // OpenCV: X right, Y down, Z forward
            // Three.js: X right, Y up, Z backward
            // Transformation: F * R * F where F = diag(1, -1, -1)
            return {
                position: new THREE.Vector3(tvec[0], -tvec[1], -tvec[2]),
                quaternion: new THREE.Quaternion(q.x, -q.y, -q.z, q.w)
            };
        }

        // POSIT (js-aruco2): X right, Y up, Z forward
        // Three.js: X right, Y up, Z backward
        // Transformation: F * R * F where F = diag(1, 1, -1)
        return {
            position: new THREE.Vector3(tvec[0], tvec[1], -tvec[2]),
            quaternion: new THREE.Quaternion(-q.x, -q.y, q.z, q.w)
        };
    }

    // ── Tracking result handler ──────────────────────────────────────────────

    _handleTrackingResult(data) {
        const statusEl = document.getElementById('tracking-status');
        const now = performance.now();
        
        // Use all detected markers for debug/overlay, but filter for pose estimation
        const rawMarkers = data.markers || [];
        const hasIdFilter = this._validMarkerIds instanceof Set && this._validMarkerIds.size > 0;
        const validMarkers = hasIdFilter
            ? rawMarkers.filter(m => this._validMarkerIds.has(Number(m.id)))
            : rawMarkers.slice();

        // Keep a copy for UI / debug (all markers, with validity flag)
        if (rawMarkers.length > 0) {
            this._lastRawMarkers = rawMarkers.map(m => ({
                id: m.id,
                isValid: hasIdFilter ? this._validMarkerIds.has(Number(m.id)) : true,
                corners: m.corners,
                rvec: m.rvec ? m.rvec.slice() : null,
                tvec: m.tvec ? m.tvec.slice() : null,
                poseError: typeof m.poseError === 'number' ? m.poseError : null,
                distance: (m.tvec && m.tvec.length === 3) ? Math.hypot(m.tvec[0], m.tvec[1], m.tvec[2]) : null,
                source: m.source || null,
                confidence: typeof m.confidence === 'number' ? m.confidence : null,
                cameraAngleDeg: Number.isFinite(m.cameraAngleDeg) ? m.cameraAngleDeg : null
            }));
            this._lastRawMarkersTs = now;
        }

        const poseful = validMarkers.filter(m => m.rvec && m.tvec);
        if (poseful.length > 0) {
            console.log('detected', poseful.length, 'valid markers');
            this._applyTrackedPose(poseful, statusEl, now);
            return;
        }
        console.log('tracking lost');
        this._handleTrackingLost(statusEl, now);
    }

    _pushAndMedianPosition(pos) {
        if (!pos) return pos;
        this._positionHistory.push(pos.clone());
        if (this._positionHistory.length > this._positionHistorySize) {
            this._positionHistory.shift();
        }

        if (this._positionHistory.length < 3) return pos;

        const xs = this._positionHistory.map(p => p.x).sort((a, b) => a - b);
        const ys = this._positionHistory.map(p => p.y).sort((a, b) => a - b);
        const zs = this._positionHistory.map(p => p.z).sort((a, b) => a - b);
        const mid = Math.floor(this._positionHistory.length / 2);
        return new THREE.Vector3(xs[mid], ys[mid], zs[mid]);
    }

    _updateRenderPose(now) {
        if (!this.isTracking || !this.modelGroup || !this.modelGroup.visible) return;
        if (!this._poseTargetPosition || !this._poseTargetQuaternion) return;

        const dt = (now - (this._lastRenderTime || now)) / 1000;
        this._lastRenderTime = now;

        // Predict position between frames if filter supports it
        if (this.posFilter && typeof this.posFilter.predict === 'function') {
            const predictedPos = this.posFilter.predict(dt);
            if (predictedPos) {
                this._poseTargetPosition.copy(predictedPos);
            }
        }

        // Smoothly interpolate render transform towards target
        this.modelGroup.position.lerp(this._poseTargetPosition, 0.8);
        this.modelGroup.quaternion.slerp(this._poseTargetQuaternion, 0.8);
    }

    /**
     * Single-marker tracking.
     */
    _applyTrackedPose(poseful, statusEl, now) {
        this._lastTrackingTime = now;
        this._framesWithoutDetection = 0;

        // single marker only
        const m = poseful[0];
        const { position, quaternion } = this._poseToThreeJs(m.rvec, m.tvec, m.source);

        // Anti-sliding: keep projected model origin locked to detected marker center.
        // This compensates small focal/pose errors that appear as lateral drift while orbiting.
        const correctedPosition = this._applyMarkerCenterLock(position, m);

        // Keep tracked quaternion pure marker pose.
        // Model up-axis correction is already baked during FBX loading.
        const finalQuat = quaternion.clone();

        const dt = (now - (this._lastPoseMeasurementTime || now)) / 1000;
        this._lastPoseMeasurementTime = now;

        let filteredPos, filteredQuat;

        if (!this._hasFirstPose) {
            if (this.posFilter) this.posFilter.reset(correctedPosition);
            if (this.quatFilter) this.quatFilter.reset(finalQuat);
            this._hasFirstPose = true;
            filteredPos = correctedPosition.clone();
            filteredQuat = finalQuat.clone();
            this.modelGroup.position.copy(filteredPos);
            this.modelGroup.quaternion.copy(filteredQuat);
            this._poseTargetPosition = filteredPos.clone();
            this._poseTargetQuaternion = filteredQuat.clone();
        } else {
            const conf = m.confidence || 1.0;
            filteredPos = this.posFilter ? this.posFilter.update(correctedPosition, dt, conf) : correctedPosition;
            filteredQuat = this.quatFilter ? this.quatFilter.update(finalQuat, dt, conf) : finalQuat;
            
            this._poseTargetPosition = filteredPos.clone();
            this._poseTargetQuaternion = filteredQuat.clone();
        }

        // Visualizza gli assi della posa marker così da confrontarli col pivot casetta
        // Usiamo la posa filtrata così il debug visivo non scatta
        if (this._markerPoseAxes) {
            this._markerPoseAxes.visible = this._debugOverlayEnabled;
            this._markerPoseAxes.position.copy(filteredPos);
            this._markerPoseAxes.quaternion.copy(filteredQuat);
        }

        // Debug output: log full pose information for external inspection
        try {
            const err = (m && typeof m.poseError !== 'undefined') ? m.poseError.toFixed(3) : 'n/a';
            const conf = (m && typeof m.confidence !== 'undefined') ? m.confidence.toFixed(3) : 'n/a';
            console.log('pose', correctedPosition.toArray().map(n=>n.toFixed(3)), 'quat', this.modelGroup.quaternion.toArray(), 'err', err, 'conf', conf);
        } catch (_e) {}


        this.isTracking = true;
        this.modelGroup.visible = true;
        // save last pose for external query
        this._lastPoseInfo = { position: correctedPosition.clone(), quaternion: this.modelGroup.quaternion.clone(), error: (m && m.poseError) || null, confidence: (m && m.confidence) || null };

        // Update marker helpers
        for (const hid in this._markerHelpers) {
            const hh = this._markerHelpers[hid];
            if (Number(hid) === Number(m.id)) {
                hh.material.opacity = 1;
                hh.scale.set(1.5, 1.5, 1.5);
                hh.visible = this._showMarkerHelpers;
            } else {
                hh.visible = false;
            }
        }

        // Status label
        if (statusEl) {
            const dist = this.modelGroup.position.length();
            statusEl.textContent = `ID ${m.id} · ${dist.toFixed(2)} m`;
            statusEl.style.color = 'var(--p-gold)';
            statusEl.classList.add('tracking');
            statusEl.style.display = ''; // make sure it's visible (was hidden by CSS)
        }
    }

    /** Handle lost-tracking with hysteresis timeout. */
    _handleTrackingLost(statusEl, now) {
        this._framesWithoutDetection++;

        // Tolerate a few dropped frames (hysteresis) before hiding the model.
        // Mobile cameras often drop frames due to motion blur or autofocus.
        // 10 frames is about 300ms at 30fps, enough to prevent rapid flickering.
        if ((now - this._lastTrackingTime) < 700) {
            return;
        }

        if (this._framesWithoutDetection >= 16) {
            this._positionHistory = [];
            this._poseTargetPosition = null;
            this._poseTargetQuaternion = null;
            this._lastRenderTime = 0;
            this._lastPoseMeasurementTime = 0;

            this.isTracking = false;
            if (this.modelGroup) this.modelGroup.visible = false;
            if (this._markerPoseAxes) this._markerPoseAxes.visible = false;
            this._hasFirstPose = false;
            if (statusEl) {
                statusEl.textContent = 'RICERCA TARGET...';
                statusEl.style.color = 'var(--p-dim)';
                statusEl.classList.remove('tracking');
                statusEl.style.display = 'none';
            }
        }
    }

    // ── Overlay drawing ──────────────────────────────────────────────────────

    _drawMarkerOverlay(markers) {
        const ctx = this.overlayCtx;
        // Clear previous marker drawings (video frame is re-drawn in _loop before this)
        // We only clear the marker annotation layer — video background is handled by drawImage in _loop
        const palette = ['#00ff88', '#00ccff', '#ffaa00', '#ff44aa'];

        for (const m of markers) {
            const c = m.corners;
            if (!c || c.length < 4) continue;
            
            // Check isValid property if available (added in recent update) or default to checking valid IDs
            const isValid = (typeof m.isValid === 'boolean') ? m.isValid : this._validMarkerIds.has(Number(m.id));
            const color = isValid ? palette[m.id % palette.length] : '#ff0033'; // Red for invalid

            // Polygon outline
            ctx.strokeStyle = color;
            ctx.lineWidth = isValid ? 3 : 2;
            if (!isValid) ctx.setLineDash([5, 5]); // Dashed for invalid
            else ctx.setLineDash([]);
            
            ctx.lineJoin = 'round';
            ctx.beginPath();
            ctx.moveTo(c[0][0], c[0][1]);
            for (let i = 1; i < c.length; i++) ctx.lineTo(c[i][0], c[i][1]);
            ctx.closePath();
            ctx.stroke();
            ctx.setLineDash([]); // Reset

            // Corner dots and label
            if (isValid) {
                for (let i = 0; i < c.length; i++) {
                    ctx.fillStyle = (i === 0) ? '#ff0000' : color;
                    ctx.beginPath(); ctx.arc(c[i][0], c[i][1], 2, 0, Math.PI * 2); ctx.fill();
                    ctx.font = '9px monospace';
                    ctx.fillStyle = '#ffffff';
                    ctx.fillText(String(i), c[i][0] + 6, c[i][1] - 6);
                }

                // Marker center crosshair + perceived plane diagonals
                const centerX = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
                const centerY = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.moveTo(centerX - 8, centerY); ctx.lineTo(centerX + 8, centerY);
                ctx.moveTo(centerX, centerY - 8); ctx.lineTo(centerX, centerY + 8);
                ctx.stroke();

                ctx.strokeStyle = 'rgba(255,255,255,0.35)';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(c[0][0], c[0][1]); ctx.lineTo(c[2][0], c[2][1]);
                ctx.moveTo(c[1][0], c[1][1]); ctx.lineTo(c[3][0], c[3][1]);
                ctx.stroke();
            }

            // ID Label
            const cx = (c[0][0] + c[2][0]) / 2;
            const cy = (c[0][1] + c[2][1]) / 2;
            
            ctx.font = isValid ? 'bold 12px monospace' : '10px monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            const label = '#' + m.id + (isValid ? '' : ' ERR');
            
            ctx.fillStyle = 'rgba(0,0,0,0.6)';
            const metrics = ctx.measureText(label);
            ctx.fillRect(cx - metrics.width/2 - 2, cy - 8, metrics.width + 4, 16);
            
            ctx.fillStyle = isValid ? '#ffffff' : '#ffcccc';
            ctx.fillText(label, cx, cy);

            // Add distance for valid markers 
            if (isValid && m.tvec && m.tvec.length === 3) {
                const dist = Math.hypot(m.tvec[0], m.tvec[1], m.tvec[2]);
                ctx.fillStyle = '#ddd';
                ctx.font = '10px monospace';
                ctx.fillText(dist.toFixed(2) + ' m', cx, cy + 12);
            }
        }
    }

    _applyMarkerCenterLock(position, marker) {
        if (!marker || !marker.corners || marker.corners.length < 4 || !this.overlay || !this.camera) {
            return position;
        }

        const centerX = (marker.corners[0][0] + marker.corners[1][0] + marker.corners[2][0] + marker.corners[3][0]) / 4;
        const centerY = (marker.corners[0][1] + marker.corners[1][1] + marker.corners[2][1] + marker.corners[3][1]) / 4;

        const projected = position.clone().project(this.camera);
        if (!Number.isFinite(projected.x) || !Number.isFinite(projected.y) || !Number.isFinite(projected.z)) {
            return position;
        }
        const px = (projected.x * 0.5 + 0.5) * this.overlay.width;
        const py = (-projected.y * 0.5 + 0.5) * this.overlay.height;

        const dxPx = centerX - px;
        const dyPx = centerY - py;
        const depth = Math.max(0.01, -position.z);
        const fx = (this._cameraMatrix && this._cameraMatrix.length >= 9) ? this._cameraMatrix[0] : this.focal;
        const fy = (this._cameraMatrix && this._cameraMatrix.length >= 9) ? this._cameraMatrix[4] : this.focal;

        if (!Number.isFinite(fx) || !Number.isFinite(fy) || fx <= 0 || fy <= 0) {
            return position;
        }

        const corrected = position.clone();
        const rawDx = ((dxPx * depth) / fx) * this._centerLockStrength;
        const rawDy = (-(dyPx * depth) / fy) * this._centerLockStrength;
        const dx = Math.max(-this._centerLockMaxMeters, Math.min(this._centerLockMaxMeters, rawDx));
        const dy = Math.max(-this._centerLockMaxMeters, Math.min(this._centerLockMaxMeters, rawDy));
        corrected.x += dx;
        corrected.y += dy;
        return corrected;
    }

    // ── Render loop ──────────────────────────────────────────────────────────

    _loop() {
        if (this._stopped) return;
        this._rafId = requestAnimationFrame(() => this._loop());

        if (this.video.readyState < this.video.HAVE_ENOUGH_DATA) {
            this.renderer.render(this.scene, this.camera);
            return;
        }

        // Draw video frame on overlay
        this.overlayCtx.drawImage(this.video, 0, 0, this.overlay.width, this.overlay.height);

        const now = performance.now();

        // Keep 2D debug overlay visible each render frame (not only when worker returns).
        if (
            this._debugOverlayEnabled &&
            this._lastRawMarkers &&
            this._lastRawMarkers.length > 0 &&
            (now - this._lastRawMarkersTs) <= this._debugOverlayHoldMs
        ) {
            this._drawMarkerOverlay(this._lastRawMarkers);
        }

        this._updateRenderPose(now);

        // Throttle detection
        const interval = 1000 / Math.max(1, this._detectionFps);

        if (this.worker && !this._workerBusy && (now - this._lastDetectionTime) >= interval) {
            this._lastDetectionTime = now;
            this._workerBusy = true;

            const src = (this._detectionCanvas && this._detectionCanvas.width > 0)
                ? this._detectionCanvas : this.overlay;

            if (src !== this.overlay) {
                this._detectionCanvasCtx.clearRect(0, 0, src.width, src.height);
                this._detectionCanvasCtx.drawImage(this.overlay, 0, 0, src.width, src.height);
            }

            createImageBitmap(src).then(bmp => {
                const scale = src.width / this.overlay.width;
                let cm;
                if (this._cameraMatrix && this._cameraMatrix.length >= 9) {
                    const s = scale;
                    cm = [
                        this._cameraMatrix[0] * s, 0, this._cameraMatrix[2] * s,
                        0, this._cameraMatrix[4] * s, this._cameraMatrix[5] * s,
                        0, 0, 1
                    ];
                } else {
                    const fx = this.focal * scale;
                    cm = [fx, 0, src.width / 2, 0, fx, src.height / 2, 0, 0, 1];
                }

                try {
                    this.worker.postMessage({
                        type: 'frame',
                        bitmap: bmp,
                        cameraMatrix: cm,
                        distCoeffs: this._distCoeffs || [],
                        markerLength: this.markerSizeMM / 1000,
                        overlayWidth: this.overlay.width,
                        overlayHeight: this.overlay.height
                    }, [bmp]);
                } catch (err) {
                    this._workerBusy = false;
                    this.log('Worker postMessage fallito: ' + err.message, 'error');
                }
            }).catch(() => {
                this._workerBusy = false;
            });
        }

        this.renderer.render(this.scene, this.camera);
    }
}
