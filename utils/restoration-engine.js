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
        this._validMarkerIds = new Set([1]);
        this._singleMarkerMode = true;
        this._singleMarkerOffsetTemplate = null;
        this._detectionFps = 30;
        // Last raw markers received from worker (copy of data.markers)
        this._lastRawMarkers = [];
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
        this._measurementConfidenceEMA = 0.8;
        this._poolSpreadEMA = 0;

        // High-frequency render-space smoothing toward measurement target
        this._poseTargetPosition = null;
        this._poseTargetQuaternion = null;
        this._lastRenderTime = 0;
        this._renderPoseTau = 0.028;      // seconds, normal tracking
        this._renderPoseTauLocked = 0.032; // seconds, damping in lock mode (faster response)

        // Measurement-rate aware anti-shake (handles abrupt phone motion spikes)
        this._lastPoseMeasurementTime = 0;
        this._maxTrustedLinearRate = 3.0;   // m/s (used only under low-trust measurement)
        this._maxTrustedAngularRate = 5.2;  // rad/s
        this._lowTrustConfidenceThreshold = 0.58;
        this._lowTrustSpreadThreshold = 0.045;
        this._viewAngleEMA = 0;
        this._obliqueRejectAngleDeg = 84;    // reject near-grazing markers
        this._obliqueSoftLimitDeg = 70;      // above this, heavily downweight
        this._obliqueWeightFloor = 0.12;
        this._adaptiveTuningEnabled = true;
        this._debugOverlayEnabled = true;
        this._lastFusionStats = null;

        // EKF option for rotation smoothing (disabled by default)
        this._useQuatEKF = false;
        this._quatEKF = null;

        // First-pose flag: snap to first detected pose, then smooth after
        this._hasFirstPose = false;

        // Runtime tuning / detection helpers
        this._minMarkerPerimeter = 30;    // px - ignore tiny detections
        this._fusionAgreeDist = 0.14;     // meters - tighter marker agreement for stable fusion
        this._fusionTrackWindow = 0.24;   // meters - temporal gate around previous fused pose
        this._fusionPosSigma = 0.14;      // meters - distance weighting sigma for multi-marker blend
        this._singleMarkerMaxJump = 0.25; // meters - stricter guard when only one marker is usable (increased from 0.10 for better fast-motion tracking)
        this._maxPoseErrorForFusion = 0.35;
        this._trackingPosDeadband = 0.0035;  // meters, suppress micro-jitter when nearly static
        this._trackingRotDeadband = 0.020;   // radians, suppress tiny orientation shimmer
        this._anchorIds = null;           // null = use all markers; array -> prefer these ids
        this._anchorBoost = 3.0;          // weight multiplier for anchor IDs

        // WebXR / hit-test state (optional AR mode)
        this._xrActive = false;
        this._xrSession = null;
        this._xrHitTestSource = null;
        this._xrRefSpace = null;
        this._xrReticle = null;
        this._xrPlacementKey = 'expear.xrPlacement';

        // Short-term marker detection history (used for auto-anchor selection)
        this._markerHistory = [];
        this._markerHistorySize = 20; // frames to consider for stability scoring

        // Anchor-lock (stabilize model when marker is steady)
        this._anchorLockEnabled = false;          // disabled by default (was freezing model!)
        this._anchorLock = null;                  // { id, position: Vector3, quaternion: Quaternion }
        this._anchorAutoLockEnabled = false;      // disabled: auto-lock was the #1 cause of stuck model
        this._anchorAutoLockMinFrames = 12;       // require many more stable frames if enabled
        this._anchorAutoLockMaxPoseError = 0.06;  // stricter stability requirement
        this._anchorLockPositionTolerance = 0.15; // meters - wider tolerance before correction
        this._anchorLockRotationTolerance = 0.20; // radians - wider tolerance
        this._anchorLockKey = 'expear.anchorLock.v2';

        // ── World Anchor (pose lock) ──────────────────────────────────────
        // Once multiple markers consistently agree on the house pose,
        // the model is "locked" in world space.  Only ultra-slow drift
        // corrections are applied, so the house stands rock-solid while
        // the user orbits around it.
        this._worldAnchorActive = false;
        this._worldAnchorPos = null;       // THREE.Vector3
        this._worldAnchorQuat = null;      // THREE.Quaternion
        this._worldAnchorBuildup = 0;      // consecutive qualifying frames
        this._worldAnchorBuildupTarget = 6; // frames before lock engages
        this._worldAnchorMinMarkers = 2;   // markers needed to start buildup
        this._worldAnchorCorrectionAlpha = 0.018; // very slow drift correction
        this._worldAnchorRotCorrectionAlpha = 0.015;
        this._worldAnchorBreakDistance = 0.18; // meters – unlock if drift exceeds
        this._worldAnchorBreakAngle = Math.PI / 5; // radians – unlock on big rotation drift
        this._worldAnchorMaxAgreeDist = 0.08; // meters – markers must agree within this to buildup
        this._worldAnchorHoldPosEps = 0.006; // 6 mm: ignore tiny locked corrections
        this._worldAnchorHoldRotEps = 0.030; // ~1.7°: ignore tiny locked corrections
        this._worldAnchorSingleMarkerPosAlpha = 0.045; // less frozen under single marker
        this._worldAnchorSingleMarkerRotAlpha = 0.055;
        this._fastRepositionPosDelta = 0.032; // meters
        this._fastRepositionRotDelta = Math.PI / 12; // radians (~15°)
        this._fastRepositionUnlockRatio = 0.55; // of break threshold

        // Jump/clamp protection
        this._maxPositionJump = 0.5;      // meters (was 0.25, too aggressive)
        this._maxRotationJump = Math.PI/3; // radians (~60°)
        // Distance (meters) above which a single-marker pose is considered an outlier for fusion
        this._markerOutlierDistance = 0.35; // meters (can be tuned at runtime)
        // Minimum detector confidence (0..1) to accept a marker for fusion
        this._markerConfidenceThreshold = 0.15;
        // Corner-flow normalized SSD threshold (worker-side)
        this._cornerFlowSSDThreshold = 60;

        // Runtime-overridable marker offsets and persisted storage
        this._markerOffsets = null;
        this._markerOffsetsStorageKey = 'expear.markerOffsets.v4';
        this._autoOffsetsComputed = false;

        // Optional camera calibration (cameraMatrix [9], distCoeffs [])
        this._cameraMatrix = null;
        this._distCoeffs = null;

        // Optional features
        this._useSubpixel = false;
        this._useAprilTag = false;
        this._useSolvePnP = false;
        this._usePyrLKFlow = false;
        this._maxDetectSize = 768; // Increased from default to help with larger markers/details

        // If true, swap mapping of marker IDs 2/3 (Left/Right) — useful when physical markers are inverted
        this._swapLeftRight = false;

        this.restorationLevel = 0;
        this.onLog = null;
        this._stopped = false;
        this._rafId = null;
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
        const target = this.houseSizeMM / 1000;
        if (this.restoredModel)  this._scaleModel(this.restoredModel, target);
        if (this.destroyedModel) this._scaleModel(this.destroyedModel, target);
        this._recomputeBBox();
        this._repositionMarkerHelpers();
        this.log(`Casa: ${this.houseSizeMM} mm → ${target.toFixed(3)} m`);
    }

    updateDetectionFps(fps) {
        this._detectionFps = Math.max(1, Math.min(60, Number(fps) || 24));
    }

    // Anchor IDs: prefer these markers during fusion (pass array or null to reset)
    setAnchorIds(ids) {
        this._anchorIds = Array.isArray(ids) ? ids.map(Number) : null;
        // if explicitly clearing anchors, remove persisted value
        if (!this._anchorIds) try { localStorage.removeItem('expear.anchorIds'); } catch (e) { /* ignore */ }
        this.log('Anchor IDs set: ' + JSON.stringify(this._anchorIds));
    }

    // Anchor boost — increase weight for anchor IDs
    setAnchorBoost(v) { this._anchorBoost = Math.max(1, Number(v) || this._anchorBoost); this.log('Anchor boost set: ' + this._anchorBoost); }

    // Set the maximum distance (meters) used to detect outlier marker poses during fusion
    setMarkerOutlierDistanceMeters(m) {
        this._markerOutlierDistance = Math.max(0.01, Number(m) || this._markerOutlierDistance);
        this.log('Marker outlier distance set: ' + this._markerOutlierDistance + ' m');
    }

    // Set minimum detector confidence (0..1) below which markers are ignored for fusion
    setMarkerConfidenceThreshold(t) {
        this._markerConfidenceThreshold = Math.max(0, Math.min(1, Number(t) || this._markerConfidenceThreshold));
        this.log('Marker confidence threshold set: ' + this._markerConfidenceThreshold);
    }

    setAdaptiveTuningEnabled(enable) {
        this._adaptiveTuningEnabled = !!enable;
        this.log('Adaptive fusion tuning ' + (this._adaptiveTuningEnabled ? 'enabled' : 'disabled'));
    }

    setDebugOverlayEnabled(enable) {
        this._debugOverlayEnabled = !!enable;
        this.log('Debug overlay ' + (this._debugOverlayEnabled ? 'enabled' : 'disabled'));
    }

    // Set corner-flow normalized SSD threshold (worker-side). Also forwards to worker.
    setCornerFlowSSDThreshold(v) {
        this._cornerFlowSSDThreshold = Math.max(1, Number(v) || this._cornerFlowSSDThreshold);
        try { this.worker && this.worker.postMessage({ type: 'config', cornerFlowMaxNormalizedSSD: this._cornerFlowSSDThreshold }); } catch (e) { /* ignore */ }
        this.log('Corner-flow SSD threshold set: ' + this._cornerFlowSSDThreshold);
    }

    // Anchor-lock API disabled by project requirement (no lock behavior).
    setAnchorLockEnabled(_enable) {
        this._anchorLockEnabled = false;
        this._anchorLock = null;
        this.log('Anchor-lock disattivato (modalità no-lock)');
    }

    // ── World Anchor public API ──────────────────────────────────────────────

    /** Check whether the world anchor is currently active (locked state is forced off). */
    isWorldAnchorActive() { return false; }

    /** Manually force-lock disabled by project requirement. */
    forceWorldAnchorLock() {
        this._worldAnchorActive = false;
        this._worldAnchorBuildup = 0;
        this._worldAnchorPos = null;
        this._worldAnchorQuat = null;
        this.log('World Anchor disattivato (modalità no-lock)', 'warn');
        return false;
    }

    /** Manually unlock the world anchor so tracking resumes normally. */
    releaseWorldAnchor() {
        this._worldAnchorActive = false;
        this._worldAnchorBuildup = 0;
        this._worldAnchorPos = null;
        this._worldAnchorQuat = null;
        this.log('World Anchor disattivato');
    }

    /** Get world anchor state for UI / debug. */
    getWorldAnchorState() {
        return {
            active: this._worldAnchorActive,
            buildup: this._worldAnchorBuildup,
            target: this._worldAnchorBuildupTarget,
            pos: this._worldAnchorPos ? [this._worldAnchorPos.x, this._worldAnchorPos.y, this._worldAnchorPos.z] : null,
        };
    }

    // Force a lock from the current model transform (persisted by default)
    forceAnchorLockFromModel({ persist = true } = {}) {
        if (!this.modelGroup) return null;
        this._anchorLock = {
            id: Array.isArray(this._anchorIds) && this._anchorIds.length ? this._anchorIds[0] : null,
            position: this.modelGroup.position.clone(),
            quaternion: this.modelGroup.quaternion.clone(),
            createdAt: Date.now()
        };
        if (persist) this._saveAnchorLockToStorage();
        this.log('Anchor-lock creato da modello corrente' + (this._anchorLock.id != null ? ' (marker ' + this._anchorLock.id + ')' : ''));
        return this.getAnchorLockState();
    }

    // Clear stored anchor-lock
    clearAnchorLock({ persist = true } = {}) {
        this._anchorLock = null;
        if (persist) try { localStorage.removeItem(this._anchorLockKey); } catch (e) { /* ignore */ }
        this.log('Anchor-lock cancellato');
    }

    // Return a plain JS snapshot of the current anchor-lock state
    getAnchorLockState() {
        if (!this._anchorLock) return null;
        return {
            id: this._anchorLock.id,
            pos: [this._anchorLock.position.x, this._anchorLock.position.y, this._anchorLock.position.z],
            quat: [this._anchorLock.quaternion.x, this._anchorLock.quaternion.y, this._anchorLock.quaternion.z, this._anchorLock.quaternion.w],
            createdAt: this._anchorLock.createdAt
        };
    }

    setAnchorAutoLockEnabled(_enable) {
        this._anchorAutoLockEnabled = false;
        this.log('Anchor auto-lock disattivato (modalità no-lock)');
    }

    // Auto-select the most stable marker seen over the short-term history
    autoSelectAnchor({ persist = true, minPresenceFraction = 0.25 } = {}) {
        if (!this._markerHistory || this._markerHistory.length === 0) {
            // fallback: pick best marker from lastRawMarkers by perimeter/poseError
            const cand = (this._lastRawMarkers || []).slice().sort((a, b) => {
                const pa = (a.poseError || 0), pb = (b.poseError || 0);
                const perA = this._markerPerimeter(a.corners || []), perB = this._markerPerimeter(b.corners || []);
                if (perB !== perA) return perB - perA;
                return (pa - pb);
            })[0];
            if (!cand) { this.log('autoSelectAnchor: no candidate', 'warn'); return null; }
            this.setAnchorIds([cand.id]);
            if (persist) try { localStorage.setItem('expear.anchorIds', JSON.stringify([cand.id])); } catch (e) { /* ignore */ }
            this.log('Auto-selected anchor (single-frame): ' + cand.id);
            return cand.id;
        }

        const frames = this._markerHistory.length;
        const counts = Object.create(null);
        const sumPerim = Object.create(null);
        const sumErr = Object.create(null);

        for (const frame of this._markerHistory) {
            for (const idStr in frame) {
                const id = Number(idStr);
                const entry = frame[idStr];
                counts[id] = (counts[id] || 0) + 1;
                sumPerim[id] = (sumPerim[id] || 0) + (entry.perimeter || 0);
                sumErr[id] = (sumErr[id] || 0) + (entry.poseError || 0);
            }
        }

        let bestId = null, bestScore = -Infinity;
        for (const idStr in counts) {
            const id = Number(idStr);
            const cnt = counts[id];
            const presence = cnt / frames;
            if (presence < minPresenceFraction || cnt < 2) continue; // require some presence
            const avgPerim = sumPerim[id] / cnt;
            const avgErr = sumErr[id] / cnt;
            const score = presence * Math.log(1 + avgPerim) / (1 + avgErr * 5);
            if (score > bestScore) { bestScore = score; bestId = id; }
        }

        if (bestId === null) {
            this.log('autoSelectAnchor: no stable candidate', 'warn');
            return null;
        }

        this.setAnchorIds([bestId]);
        if (persist) try { localStorage.setItem('expear.anchorIds', JSON.stringify([bestId])); } catch (e) { /* ignore */ }
        this.log('Auto-selected anchor: ' + bestId);
        return bestId;
    }

    _loadAnchorFromStorage() {
        try {
            const raw = localStorage.getItem('expear.anchorIds');
            if (!raw) return;
            const parsed = JSON.parse(raw);
            if (Array.isArray(parsed)) this.setAnchorIds(parsed);
        } catch (e) { /* ignore */ }
    }

    _saveAnchorLockToStorage() {
        try {
            if (!this._anchorLock) { localStorage.removeItem(this._anchorLockKey); return; }
            const o = this._anchorLock;
            localStorage.setItem(this._anchorLockKey, JSON.stringify({ id: o.id, pos: [o.position.x, o.position.y, o.position.z], quat: [o.quaternion.x, o.quaternion.y, o.quaternion.z, o.quaternion.w], createdAt: o.createdAt }));
            this.log('Anchor-lock salvato');
        } catch (e) { /* ignore */ }
    }

    _loadAnchorLockFromStorage() {
        if (!this._anchorLockEnabled) return; // don't load stale locks when feature is disabled
        try {
            const raw = localStorage.getItem(this._anchorLockKey);
            if (!raw) return;
            const p = JSON.parse(raw);
            if (!p || !p.pos || !p.quat) return;
            this._anchorLock = {
                id: (typeof p.id !== 'undefined') ? p.id : null,
                position: new THREE.Vector3(p.pos[0], p.pos[1], p.pos[2]),
                quaternion: new THREE.Quaternion(p.quat[0], p.quat[1], p.quat[2], p.quat[3]),
                createdAt: p.createdAt || Date.now()
            };
            this.log('Anchor-lock caricata da storage');
        } catch (e) { /* ignore */ }
    }

    // Position history buffer size (median filter)
    setPositionHistorySize(n) {
        this._positionHistorySize = Math.max(1, Math.min(15, Number(n) || this._positionHistorySize));
        this.log('Position history size: ' + this._positionHistorySize);
        if (this._positionHistory && this._positionHistory.length > this._positionHistorySize) {
            this._positionHistory = this._positionHistory.slice(-this._positionHistorySize);
        }
    }

    // Worker corner smoothing control (0..1)
    setCornerSmoothing(v) {
        const val = Math.max(0, Math.min(1, Number(v) || 0));
        try { this.worker && this.worker.postMessage({ type: 'config', cornerSmoothing: val }); } catch (e) { /* ignore */ }
        this.log('Corner smoothing set: ' + val);
    }

    // Enable/disable lightweight corner-flow stabilization in the worker
    setCornerFlowEnabled(enable) {
        try { this.worker && this.worker.postMessage({ type: 'config', cornerFlowEnabled: !!enable }); } catch (e) { /* ignore */ }
        this.log('Corner flow ' + (enable ? 'enabled' : 'disabled'));
    }

    // Configure corner-flow params: radius (px), templateSize (px)
    setCornerFlowParams({ radius, templateSize } = {}) {
        try {
            const cfg = {};
            if (typeof radius === 'number') cfg.cornerFlowRadius = Math.max(1, Math.min(32, Math.floor(radius)));
            if (typeof templateSize === 'number') cfg.cornerFlowTemplate = Math.max(3, Math.min(33, Math.floor(templateSize)));
            if (Object.keys(cfg).length && this.worker) this.worker.postMessage(Object.assign({ type: 'config' }, cfg));
            this.log('Corner flow params updated: ' + JSON.stringify(cfg));
        } catch (e) { /* ignore */ }
    }

    // Sub-pixel refinement control (uses OpenCV.js inside worker when enabled)
    setUseSubpixel(enable) {
        this._useSubpixel = !!enable;
        try { this.worker && this.worker.postMessage({ type: 'config', useSubpixel: !!enable }); } catch (e) { /* ignore */ }
        this.log('Sub-pixel refinement ' + (enable ? 'enabled' : 'disabled'));
    }

    // Prefer solvePnP (OpenCV) over POSIT when OpenCV is available in the worker
    setUseSolvePnP(enable) {
        this._useSolvePnP = !!enable;
        try { this.worker && this.worker.postMessage({ type: 'config', useSolvePnP: !!enable }); } catch (e) { /* ignore */ }
        this.log('Use solvePnP ' + (enable ? 'ON' : 'OFF'));
    }

    // Prefer calcOpticalFlowPyrLK (OpenCV) for corner tracking when available
    setUsePyrLKFlow(enable) {
        this._usePyrLKFlow = !!enable;
        try { this.worker && this.worker.postMessage({ type: 'config', usePyrLKFlow: !!enable }); } catch (e) { /* ignore */ }
        this.log('Use PyrLK flow ' + (enable ? 'ON' : 'OFF'));
    }

    // Enable/disable AprilTag detection fallback in worker
    setUseAprilTag(enable) {
        this._useAprilTag = !!enable;
        try { this.worker && this.worker.postMessage({ type: 'config', useAprilTag: !!enable }); } catch (e) { /* ignore */ }
        this.log('AprilTag fallback ' + (enable ? 'enabled' : 'disabled'));
    }

    // Swap left/right mapping for marker IDs 2/3 (useful when physical markers are inverted)
    setSwapLeftRight(enable) {
        this._swapLeftRight = !!enable;
        try { this._repositionMarkerHelpers(); } catch (e) { /* ignore */ }
        this.log('Swap Left/Right mapping ' + (this._swapLeftRight ? 'ON' : 'OFF'));
    }

    setSubpixelParams({ win, maxIter, eps } = {}) {
        try {
            const cfg = {};
            if (typeof win === 'number') cfg.subpixWin = Math.max(3, Math.min(31, Math.floor(win)));
            if (typeof maxIter === 'number') cfg.subpixMaxIter = Math.max(1, Math.min(200, Math.floor(maxIter)));
            if (typeof eps === 'number') cfg.subpixEPS = Math.max(0.0, Math.min(10.0, Number(eps)));
            if (Object.keys(cfg).length && this.worker) this.worker.postMessage(Object.assign({ type: 'config' }, cfg));
            this.log('Sub-pixel params updated: ' + JSON.stringify(cfg));
        } catch (e) { /* ignore */ }
    }

    // Enable/disable Quaternion EKF
    setUseQuaternionEKF(enable) {
        this._useQuatEKF = !!enable;
        if (this._useQuatEKF && !this._quatEKF) {
            this._quatEKF = new PoseFilters.QuaternionEKF({
                qInit: (this.modelGroup && this.modelGroup.quaternion)
                    ? this.modelGroup.quaternion.clone()
                    : new THREE.Quaternion()
            });
        }
        this.log('Quaternion EKF ' + (this._useQuatEKF ? 'enabled' : 'disabled'));
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
                filter: { positionSmoothing: 0.10, rotationTimeConstant: 0.085 },
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
                filter: { positionSmoothing: 0.12, rotationTimeConstant: 0.10 },
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
                filter: { positionSmoothing: 0.05, rotationTimeConstant: 0.05 },
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

    /**
     * Scale a loaded FBX so its largest dimension equals `targetMeters`.
     * Resets scale/position first so repeated calls are idempotent.
     */
    _scaleModel(object, targetMeters) {
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

        // Re-centre: pivot at bottom-centre
        object.updateMatrixWorld(true);
        const box2 = new THREE.Box3().setFromObject(object);
        const centre = new THREE.Vector3();
        box2.getCenter(centre);
        object.position.sub(centre);
        object.position.y += (box2.max.y - box2.min.y) / 2;
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

        // Shadow ground plane
        const shadowPlane = new THREE.Mesh(
            new THREE.PlaneGeometry(2, 2),
            new THREE.ShadowMaterial({ opacity: 0.25 })
        );
        shadowPlane.rotation.x = -Math.PI / 2;
        shadowPlane.position.y = -0.001;
        shadowPlane.receiveShadow = true;
        this.modelGroup.add(shadowPlane);

        // Load persisted marker offsets, anchor and anchor-lock (if any), then create helpers
        this._loadMarkerOffsetsFromStorage();
        this._loadAnchorFromStorage();
        this._loadAnchorLockFromStorage();
        this._createMarkerHelpers();

        // Load FBX models
        this._loadModels();

        // Pose filters — use PredictivePositionFilter for velocity extrapolation
        // between detection frames. Much more responsive than AdaptivePositionFilter.
        this.posFilter = new PoseFilters.PredictivePositionFilter({
            responsiveness: 0.92,
            velocitySmoothing: 0.15,
            predictionFactor: 0.2,
            maxVelocity: 1.0,
            maxPredictionDt: 0.033,
            maxPredictionStep: 0.015,
            velocityDamping: 0.9,
            positionDeadband: 0.0015
        });
        this.quatFilter = new PoseFilters.QuaternionFilter({ timeConstant: 0.045 });
    }

    // ── FBX loading ──────────────────────────────────────────────────────────

    _loadModels() {
        const loader = new THREE.FBXLoader();
        // global multiplier from UI slider (1.0 default)
        const mult = parseFloat(window.houseScaleMultiplier) || 1.0;
        const targetSize = (this.houseSizeMM / 1000) * mult;

        // Helper: shared material + scaling + bbox recompute
        const onLoaded = (object, opts) => {
            // Some FBX exports use Z-up; rotate to Y-up so house base lies flat.
            object.rotateX(-Math.PI/2);
            object.updateMatrixWorld(true);

            this._scaleModel(object, targetSize);
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
        this.worker = new Worker('workers/aruco-worker.js');
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

        // Both OpenCV and js-aruco2 POSIT use: X right, Y down, Z forward
        // Three.js uses: X right, Y up, Z backward
        // Transformation: F * R * F where F = diag(1, -1, -1)
        return {
            position: new THREE.Vector3(tvec[0], -tvec[1], -tvec[2]),
            quaternion: new THREE.Quaternion(q.x, -q.y, -q.z, q.w)
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

        if (this._debugOverlayEnabled && rawMarkers.length > 0) {
            this._drawMarkerOverlay(this._lastRawMarkers);
        }

        const poseful = validMarkers.filter(m => m.rvec && m.tvec);
        if (poseful.length > 0) {
            this._applyTrackedPose(poseful, statusEl, now);
            return;
        }
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

        // In single marker mode, we just snap to the target pose to avoid trailing
        this.modelGroup.position.copy(this._poseTargetPosition);
        this.modelGroup.quaternion.copy(this._poseTargetQuaternion);
    }

    /**
     * Single-marker tracking.
     */
    _applyTrackedPose(poseful, statusEl, now) {
        this._lastTrackingTime = now;
        this._framesWithoutDetection = 0;

        // single marker only, ignore orientation entirely
        const m = poseful[0];
        const { position } = this._poseToThreeJs(m.rvec, m.tvec, m.source);

        this.modelGroup.position.copy(position);
        this.modelGroup.quaternion.set(0, 0, 0, 1);
        this._poseTargetPosition = position.clone();
        this._poseTargetQuaternion = new THREE.Quaternion();


        this.isTracking = true;
        this.modelGroup.visible = true;

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
        const dist = this.modelGroup.position.length();
        statusEl.textContent = `ID ${m.id} · ${dist.toFixed(2)} m`;
        statusEl.style.color = 'var(--p-gold)';
        statusEl.classList.add('tracking');
    }

    /** Handle lost-tracking with hysteresis timeout. */
    _handleTrackingLost(statusEl, now) {
        this._framesWithoutDetection++;

        // Tolerate a few dropped frames (hysteresis) before hiding the model.
        // Mobile cameras often drop frames due to motion blur or autofocus.
        // 10 frames is about 300ms at 30fps, enough to prevent rapid flickering.
        if (this._framesWithoutDetection >= 10) {
            this._positionHistory = [];
            this._poseTargetPosition = null;
            this._poseTargetQuaternion = null;
            this._lastRenderTime = 0;
            this._lastPoseMeasurementTime = 0;

            this.isTracking = false;
            if (this.modelGroup) this.modelGroup.visible = false;
            this._hasFirstPose = false;
            statusEl.textContent = 'RICERCA TARGET...';
            statusEl.style.color = 'var(--p-dim)';
            statusEl.classList.remove('tracking');
        }
    }

    // ── Overlay drawing ──────────────────────────────────────────────────────

    _drawMarkerOverlay(markers) {
        const ctx = this.overlayCtx;
        // Clear previous marker drawings (video frame is re-drawn in _loop before this)
        // We only clear the marker annotation layer — video background is handled by drawImage in _loop
        const palette = ['#00ff88', '#00ccff', '#ffaa00', '#ff44aa'];
        const markerLabels = {
            1: 'BL', 7: 'BC', 2: 'BR',
            5: 'ML', 6: 'MR',
            3: 'FL', 8: 'FC', 4: 'FR'
        };

        for (const m of markers) {
            const c = m.corners;
            if (!c || c.length < 4) continue;
            
            // Check isValid property if available (added in recent update) or default to checking valid IDs
            const hasIdFilter = this._validMarkerIds instanceof Set && this._validMarkerIds.size > 0;
            const isValid = (typeof m.isValid === 'boolean') ? m.isValid : (hasIdFilter ? this._validMarkerIds.has(Number(m.id)) : true);
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
                }
            }

            // ID Label
            const cx = (c[0][0] + c[2][0]) / 2;
            const cy = (c[0][1] + c[2][1]) / 2;
            
            ctx.font = isValid ? 'bold 12px monospace' : '10px monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            const label = (markerLabels[m.id] || '') + ' #' + m.id + (isValid ? '' : ' ERR');
            
            ctx.fillStyle = 'rgba(0,0,0,0.6)';
            const metrics = ctx.measureText(label);
            ctx.fillRect(cx - metrics.width/2 - 2, cy - 8, metrics.width + 4, 16);
            
            ctx.fillStyle = isValid ? '#ffffff' : '#ffcccc';
            ctx.fillText(label, cx, cy);

            // draw prominent 3D axes (X=Red, Y=Green, Z=Blue) on the marker itself
            if (isValid && m.rvec && m.tvec) {
                // We need to project the 3D axes from the marker's local space to screen space.
                // We can use the already computed modelViewMatrix or just project manually.
                // Since we don't have easy access to the full MVP here without re-computing, 
                // let's use a simpler approach: use the unit vectors from the rotation matrix.
                // But wait, we have rvec/tvec. We can use OpenCV projectPoints equivalent or just reuse the model projection logic?
                // Actually, let's just use the 2D corners for X and Y, and cross product for Z approximation for visual feedback.
                // True 3D projection is better. We can use the `_drawProjectedModelAxes` logic adapted for per-marker.
                
                // Let's rely on the model axes for the main alignment interacting with the user's "red rectangle".
                // But to help debug "fatica a leggere", let's make the outline THICKER and brighter.
                ctx.lineWidth = 4;
                ctx.strokeStyle = isValid ? '#00ff00' : '#ff0000';
                ctx.stroke();

                // Draw X/Y axes based on corners (approximation of 3D orientation)
                const c0 = c[0]; // Top-Left (in ArUco standard)
                const c1 = c[1]; // Top-Right
                const c3 = c[3]; // Bottom-Left
                
                ctx.beginPath();
                ctx.lineWidth = 3;
                ctx.strokeStyle = '#ff0000'; // X Axis (Right)
                ctx.moveTo(c0[0], c0[1]);
                ctx.lineTo(c1[0], c1[1]);
                ctx.stroke();

                ctx.beginPath();
                ctx.strokeStyle = '#00ff00'; // Y Axis (Down/Forward in 2D) - actually usually Z in 3D but let's visualise the plane
                ctx.moveTo(c0[0], c0[1]);
                ctx.lineTo(c3[0], c3[1]);
                ctx.stroke();

                // Draw center cross
                ctx.beginPath();
                ctx.arc(cx, cy, 5, 0, Math.PI * 2);
                ctx.fillStyle = '#ffff00';
                ctx.fill();
            }

            // Add distance for valid markers 
            if (isValid && m.tvec && m.tvec.length === 3) {
                const dist = Math.hypot(m.tvec[0], m.tvec[1], m.tvec[2]);
                ctx.fillStyle = '#ddd';
                ctx.font = '10px monospace';
                ctx.fillText(dist.toFixed(2) + ' m', cx, cy + 12);
            }
        }
    }

    _drawDebugHud() {
        if (!this._debugOverlayEnabled || !this.overlayCtx) return;
        const ctx = this.overlayCtx;
        const stats = this._lastFusionStats;
        const w = this.overlay?.width || 0;
        if (!w) return;

        const lines = [];
        // Show detected IDs explicitly in HUD
        const detectedIds = (this._lastRawMarkers || []).map(m => m.id).join(',');
        lines.push(`trk:${this.isTracking ? 'ON' : 'OFF'} IDs:[${detectedIds}] lock:${this._worldAnchorActive ? 'ON' : 'OFF'}`);
        if (stats) {
            lines.push(`pool ${stats.poolSize}/${stats.candidateSize} conf ${stats.avgConfidence.toFixed(2)} spread ${(stats.spread * 1000).toFixed(0)}mm`);
            lines.push(`view ${Math.round(stats.viewAngleDeg)}° th ${stats.adaptiveConfidenceThreshold.toFixed(2)} out ${stats.adaptiveOutlierDistance.toFixed(2)}m`);
            lines.push(`tw ${stats.adaptiveTrackWindow.toFixed(2)} soft/rej ${Math.round(stats.adaptiveObliqueSoftLimitDeg)}°/${Math.round(stats.adaptiveObliqueRejectDeg)}° ${stats.adaptiveEnabled ? 'AT' : 'FIX'}`);
        }

        const x = 12;
        const y = 12;
        const lineH = 14;
        const boxH = 10 + lines.length * lineH;
        const boxW = 360;

        ctx.save();
        ctx.fillStyle = 'rgba(0,0,0,0.55)';
        ctx.fillRect(x - 6, y - 4, boxW, boxH);
        ctx.font = '11px monospace';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillStyle = '#d8f7ff';
        for (let i = 0; i < lines.length; i++) {
            ctx.fillText(lines[i], x, y + i * lineH);
        }
        ctx.restore();

        this._drawProjectedModelAxes(ctx);
    }

    _drawProjectedModelAxes(ctx) {
        if (!this.modelGroup || !this.modelGroup.visible || !this.camera || !this.overlay) return;

        const w = this.overlay.width;
        const h = this.overlay.height;
        if (!w || !h) return;

        const origin = new THREE.Vector3();
        const worldQ = new THREE.Quaternion();
        this.modelGroup.getWorldPosition(origin);
        this.modelGroup.getWorldQuaternion(worldQ);

        const axisLen = Math.max(0.04, (this.markerSizeMM / 1000) * 0.45);
        const worldX = origin.clone().add(new THREE.Vector3(axisLen, 0, 0).applyQuaternion(worldQ));
        const worldY = origin.clone().add(new THREE.Vector3(0, axisLen, 0).applyQuaternion(worldQ));
        const worldZ = origin.clone().add(new THREE.Vector3(0, 0, axisLen).applyQuaternion(worldQ));

        const projectToScreen = (worldPos) => {
            const p = worldPos.clone().project(this.camera);
            if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) return null;
            return {
                x: (p.x * 0.5 + 0.5) * w,
                y: (-p.y * 0.5 + 0.5) * h
            };
        };

        const sO = projectToScreen(origin);
        const sX = projectToScreen(worldX);
        const sY = projectToScreen(worldY);
        const sZ = projectToScreen(worldZ);
        if (!sO || !sX || !sY || !sZ) return;

        ctx.save();
        ctx.lineWidth = 2;

        ctx.strokeStyle = '#ff3333'; // X axis
        ctx.beginPath();
        ctx.moveTo(sO.x, sO.y);
        ctx.lineTo(sX.x, sX.y);
        ctx.stroke();

        ctx.strokeStyle = '#33ff66'; // Y axis
        ctx.beginPath();
        ctx.moveTo(sO.x, sO.y);
        ctx.lineTo(sY.x, sY.y);
        ctx.stroke();

        ctx.strokeStyle = '#44aaff'; // Z axis
        ctx.beginPath();
        ctx.moveTo(sO.x, sO.y);
        ctx.lineTo(sZ.x, sZ.y);
        ctx.stroke();

        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(sO.x, sO.y, 3.5, 0, Math.PI * 2);
        ctx.fill();

        ctx.font = '10px monospace';
        ctx.fillStyle = '#ffffff';
        ctx.fillText('pivot', sO.x + 6, sO.y - 6);

        ctx.fillStyle = '#ff3333';
        ctx.fillText('X', sX.x + 3, sX.y + 3);
        ctx.fillStyle = '#33ff66';
        ctx.fillText('Y', sY.x + 3, sY.y + 3);
        ctx.fillStyle = '#44aaff';
        ctx.fillText('Z', sZ.x + 3, sZ.y + 3);
        ctx.restore();
    }

    // ── Render loop ──────────────────────────────────────────────────────────

    _loop() {
        if (this._stopped) return;
        this._rafId = requestAnimationFrame(() => this._loop());

        // When an XR session is active the XR rendering loop takes over — skip the non‑XR worker/render flow.
        if (this._xrActive) return;

        if (this.video.readyState < this.video.HAVE_ENOUGH_DATA) {
            this.renderer.render(this.scene, this.camera);
            return;
        }

        // Draw video frame on overlay
        this.overlayCtx.drawImage(this.video, 0, 0, this.overlay.width, this.overlay.height);

        const now = performance.now();

        this._updateRenderPose(now);
        this._drawDebugHud();

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

    // Public API: return last raw markers (copy)
    getLastRawMarkers() {
        return (this._lastRawMarkers || []).map(m => ({ id: m.id, rvec: m.rvec ? m.rvec.slice() : null, tvec: m.tvec ? m.tvec.slice() : null, poseError: m.poseError, distance: m.distance }));
    }

    // ── Screenshot ───────────────────────────────────────────────────────────

    async captureScreenshot() {
        const w = this.overlay.width, h = this.overlay.height;
        const canvas = document.createElement('canvas');
        canvas.width = w; canvas.height = h;
        const ctx = canvas.getContext('2d');
        try { ctx.drawImage(this.video, 0, 0, w, h); } catch (e) { this.log('Screenshot video: ' + e.message, 'warn'); }
        try { ctx.drawImage(this.renderer.domElement, 0, 0, w, h); } catch (e) { this.log('Screenshot 3d: ' + e.message, 'warn'); }
        try { ctx.drawImage(this.overlay, 0, 0, w, h); } catch (e) { this.log('Screenshot overlay: ' + e.message, 'warn'); }
        return canvas.toDataURL('image/png');
    }
}
