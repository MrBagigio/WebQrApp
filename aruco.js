/**
 * aruco.js — AAA-grade ArUco + Three.js AR Overlay Engine
 *
 * Complete rewrite: fixes broken syntax, adds proper AR projection,
 * coordinate conversion, responsive filtering, and fluid tracking.
 *
 * Architecture:
 *   1. Video feed as background
 *   2. Three.js canvas overlaid WITH transparent background
 *   3. Projection matrix derived from camera intrinsics
 *   4. Render loop decoupled from detection (60fps render, 30fps detect)
 *   5. Velocity-based prediction for inter-frame smoothness
 *   6. Graceful marker-loss fading
 */

// ═══════════════════════════════════════════════════════════════
// MODULE STATE
// ═══════════════════════════════════════════════════════════════
let video = null;
let overlay = null;         // 2D canvas for debug marker outlines
let overlayCtx = null;
let streaming = false;
let srcMat = null;
let detectorParams = null;
let dictionary = null;

// Three.js core
let renderer = null;
let scene = null;
let camera3 = null;
let model = null;

// Camera intrinsics (updated when video starts)
let videoWidth = 640;
let videoHeight = 480;

// UI controls
let markerSizeInput = null;
let focalInput = null;
let markerSizeMM = 50;
let focal = 800;

// ===== Filters (MODULE-scoped — accessible everywhere) =====
let posFilter = null;
let quatFilter = null;
let quatEKF = null;

// ===== Kalman (MODULE-scoped) =====
let kalman = null;
let rotationKalman = { x: null, y: null, z: null };

// ===== Timing / Detection =====
let detectionIntervalMs = 33;   // ~30fps detection (was 100ms!)
let _lastDetectionTime = 0;
let _lastPoseTime = 0;

// ===== Marker tracking state =====
let _lastMarkerSeen = 0;
let _markerVisible = false;
let _modelOpacity = 0;
const MARKER_FADE_MS = 300;     // Start fading after no detection for this long
const MARKER_HIDE_MS = 800;     // Fully hide model after this long

// Render loop
let _lastRenderTime = 0;
let prevPose = null;

// Scale factor (user-adjustable via UI)
let userScaleFactor = 2.0;
// Cached scale to avoid recomputing every frame
let _cachedScale = null;
let _cachedScaleMarkerMM = null;

// ═══════════════════════════════════════════════════════════════
// KALMAN FILTERS
// ═══════════════════════════════════════════════════════════════
class KalmanFilter3D {
  constructor({ R = 0.005, Q = 0.002, initial = { x: 0, y: 0, z: 0 } } = {}) {
    this.R = R;
    this.Q = Q;
    this.x = [initial.x, initial.y, initial.z];
    this.P = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  }
  update(measurement) {
    for (let i = 0; i < 3; i++) {
      const z = measurement[i];
      const K = this.P[i][i] / (this.P[i][i] + this.R);
      this.x[i] += K * (z - this.x[i]);
      this.P[i][i] = (1 - K) * this.P[i][i] + this.Q;
    }
    return { x: this.x[0], y: this.x[1], z: this.x[2] };
  }
  setState(s) { this.x = [s.x, s.y, s.z]; }
}

class KalmanFilter1D {
  constructor({ R = 0.01, Q = 0.003, initial = 0 } = {}) {
    this.R = R; this.Q = Q; this.x = initial; this.P = 1;
  }
  update(z) {
    const K = this.P / (this.P + this.R);
    this.x += K * (z - this.x);
    this.P = (1 - K) * this.P + this.Q;
    return this.x;
  }
  setState(v) { this.x = v; }
}

function ensureKalman() {
  if (!kalman) kalman = new KalmanFilter3D({ R: 0.005, Q: 0.002 });
  if (!rotationKalman.x) rotationKalman.x = new KalmanFilter1D({ R: 0.01, Q: 0.003 });
  if (!rotationKalman.y) rotationKalman.y = new KalmanFilter1D({ R: 0.01, Q: 0.003 });
  if (!rotationKalman.z) rotationKalman.z = new KalmanFilter1D({ R: 0.01, Q: 0.003 });
}

// ═══════════════════════════════════════════════════════════════
// PRESET HELPERS
// ═══════════════════════════════════════════════════════════════
function loadPresets() {
  try { return JSON.parse(localStorage.getItem('aruco_presets') || '{}'); } catch (e) { return {}; }
}
function savePresets(obj) {
  try { localStorage.setItem('aruco_presets', JSON.stringify(obj)); } catch (e) {}
}
function refreshPresetList() {
  const sel = document.getElementById('preset-list');
  if (!sel) return;
  sel.innerHTML = '';
  const presets = loadPresets();
  for (const name of Object.keys(presets)) {
    const opt = document.createElement('option');
    opt.value = name; opt.textContent = name;
    sel.appendChild(opt);
  }
}

// ═══════════════════════════════════════════════════════════════
// AR PROJECTION MATH
// ═══════════════════════════════════════════════════════════════

/**
 * Build a THREE.Matrix4 projection matrix from OpenCV camera intrinsics.
 * This is the KEY to correct AR overlay — the 3D projection matches the
 * physical camera so 3D objects align with the video feed.
 *
 * @param {number} fx  Focal length X (pixels)
 * @param {number} fy  Focal length Y (pixels)
 * @param {number} cx  Principal point X (pixels)
 * @param {number} cy  Principal point Y (pixels)
 * @param {number} w   Image width
 * @param {number} h   Image height
 * @param {number} near Near clip plane
 * @param {number} far  Far clip plane
 * @returns {THREE.Matrix4}
 */
function buildProjectionMatrix(fx, fy, cx, cy, w, h, near, far) {
  // OpenCV → OpenGL/Three.js NDC projection
  // Maps camera-space coords to clip-space correctly
  return new THREE.Matrix4().set(
    2 * fx / w,  0,            -(2 * cx / w - 1),              0,
    0,           2 * fy / h,   (2 * cy / h - 1),               0,
    0,           0,            -(far + near) / (far - near),    -2 * far * near / (far - near),
    0,           0,            -1,                               0
  );
}

/**
 * Convert OpenCV pose (rvec/tvec) to Three.js position + quaternion.
 *
 * OpenCV camera coords: X-right, Y-down, Z-forward
 * Three.js camera coords: X-right, Y-up, Z-backward
 *
 * Transform: x' = x, y' = -y, z' = -z
 * For rotation: R' = F · R · F  where F = diag(1,-1,-1)
 */
function cvPoseToThree(rvec, tvec) {
  // Position: flip Y and Z
  const position = new THREE.Vector3(tvec[0], -tvec[1], -tvec[2]);

  // Rotation: rvec → rotation matrix → flip YZ → quaternion
  const angle = Math.sqrt(rvec[0] ** 2 + rvec[1] ** 2 + rvec[2] ** 2);
  let rotMat = new THREE.Matrix4();
  if (angle > 1e-6) {
    const axis = new THREE.Vector3(rvec[0] / angle, rvec[1] / angle, rvec[2] / angle);
    rotMat.makeRotationAxis(axis, angle);
  }

  // Apply coordinate flip: R_three = F * R_cv * F
  const F = new THREE.Matrix4().set(
    1,  0,  0, 0,
    0, -1,  0, 0,
    0,  0, -1, 0,
    0,  0,  0, 1
  );
  rotMat = new THREE.Matrix4().multiplyMatrices(F, rotMat).multiply(F);

  const quaternion = new THREE.Quaternion().setFromRotationMatrix(rotMat);
  return { position, quaternion };
}

/**
 * Compute proper model scale based on bounding box normalization.
 * Ensures the model visually matches the physical marker size.
 *
 * @param {THREE.Object3D} targetModel
 * @param {number} markerLengthM  Marker side length in meters
 * @param {number} scaleFactor    User multiplier (1.0 = same size as marker)
 * @returns {number} Uniform scale to apply
 */
function computeModelScale(targetModel, markerLengthM, scaleFactor) {
  if (!targetModel) return 1.0;

  // Temporarily reset scale to measure true geometry size
  const saved = targetModel.scale.clone();
  targetModel.scale.set(1, 1, 1);
  targetModel.updateMatrixWorld(true);

  const bbox = new THREE.Box3().setFromObject(targetModel);
  const size = new THREE.Vector3();
  bbox.getSize(size);

  // Restore
  targetModel.scale.copy(saved);

  const maxDim = Math.max(size.x, size.y, size.z, 0.001);
  return (markerLengthM * scaleFactor) / maxDim;
}

// ═══════════════════════════════════════════════════════════════
// OPENCV ENTRYPOINT
// ═══════════════════════════════════════════════════════════════
function onOpenCvReady() {
  console.log('OpenCV.js loaded');
  initPOC();
}
window.onOpenCvReady = onOpenCvReady;

// Ensure worker toggle is available even if OpenCV fails to load
document.addEventListener('DOMContentLoaded', () => {
  try { setupWorkerToggle(); } catch (e) {}
});

// ═══════════════════════════════════════════════════════════════
// POC INITIALIZATION
// ═══════════════════════════════════════════════════════════════
async function initPOC() {
  video = document.getElementById('video');
  overlay = document.getElementById('overlay');
  overlayCtx = overlay.getContext('2d');

  markerSizeInput = document.getElementById('marker-size');
  focalInput = document.getElementById('focal');
  markerSizeMM = Number(markerSizeInput.value || 50);
  focal = Number(focalInput.value || 800);

  // Setup worker toggle early (tests need it)
  setupWorkerToggle();

  // Calibrate button
  document.getElementById('calibrate').addEventListener('click', () => {
    localStorage.setItem('aruco_marker_size', markerSizeInput.value);
    localStorage.setItem('aruco_focal', focalInput.value);
    alert('Calibrazione salvata');
  });

  document.getElementById('download-marker').addEventListener('click', () => downloadMarker(0));

  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: 'environment',
        width: { ideal: 1280 },
        height: { ideal: 720 }
      }
    });
    video.srcObject = stream;
    await video.play();
    streaming = true;
    videoWidth = video.videoWidth;
    videoHeight = video.videoHeight;

    setupCanvas();
    await initOpenCVStuff();
    initThree();

    // Re-init worker after Three is ready
    setupWorkerToggle();

    // Start the unified render/detect loop
    _lastRenderTime = performance.now();
    requestAnimationFrame(mainLoop);
  } catch (e) {
    console.error('Camera error', e);
    
    // Fallback for local testing without HTTPS/Camera permissions
    if (location.hostname === 'localhost' || location.hostname === '127.0.0.1') {
      console.warn('Running on localhost without camera access. Using mock video stream.');
      // Create a mock video stream (e.g., a black canvas)
      const canvas = document.createElement('canvas');
      canvas.width = 640;
      canvas.height = 480;
      const ctx = canvas.getContext('2d');
      ctx.fillStyle = '#333';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#fff';
      ctx.font = '20px Arial';
      ctx.fillText('Mock Camera Stream', 50, 50);
      
      const stream = canvas.captureStream(30); // 30 fps
      video.srcObject = stream;
      await video.play();
      streaming = true;
      videoWidth = canvas.width;
      videoHeight = canvas.height;

      setupCanvas();
      await initOpenCVStuff();
      initThree();
      setupWorkerToggle();
      _lastRenderTime = performance.now();
      requestAnimationFrame(mainLoop);
    } else {
      alert('Impossibile accedere alla fotocamera. Controlla i permessi o assicurati di usare HTTPS.');
    }
  }
}

function setupCanvas() {
  overlay.width = video.videoWidth;
  overlay.height = video.videoHeight;
}

// ═══════════════════════════════════════════════════════════════
// THREE.JS SETUP (overlaid on video)
// ═══════════════════════════════════════════════════════════════
function initThree() {
  const container = document.getElementById('three-container');

  // *** Transparent renderer that overlays the video ***
  renderer = new THREE.WebGLRenderer({
    antialias: true,
    alpha: true,
    powerPreference: 'high-performance'
  });
  renderer.setClearColor(0x000000, 0); // Fully transparent
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  if (renderer.outputColorSpace !== undefined) {
    renderer.outputColorSpace = 'srgb';
  }
  container.appendChild(renderer.domElement);

  // Match renderer size to video display area
  updateRendererSize();

  window.renderer = renderer;

  scene = new THREE.Scene();
  window.scene = scene;

  // *** Camera at ORIGIN — projection from intrinsics ***
  camera3 = new THREE.PerspectiveCamera(45, videoWidth / videoHeight, 0.001, 100);
  camera3.position.set(0, 0, 0);
  // Override projection matrix with physical camera intrinsics
  updateProjectionMatrix();

  // Professional 3-point lighting
  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const key = new THREE.DirectionalLight(0xffffff, 1.2);
  key.position.set(1, 2, 1).normalize();
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 0.4);
  fill.position.set(-1, 0.5, -1).normalize();
  scene.add(fill);

  // *** Default model: a little house (casetta) ***
  model = createDefaultHouse();
  model.visible = false;
  scene.add(model);

  // Axis helper for debug (tiny)
  const axes = new THREE.AxesHelper(0.02);
  scene.add(axes);

  // Initialize filters with AAA-grade parameters
  posFilter = new PoseFilters.PredictivePositionFilter({
    responsiveness: 0.85,
    velocitySmoothing: 0.6,
    predictionFactor: 0.4,
    maxVelocity: 2.0
  });
  quatFilter = new PoseFilters.QuaternionFilter({ timeConstant: 0.04 });
  quatEKF = new PoseFilters.QuaternionEKF();

  // Setup all UI controls
  setupControls();

  // Handle window resize
  window.addEventListener('resize', () => {
    updateRendererSize();
    updateProjectionMatrix();
  });
}

/**
 * Creates a detailed low-poly house model.
 * Units are normalized: the house fits in a ~1×1×1 bounding box.
 */
function createDefaultHouse() {
  const group = new THREE.Group();

  // Walls
  const wallGeom = new THREE.BoxGeometry(1, 0.65, 0.8);
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0xE8C07A,
    roughness: 0.7,
    metalness: 0.05
  });
  const walls = new THREE.Mesh(wallGeom, wallMat);
  walls.position.y = 0.325;
  group.add(walls);

  // Roof (pyramid)
  const roofGeom = new THREE.ConeGeometry(0.72, 0.4, 4);
  const roofMat = new THREE.MeshStandardMaterial({
    color: 0x8B4513,
    roughness: 0.85,
    metalness: 0.02
  });
  const roof = new THREE.Mesh(roofGeom, roofMat);
  roof.position.y = 0.85;
  roof.rotation.y = Math.PI / 4;
  group.add(roof);

  // Door
  const doorGeom = new THREE.BoxGeometry(0.18, 0.35, 0.02);
  const doorMat = new THREE.MeshStandardMaterial({ color: 0x5C3317, roughness: 0.9 });
  const door = new THREE.Mesh(doorGeom, doorMat);
  door.position.set(0, 0.175, 0.41);
  group.add(door);

  // Window left
  const winGeom = new THREE.BoxGeometry(0.14, 0.14, 0.02);
  const winMat = new THREE.MeshStandardMaterial({ color: 0x87CEEB, roughness: 0.3, metalness: 0.6 });
  const winL = new THREE.Mesh(winGeom, winMat);
  winL.position.set(-0.28, 0.4, 0.41);
  group.add(winL);

  // Window right
  const winR = new THREE.Mesh(winGeom, winMat);
  winR.position.set(0.28, 0.4, 0.41);
  group.add(winR);

  // Chimney
  const chimGeom = new THREE.BoxGeometry(0.1, 0.3, 0.1);
  const chimMat = new THREE.MeshStandardMaterial({ color: 0x666666, roughness: 0.9 });
  const chimney = new THREE.Mesh(chimGeom, chimMat);
  chimney.position.set(0.25, 1.0, -0.1);
  group.add(chimney);

  return group;
}

function updateRendererSize() {
  if (!renderer || !video) return;
  const vw = video.offsetWidth || video.videoWidth || 640;
  const vh = video.offsetHeight || video.videoHeight || 480;
  renderer.setSize(vw, vh);
}

function updateProjectionMatrix() {
  if (!camera3) return;
  const fx = Number(focalInput?.value || focal);
  const fy = fx;
  const cx = videoWidth / 2;
  const cy = videoHeight / 2;

  camera3.projectionMatrix = buildProjectionMatrix(
    fx, fy, cx, cy, videoWidth, videoHeight, 0.001, 100
  );
  camera3.projectionMatrixInverse.copy(camera3.projectionMatrix).invert();
}

// ═══════════════════════════════════════════════════════════════
// CONTROLS SETUP
// ═══════════════════════════════════════════════════════════════
function setupControls() {
  // GLTF model upload
  if (typeof THREE.GLTFLoader !== 'undefined') {
    const loader = new THREE.GLTFLoader();
    const upload = document.getElementById('upload-model');
    if (upload) {
      upload.addEventListener('change', (ev) => {
        const f = ev.target.files && ev.target.files[0];
        if (!f) return;
        const reader = new FileReader();
        reader.onload = function (e) {
          try {
            loader.parse(e.target.result, '', (gltf) => {
              if (model) scene.remove(model);
              model = gltf.scene;
              model.traverse(c => { if (c.isMesh) c.castShadow = true; });
              model.visible = false;
              scene.add(model);
              // Reset cached scale so it recomputes for the new model
              _cachedScale = null;
              const occToggle = document.getElementById('toggle-occlusion');
              if (occToggle && occToggle.checked) createOccluderFromModel();
            }, (err) => console.error('GLTF parse error', err));
          } catch (e) { console.error(e); }
        };
        reader.readAsArrayBuffer(f);
      });
    }
  }

  // Occlusion toggle
  const occToggle = document.getElementById('toggle-occlusion');
  if (occToggle) {
    occToggle.addEventListener('change', () => {
      if (occToggle.checked) createOccluderFromModel();
      else if (window._occluder) { scene.remove(window._occluder); window._occluder = null; }
    });
  }

  // Advanced occluder toggle
  const advOcc = document.getElementById('toggle-occluder-advanced');
  if (advOcc) {
    advOcc.addEventListener('change', () => {
      if (window._occluder) { scene.remove(window._occluder); window._occluder = null; }
      if (advOcc.checked) createOccluderFromModel(true);
      else if (occToggle && occToggle.checked) createOccluderFromModel(false);
    });
  }

  // Focal length → update projection
  if (focalInput) {
    focalInput.addEventListener('input', () => {
      focal = Number(focalInput.value || 800);
      updateProjectionMatrix();
    });
  }

  // Marker size → reset cached scale
  if (markerSizeInput) {
    markerSizeInput.addEventListener('input', () => {
      markerSizeMM = Number(markerSizeInput.value || 50);
      _cachedScale = null;
    });
  }

  // Detection rate control
  const dr = document.getElementById('detection-rate');
  if (dr) {
    dr.addEventListener('input', (e) => {
      detectionIntervalMs = Number(e.target.value || 33);
    });
    dr.value = detectionIntervalMs;
  }

  // Scale factor control
  const sf = document.getElementById('scale-factor');
  if (sf) {
    sf.addEventListener('input', (e) => {
      userScaleFactor = Number(e.target.value || 2.0);
      _cachedScale = null;
    });
  }

  // Presets
  refreshPresetList();

  const saveBtn = document.getElementById('save-preset');
  if (saveBtn) {
    saveBtn.addEventListener('click', () => {
      const name = document.getElementById('preset-name')?.value?.trim();
      if (!name) return;
      const presets = loadPresets();
      presets[name] = {
        markerSizeMM, focal, detectionIntervalMs, userScaleFactor,
        smoothing: document.getElementById('toggle-smoothing')?.checked ?? true,
        kalman: document.getElementById('toggle-kalman')?.checked ?? false,
        ekf: document.getElementById('toggle-ekf')?.checked ?? false,
      };
      savePresets(presets);
      refreshPresetList();
    });
  }

  const loadBtn = document.getElementById('load-preset');
  if (loadBtn) {
    loadBtn.addEventListener('click', () => {
      const sel = document.getElementById('preset-list');
      if (!sel) return;
      const presets = loadPresets();
      const p = presets[sel.value];
      if (!p) return;
      if (markerSizeInput) markerSizeInput.value = p.markerSizeMM || 50;
      if (focalInput) focalInput.value = p.focal || 800;
      markerSizeMM = p.markerSizeMM || 50;
      focal = p.focal || 800;
      detectionIntervalMs = p.detectionIntervalMs || 33;
      userScaleFactor = p.userScaleFactor || 2.0;
      _cachedScale = null;
      updateProjectionMatrix();
    });
  }

  const delBtn = document.getElementById('delete-preset');
  if (delBtn) {
    delBtn.addEventListener('click', () => {
      const sel = document.getElementById('preset-list');
      if (!sel) return;
      const presets = loadPresets();
      delete presets[sel.value];
      savePresets(presets);
      refreshPresetList();
    });
  }
}

// ═══════════════════════════════════════════════════════════════
// UNIFIED RENDER LOOP (decoupled from detection!)
// ═══════════════════════════════════════════════════════════════
/**
 * Main loop: runs at display refresh rate (60fps).
 * Detection is throttled separately for efficiency.
 * Between detections, we PREDICT position using velocity for ultra-smooth motion.
 */
function mainLoop(timestamp) {
  if (!streaming) {
    requestAnimationFrame(mainLoop);
    return;
  }

  const now = performance.now();
  const dt = Math.min((now - _lastRenderTime) / 1000.0, 0.1); // cap at 100ms
  _lastRenderTime = now;

  // ═══ DETECTION STEP (throttled) ═══
  if (now - _lastDetectionTime > detectionIntervalMs) {
    _lastDetectionTime = now;
    sendFrameForDetection();
  }

  // ═══ RENDER STEP (every frame) ═══
  if (_markerVisible && model && model.visible) {
    const timeSinceDetection = now - _lastMarkerSeen;

    if (timeSinceDetection < MARKER_HIDE_MS) {
      // Inter-frame velocity prediction for smooth motion
      if (posFilter && typeof posFilter.predict === 'function' && dt > 0) {
        const predicted = posFilter.predict(dt);
        if (predicted) model.position.copy(predicted);
      }

      // Graceful fade near timeout
      if (timeSinceDetection > MARKER_FADE_MS) {
        const fadeProgress = (timeSinceDetection - MARKER_FADE_MS) / (MARKER_HIDE_MS - MARKER_FADE_MS);
        _modelOpacity = Math.max(0, 1 - fadeProgress);
        setModelOpacity(model, _modelOpacity);
      }
    } else {
      // Lost marker — hide
      model.visible = false;
      _markerVisible = false;
    }
  }

  // Render
  if (renderer && scene && camera3) {
    renderer.render(scene, camera3);
  }

  requestAnimationFrame(mainLoop);
}

/**
 * Set opacity on all meshes in a model hierarchy.
 */
function setModelOpacity(obj, opacity) {
  if (!obj) return;
  obj.traverse((child) => {
    if (child.isMesh && child.material) {
      if (opacity < 0.99) {
        child.material.transparent = true;
        child.material.opacity = opacity;
        child.material.needsUpdate = true;
      } else {
        child.material.transparent = false;
        child.material.opacity = 1;
      }
    }
  });
}

// ═══════════════════════════════════════════════════════════════
// DETECTION DISPATCH
// ═══════════════════════════════════════════════════════════════
function sendFrameForDetection() {
  if (!overlay || !overlayCtx || !video) return;
  overlayCtx.drawImage(video, 0, 0, overlay.width, overlay.height);

  const w = overlay.width;
  const h = overlay.height;
  const fx = Number(focalInput?.value || focal);
  const cx = w / 2;
  const cy = h / 2;

  const useWorker = document.getElementById('toggle-worker')?.checked ?? true;

  if (useWorker && window._arWorker && !window._arWorker.stub) {
    try {
      createImageBitmap(overlay).then((bmp) => {
        window._arWorker.postMessage({
          type: 'frame',
          bitmap: bmp,
          cameraMatrix: [fx, 0, cx, 0, fx, cy, 0, 0, 1],
          distCoeffs: [0, 0, 0, 0, 0],
          markerLength: markerSizeMM / 1000.0,
          overlayWidth: w,
          overlayHeight: h
        }, [bmp]);
      }).catch(() => {});
    } catch (e) {}
  } else if (!useWorker && typeof cv !== 'undefined' && dictionary) {
    mainThreadDetect(w, h, fx, cx, cy);
  }
}

function mainThreadDetect(w, h, fx, cx, cy) {
  try {
    const imageData = overlayCtx.getImageData(0, 0, w, h);
    srcMat.data.set(imageData.data);

    let corners = new cv.MatVector();
    let ids = new cv.Mat();
    let rejected = new cv.MatVector();
    cv.aruco.detectMarkers(srcMat, dictionary, corners, ids, detectorParams, rejected);

    // Draw debug overlay
    overlayCtx.lineWidth = 2;
    overlayCtx.strokeStyle = '#00FF88';
    overlayCtx.fillStyle = 'rgba(0,255,136,0.12)';

    if (!ids.empty()) {
      for (let i = 0; i < corners.size(); i++) {
        const c = corners.get(i);
        overlayCtx.beginPath();
        overlayCtx.moveTo(c.data32F[0], c.data32F[1]);
        for (let j = 1; j < 4; j++) overlayCtx.lineTo(c.data32F[j * 2], c.data32F[j * 2 + 1]);
        overlayCtx.closePath();
        overlayCtx.stroke();
        overlayCtx.fill();
      }

      // Pose estimation
      const cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, [fx, 0, cx, 0, fx, cy, 0, 0, 1]);
      const distCoeffs = cv.Mat.zeros(1, 5, cv.CV_64F);
      const rvecs = new cv.Mat();
      const tvecs = new cv.Mat();

      cv.aruco.estimatePoseSingleMarkers(corners, markerSizeMM / 1000.0, cameraMatrix, distCoeffs, rvecs, tvecs);

      if (rvecs.rows > 0) {
        const rvec = [rvecs.data64F[0], rvecs.data64F[1], rvecs.data64F[2]];
        const tvec = [tvecs.data64F[0], tvecs.data64F[1], tvecs.data64F[2]];
        applyPose(rvec, tvec, 1.0);
      }

      rvecs.delete(); tvecs.delete();
      cameraMatrix.delete(); distCoeffs.delete();
    }

    corners.delete(); ids.delete(); rejected.delete();
  } catch (e) {
    console.warn('Main thread detect error:', e);
  }
}

// ═══════════════════════════════════════════════════════════════
// POSE APPLICATION — the core of fluid tracking
// ═══════════════════════════════════════════════════════════════
/**
 * Apply a detected pose to the 3D model.
 * Handles coordinate conversion, filtering, scaling.
 * Called from worker results or main thread detection.
 *
 * @param {number[]} rvec  Rodrigues rotation vector [rx, ry, rz]
 * @param {number[]} tvec  Translation vector [tx, ty, tz] in meters
 * @param {number} confidence  0–1 detection confidence
 */
function applyPose(rvec, tvec, confidence) {
  if (!model) return;

  const now = performance.now();
  const dt = Math.max(0.001, Math.min(0.2, (now - (_lastPoseTime || now)) / 1000.0));
  _lastPoseTime = now;
  _lastMarkerSeen = now;

  // *** Convert OpenCV → Three.js coordinates ***
  const { position, quaternion: measuredQuat } = cvPoseToThree(rvec, tvec);

  // Update projection (in case focal changed)
  updateProjectionMatrix();

  // *** Select filtering mode ***
  const useKalmanCB = document.getElementById('toggle-kalman');
  const useEKFCB = document.getElementById('toggle-ekf');
  const useSmoothingCB = document.getElementById('toggle-smoothing');

  const useKalman = useKalmanCB ? useKalmanCB.checked : false;
  const useEKF = useEKFCB ? useEKFCB.checked : false;
  const useSmoothing = useSmoothingCB ? useSmoothingCB.checked : true;

  let finalPos, finalQuat;

  if (!useSmoothing) {
    // Raw pose — no filtering (for debugging)
    finalPos = position;
    finalQuat = measuredQuat;
  } else if (useEKF) {
    // Quaternion EKF for orientation
    let omega = new THREE.Vector3(0, 0, 0);
    if (window._lastMeasQuat) {
      const qprevInv = window._lastMeasQuat.clone().conjugate();
      const qd = measuredQuat.clone().multiply(qprevInv);
      const qAngle = 2 * Math.acos(Math.min(1, Math.abs(qd.w)));
      if (qAngle > 1e-6) {
        const s = Math.sqrt(1 - qd.w * qd.w) || 1e-9;
        omega.set(qd.x / s, qd.y / s, qd.z / s).multiplyScalar(qAngle / Math.max(0.001, dt));
      }
    }
    quatEKF.predict(omega, dt);
    quatEKF.update(measuredQuat);
    finalQuat = quatEKF.q.clone();
    window._lastMeasQuat = measuredQuat.clone();

    // Position via kalman or predictive filter
    if (useKalman) {
      ensureKalman();
      const f = kalman.update([position.x, position.y, position.z]);
      finalPos = new THREE.Vector3(f.x, f.y, f.z);
    } else {
      finalPos = posFilter.update(position, dt, confidence);
    }
  } else if (useKalman) {
    ensureKalman();
    const f = kalman.update([position.x, position.y, position.z]);
    finalPos = new THREE.Vector3(f.x, f.y, f.z);
    quatFilter.update(measuredQuat, dt, confidence);
    finalQuat = quatFilter.current.clone();
  } else {
    // Default: PredictivePositionFilter + QuaternionFilter
    finalPos = posFilter.update(position, dt, confidence);
    finalQuat = quatFilter.update(measuredQuat, dt, confidence);
  }

  // Apply pose to model
  model.position.copy(finalPos);
  model.quaternion.copy(finalQuat);

  // *** Proper scaling ***
  const markerLengthM = markerSizeMM / 1000.0;
  if (!_cachedScale || _cachedScaleMarkerMM !== markerSizeMM) {
    _cachedScale = computeModelScale(model, markerLengthM, userScaleFactor);
    _cachedScaleMarkerMM = markerSizeMM;
  }
  model.scale.set(_cachedScale, _cachedScale, _cachedScale);

  // Show model
  model.visible = true;
  _markerVisible = true;
  _modelOpacity = 1.0;
  setModelOpacity(model, 1.0);

  // Update occluder
  if (window._occluder) {
    window._occluder.position.copy(model.position);
    window._occluder.quaternion.copy(model.quaternion);
    window._occluder.scale.copy(model.scale);
  }

  prevPose = { position: model.position.clone(), rotation: model.quaternion.clone() };
}

// ═══════════════════════════════════════════════════════════════
// WORKER RESULT HANDLER
// ═══════════════════════════════════════════════════════════════
function handleWorkerResult(msg) {
  if (!msg || !msg.markers) return;

  // Draw detection overlay
  overlayCtx.clearRect(0, 0, overlay.width, overlay.height);
  overlayCtx.lineWidth = 2;
  overlayCtx.strokeStyle = '#00FF88';
  overlayCtx.fillStyle = 'rgba(0,255,136,0.12)';

  for (const m of msg.markers) {
    const c = m.corners;
    if (!c || c.length < 4) continue;
    overlayCtx.beginPath();
    overlayCtx.moveTo(c[0][0], c[0][1]);
    overlayCtx.lineTo(c[1][0], c[1][1]);
    overlayCtx.lineTo(c[2][0], c[2][1]);
    overlayCtx.lineTo(c[3][0], c[3][1]);
    overlayCtx.closePath();
    overlayCtx.stroke();
    overlayCtx.fill();
  }

  // No markers → let mainLoop handle fading
  if (msg.markers.length === 0) return;

  const first = msg.markers[0];
  if (first && first.rvec && first.tvec) {
    applyPose(first.rvec, first.tvec, first.confidence || 1.0);
  }
}

// ═══════════════════════════════════════════════════════════════
// OCCLUDER
// ═══════════════════════════════════════════════════════════════
function createOccluderFromModel(advanced = false) {
  if (!model) return;
  if (window._occluder) { scene.remove(window._occluder); window._occluder = null; }

  model.updateMatrixWorld(true);
  const bbox = new THREE.Box3().setFromObject(model);
  const size = new THREE.Vector3();
  bbox.getSize(size);
  const center = new THREE.Vector3();
  bbox.getCenter(center);

  if (advanced) {
    const longAxis = (size.x >= size.y && size.x >= size.z) ? 'x' : (size.y >= size.z) ? 'y' : 'z';
    const slices = 3;
    const group = new THREE.Group();
    for (let i = 0; i < slices; i++) {
      const sliceSize = size.clone();
      sliceSize[longAxis] = size[longAxis] / slices * 1.02;
      const geo = new THREE.BoxGeometry(sliceSize.x, sliceSize.y, sliceSize.z);
      const mat = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: true });
      const box = new THREE.Mesh(geo, mat);
      box.position.copy(center);
      box.position[longAxis] += ((i + 0.5) / slices - 0.5) * size[longAxis];
      box.renderOrder = 0;
      group.add(box);
    }
    model.renderOrder = 1;
    scene.add(group);
    window._occluder = group;
  } else {
    const geo = new THREE.BoxGeometry(size.x * 1.02, size.y * 1.02, size.z * 1.02);
    const mat = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: true });
    const box = new THREE.Mesh(geo, mat);
    box.position.copy(center);
    box.renderOrder = 0;
    model.renderOrder = 1;
    scene.add(box);
    window._occluder = box;
  }
}

// ═══════════════════════════════════════════════════════════════
// OPENCV INIT
// ═══════════════════════════════════════════════════════════════
async function initOpenCVStuff() {
  if (typeof cv === 'undefined' || !cv) {
    throw new Error('OpenCV.js not ready');
  }
  srcMat = new cv.Mat(video.videoHeight, video.videoWidth, cv.CV_8UC4);
  detectorParams = new cv.DetectorParameters();
  if (cv.aruco && cv.aruco.getPredefinedDictionary) {
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250);
  } else {
    console.warn('cv.aruco not found. Marker detection will not work on main thread.');
  }
}

// ═══════════════════════════════════════════════════════════════
// WORKER MANAGEMENT
// ═══════════════════════════════════════════════════════════════
function setupWorkerToggle() {
  const cb = document.getElementById('toggle-worker');
  if (!cb) return;

  // Remove old listener to prevent double-bind
  const handler = (e) => {
    if (e.target.checked) initWorker();
    else {
      if (window._arWorker && !window._arWorker.stub) {
        try { window._arWorker.terminate(); } catch (e) {}
      }
      window._arWorker = null;
    }
  };
  cb.removeEventListener('change', handler);
  cb.addEventListener('change', handler);

  if (cb.checked && !window._arWorker) initWorker();

  // Safety fallback for headless/test environments
  setTimeout(() => {
    if (!window._arWorker) window._arWorker = { stub: true };
  }, 400);
}

function initWorker() {
  if (window._arWorker && !window._arWorker.stub) return;
  try {
    const w = new Worker('workers/aruco-worker.js');
    window._arWorker = w;

    w.postMessage({
      type: 'init',
      markerLength: (markerSizeMM || 50) / 1000.0
    });

    w.onmessage = (ev) => {
      const msg = ev.data;
      if (msg.type === 'ready') {
        console.log('AR Worker ready (js-aruco2)');
        // Configure worker with tuned parameters
        w.postMessage({
          type: 'config',
          markerLength: markerSizeMM / 1000.0,
          cornerSmoothing: 0.4,  // Reduced from 0.6 — less lag
          focalLength: focal,
          cornerFlowEnabled: true,
          cornerFlowRadius: 4,
          cornerFlowTemplate: 9,
          cornerFlowMaxNormalizedSSD: 40
        });
      } else if (msg.type === 'result') {
        handleWorkerResult(msg);
      } else if (msg.type === 'error') {
        console.warn('Worker error:', msg.error);
      } else if (msg.type === 'log') {
        console.log('[Worker]', msg.message);
      }
    };
  } catch (e) {
    console.warn('Cannot init worker:', e);
    window._arWorker = { stub: true };
  }
}

// ═══════════════════════════════════════════════════════════════
// TEST HELPERS (compatible with existing test suite)
// ═══════════════════════════════════════════════════════════════
function _applyPoseMeasurement({ positionArr, quatArr, markerLength = null }) {
  const targetModel = window.model || model;
  if (!positionArr || !quatArr || !targetModel) return false;

  const position = new THREE.Vector3(positionArr[0], positionArr[1], positionArr[2]);
  const measuredQuat = new THREE.Quaternion(quatArr[0], quatArr[1], quatArr[2], quatArr[3]);

  const now = performance.now();
  const dt = Math.max(0.001, (now - (_lastPoseTime || now)) / 1000.0);
  _lastPoseTime = now;
  _lastMarkerSeen = now;

  if (!posFilter) posFilter = new PoseFilters.PredictivePositionFilter({ responsiveness: 0.85 });
  if (!quatFilter) quatFilter = new PoseFilters.QuaternionFilter({ timeConstant: 0.04 });
  if (!quatEKF) quatEKF = new PoseFilters.QuaternionEKF();

  const useKalmanCB = document.getElementById('toggle-kalman');
  const useEKFCB = document.getElementById('toggle-ekf');
  const useKalman = useKalmanCB ? useKalmanCB.checked : false;
  const useEKF = useEKFCB ? useEKFCB.checked : false;

  if (useEKF) {
    let omega = new THREE.Vector3(0, 0, 0);
    if (window._lastMeasQuat) {
      const qprevInv = window._lastMeasQuat.clone().conjugate();
      const qd = measuredQuat.clone().multiply(qprevInv);
      const angle = 2 * Math.acos(Math.min(1, Math.abs(qd.w)));
      if (angle > 1e-6) {
        const s = Math.sqrt(1 - qd.w * qd.w) || 1e-9;
        omega.set(qd.x / s, qd.y / s, qd.z / s).multiplyScalar(angle / Math.max(0.001, dt));
      }
    }
    quatEKF.predict(omega, dt);
    quatEKF.update(measuredQuat);
    targetModel.quaternion.copy(quatEKF.q);
    window._lastMeasQuat = measuredQuat.clone();

    if (useKalman) {
      ensureKalman();
      const f = kalman.update([position.x, position.y, position.z]);
      targetModel.position.set(f.x, f.y, f.z);
    } else {
      const p = posFilter.update(position, dt, 1.0);
      targetModel.position.copy(p);
    }
  } else if (useKalman) {
    ensureKalman();
    const f = kalman.update([position.x, position.y, position.z]);
    targetModel.position.set(f.x, f.y, f.z);
    quatFilter.update(measuredQuat, dt, 1.0);
    targetModel.quaternion.copy(quatFilter.current);
  } else {
    const p = posFilter.update(position, dt, 1.0);
    const q = quatFilter.update(measuredQuat, dt, 1.0);
    targetModel.position.copy(p);
    targetModel.quaternion.copy(q);
  }

  if (typeof markerLength === 'number') {
    const s = computeModelScale(targetModel, markerLength, userScaleFactor);
    targetModel.scale.set(s, s, s);
  }
  targetModel.visible = true;
  _markerVisible = true;
  return true;
}
window._applyPoseMeasurement = _applyPoseMeasurement;

// Stress test helpers (test-only, backward compatible)
function _startStressTest({ durationMs = 10000, intervalMs = 50, noise = 0.01, toggleOcclusion = false } = {}) {
  if (window._stressInterval) return false;
  window._stressStats = { samples: 0, errors: [], start: Date.now() };
  let cnt = 0;
  window._stressInterval = setInterval(() => {
    try {
      const basePos = [0.02, -0.01, -0.15];
      const pos = basePos.map(v => v + (Math.random() * 2 - 1) * noise);
      const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), (Math.random() * 2 - 1) * 0.05);
      const ok = _applyPoseMeasurement({ positionArr: pos, quatArr: [q.x, q.y, q.z, q.w], markerLength: 0.05 });
      if (!ok) window._stressStats.errors.push('apply failed');
      window._stressStats.samples++;
      cnt++;
      if (toggleOcclusion && cnt % 50 === 0) {
        const cb = document.getElementById('toggle-occlusion');
        if (cb) { cb.checked = !cb.checked; cb.dispatchEvent(new Event('change')); }
      }
    } catch (e) { window._stressStats.errors.push(String(e)); }
  }, intervalMs);

  window._rafCount = 0; window._rafRunning = true;
  (function rafTick() { window._rafCount++; if (window._rafRunning) requestAnimationFrame(rafTick); })();

  setTimeout(() => { _stopStressTest(); }, durationMs);
  return true;
}

function _stopStressTest() {
  if (window._stressInterval) { clearInterval(window._stressInterval); window._stressInterval = null; }
  window._rafRunning = false;
  if (window._stressStats) {
    window._stressStats.end = Date.now();
    window._stressStats.duration = window._stressStats.end - window._stressStats.start;
    window._stressStats.rafCount = window._rafCount || 0;
  }
  return window._stressStats;
}
window._startStressTest = _startStressTest;
window._stopStressTest = _stopStressTest;

// ═══════════════════════════════════════════════════════════════
// MARKER DOWNLOAD
// ═══════════════════════════════════════════════════════════════
function downloadMarker(id) {
  if (typeof cv !== 'undefined' && cv && cv.aruco && cv.aruco.drawMarker && dictionary) {
    const size = 200;
    const mat = new cv.Mat(size, size, cv.CV_8UC1);
    cv.aruco.drawMarker(dictionary, id, size, mat, 1);
    const canvas = document.createElement('canvas');
    canvas.width = size; canvas.height = size;
    const ctx = canvas.getContext('2d');
    const imgData = ctx.createImageData(size, size);
    for (let i = 0; i < size * size; i++) {
      const v = mat.data[i];
      imgData.data[i * 4] = v;
      imgData.data[i * 4 + 1] = v;
      imgData.data[i * 4 + 2] = v;
      imgData.data[i * 4 + 3] = 255;
    }
    ctx.putImageData(imgData, 0, 0);
    const link = document.createElement('a');
    link.href = canvas.toDataURL('image/png');
    link.download = `aruco_marker_${id}.png`;
    link.click();
    mat.delete();
  } else {
    window.open('https://chev.me/arucogen/', '_blank');
  }
}

// Expose for debugging / external use
window._arWorker = window._arWorker || null;
window.ArUcoPOC = { initPOC, downloadMarker };
