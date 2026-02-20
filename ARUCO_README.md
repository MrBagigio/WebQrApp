POC: Object Tracking with ArUco + OpenCV.js + Three.js

Files added:
- `aruco-poc.html` - demo page (camera preview + marker detection + Three.js overlay)
- `aruco.js` - detection and rendering logic

Quick start:
1. Open `aruco-poc.html` on a mobile device (HTTPS recommended) or desktop with camera.
2. Allow camera access when prompted.
3. Print an ArUco marker (ID 0). If the page cannot render a marker, click 'Scarica marker (id 0)' to generate one (depends on OpenCV build). Otherwise use an external generator: https://chev.me/arucogen/
4. Place marker on or next to your physical object and point the camera at it. The colored box should appear anchored over the marker.
5. Use 'Marker size' (mm) and 'Focal factor' to calibrate scale/position. Click 'Calibra (salva)' to persist settings.

Notes & Limitations:
- This is a POC: accuracy depends on marker placement, lighting, and focal calibration. Use stiff printed markers and good light.
- For production-grade object-tracking without visible markers, consider ARKit/ARCore object targets or a commercial web SDK (8th Wall / Vuforia Web).
- Occlusion is not handled in this POC (requires depth mask or a 3D model of the physical object).

If you want, I can: generate a printable sheet with several markers, add smoothing/kalman filters, or integrate a glTF model instead of the placeholder box.