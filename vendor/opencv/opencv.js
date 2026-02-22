// Lightweight local loader for OpenCV (placeholder).
// Replace this file with a full OpenCV.js build and the corresponding opencv_js.wasm in the same folder
// for offline / production usage. When present, scripts will prefer this local build.
// This placeholder will attempt to load the CDN build as a fallback.
(function(){
  // If a real build is present, it should override this placeholder.
  if (typeof cv !== 'undefined' && cv && cv.Mat) return;

  // Small helper: load CDN fallback
  var s = document.createElement('script');
  s.src = 'https://docs.opencv.org/4.5.2/opencv.js';
  s.async = true;
  s.onload = function(){ console.log('Fallback OpenCV loaded from CDN'); };
  s.onerror = function(){ console.warn('OpenCV local loader failed to fetch CDN fallback'); };
  document.head.appendChild(s);
})();
