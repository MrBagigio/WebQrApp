// utility to toggle fallback geometry when GLB loads
window.addEventListener('DOMContentLoaded', () => {
  const model = document.getElementById('house-model');
  const fallback = document.getElementById('fallback-house');
  if (!model || !fallback) return;
  model.addEventListener('model-loaded', () => {
    fallback.setAttribute('visible', 'false');
  });
  model.addEventListener('error', () => {
    fallback.setAttribute('visible', 'true');
  });
});