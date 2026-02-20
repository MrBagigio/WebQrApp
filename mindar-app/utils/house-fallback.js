// optional script to show fallback house if GLB fails to load
window.addEventListener('load', () => {
  const model = document.getElementById('house-model');
  const box = document.getElementById('house-box');
  model.addEventListener('model-loaded', () => {
    box.setAttribute('visible', 'false');
    model.setAttribute('visible', 'true');
  });
  model.addEventListener('error', () => {
    box.setAttribute('visible', 'true');
  });
});