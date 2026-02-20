// chooses loader based on file extension and inserts model into #house-model
window.addEventListener('DOMContentLoaded', () => {
  const container = document.getElementById('house-model');
  const fallback = document.getElementById('fallback-house');
  if (!container) return;
  const path = 'assets/house';
  const attemptLoad = (url) => {
    const ext = url.split('.').pop().toLowerCase();
    if (ext === 'glb' || ext === 'gltf') {
      const loader = new THREE.GLTFLoader();
      loader.load(url, (gltf) => {
        container.setAttribute('visible','true');
        container.object3D.add(gltf.scene);
        fallback.setAttribute('visible','false');
      }, undefined, () => { fallback.setAttribute('visible','true'); });
    } else if (ext === 'fbx') {
      const loader = new THREE.FBXLoader();
      loader.load(url, (fbx) => {
        container.setAttribute('visible','true');
        container.object3D.add(fbx);
        fallback.setAttribute('visible','false');
      }, undefined, () => { fallback.setAttribute('visible','true'); });
    } else {
      fallback.setAttribute('visible','true');
    }
  };
  // try common extensions in order
  ['glb','gltf','fbx'].forEach(ext => {
    const url = `${path}.${ext}`;
    fetch(url, {method:'HEAD'}).then(r=>{if(r.ok) attemptLoad(url);});
  });
});