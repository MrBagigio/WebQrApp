// chooses loader based on file extension and inserts model into #house-model
window.addEventListener('DOMContentLoaded', () => {
  const container = document.getElementById('house-model');
  const fallback = document.getElementById('fallback-house');
  const uploader = document.getElementById('model-upload');
  if (!container) return;

  const loadUrl = (url) => {
    const ext = url.split('.').pop().toLowerCase();
    let loader;
    if (ext === 'glb' || ext === 'gltf') loader = new THREE.GLTFLoader();
    else if (ext === 'fbx') loader = new THREE.FBXLoader();
    if (!loader) return;
    loader.load(url, (g) => {
      container.setAttribute('visible','true');
      container.object3D.add(ext === 'fbx' ? g : g.scene);
      fallback.setAttribute('visible','false');
    }, undefined, () => { fallback.setAttribute('visible','true'); });
  };

  // local upload handler
  if (uploader) {
    uploader.addEventListener('change', ev => {
      const f = ev.target.files && ev.target.files[0];
      if (!f) return;
      const reader = new FileReader();
      reader.onload = e => {
        const blobUrl = URL.createObjectURL(new Blob([e.target.result]));
        loadUrl(blobUrl);
      };
      reader.readAsArrayBuffer(f);
    });
  }

  // try static files too
  const path = 'assets/house';
  ['glb','gltf','fbx'].forEach(ext => {
    const url = `${path}.${ext}`;
    fetch(url, {method:'HEAD'}).then(r=>{if(r.ok) loadUrl(url);});
  });
});