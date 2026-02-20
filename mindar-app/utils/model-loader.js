// chooses loader based on file extension and inserts model into #house-model
window.addEventListener('DOMContentLoaded', () => {
  const container = document.getElementById('house-model');
  const fallback = document.getElementById('fallback-house');
  const uploader = document.getElementById('model-upload');
  if (!container) return;

  const loadUrl = (url) => {
    console.log('attempting load of', url);
    const ext = url.split('.').pop().toLowerCase();
    let loader;
    if (ext === 'glb' || ext === 'gltf') loader = new THREE.GLTFLoader();
    else if (ext === 'fbx') loader = new THREE.FBXLoader();
    if (!loader) { console.warn('no loader for', ext); return; }
    loader.load(url, (g) => {
      console.log('loader success for', url);
      container.setAttribute('visible','true');
        const obj = (ext === 'fbx' ? g : g.scene);
        // force materials to opaque in case opacity was zero
        obj.traverse(o => {
          if (o.isMesh && o.material) {
            o.material.opacity = 1;
            o.material.transparent = false;
            o.material.needsUpdate = true;
          }
        });
        container.object3D.add(obj);
      fallback.setAttribute('visible','false');
    }, undefined, (err) => { console.error('loader error',err); fallback.setAttribute('visible','true'); });
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
    console.log('checking static url', url);
    fetch(url, {method:'HEAD'}).then(r=>{
      console.log('HEAD', url, r.status);
      if(r.ok) loadUrl(url);
    }).catch(e=>console.warn('HEAD error',url,e));
  });
});