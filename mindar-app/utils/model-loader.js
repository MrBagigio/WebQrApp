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
        // diagnostics: compute bounding box
        const box = new THREE.Box3().setFromObject(obj);
        const size = new THREE.Vector3(); box.getSize(size);
        console.log('model bbox size', size);
        // center the model geometry at origin so it sits nicely on the QR
        const center = new THREE.Vector3();
        box.getCenter(center);
        console.log('centering model by', center);
        obj.position.sub(center);
        // automatically scale model so largest dimension ~1.5m (bigger for visibility)
        const maxd = Math.max(size.x, size.y, size.z);
        if (maxd > 0) {
          // global multiplier comes from UI slider (or console) and defaults to 1
          const globalMult = parseFloat(window.houseScaleMultiplier) || 1.0;
          const targetSize = 1.5 * globalMult;
          const factor = targetSize / maxd;
          obj.scale.setScalar(factor);
          console.log('scaling model by', factor, '(target', targetSize, 'm, mult', globalMult,')');
        }
        // apply vertical offset if set
        if (window.verticalOffset) {
          obj.position.y += window.verticalOffset;
        }
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

  // try static files too, but perform HEAD first so we can detect a missing web server
  const tryStatic = () => {
    const path = './assets/house'; // base path for static model (keep unchanged for compatibility)
    let pending = 0;
    let found = false;
    let connRefused = false;
    ['glb','gltf','fbx'].forEach(ext => {
      pending++;
      const url = `${path}.${ext}`;
      console.log('checking static url', url);
      fetch(url, { method: 'HEAD' })
        .then(resp => {
          if (resp.ok && !found) {
            found = true;
            loadUrl(url);
          }
        })
        .catch(err => {
          console.log('HEAD error', url, err);
          // network failures (connection refused) show up as "Failed to fetch"
          if (err.message && err.message.includes('Failed to fetch')) {
            connRefused = true;
          }
        })
        .finally(() => {
          pending--;
          if (pending === 0 && !found) {
            if (connRefused) {
              const hint = document.getElementById('hint');
              if (hint) {
                hint.innerHTML = 'Impossibile raggiungere il server; avvia un server statico (es. <code>npx live-server</code>).';
              }
            } else {
              console.log('no static model file found');
            }
          }
        });
    });
  };

  tryStatic();
});