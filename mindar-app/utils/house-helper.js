// utility to load either FBX or GLB model inside mindar-app
async function exists(url) {
  try {
    const r = await fetch(url, { method: 'HEAD' });
    return r.ok;
  } catch {
    return false;
  }
}

async function loadHouse() {
  const anchor = document.getElementById('model-anchor');
  if (!anchor) return;

  let path = null;
  if (await exists('assets/house.fbx')) path = 'assets/house.fbx';
  else if (await exists('assets/house.glb')) path = 'assets/house.glb';
  if (!path) return; // no model, nothing to do

  if (path.endsWith('.fbx')) {
    if (typeof THREE.FBXLoader !== 'undefined') {
      const loader = new THREE.FBXLoader();
      loader.load(path, obj => {
        obj.scale.set(0.1,0.1,0.1);
        obj.rotation.y = Math.PI; // flip
        anchor.appendChild(obj);
      }, undefined, err => console.error('FBX load error', err));
    } else {
      console.warn('FBXLoader not available, cannot load FBX');
    }
  } else {
    const ent = document.createElement('a-entity');
    ent.setAttribute('gltf-model', path);
    ent.setAttribute('scale','0.1 0.1 0.1');
    ent.setAttribute('rotation','0 180 0');
    anchor.appendChild(ent);
  }
}

window.addEventListener('DOMContentLoaded', loadHouse);
