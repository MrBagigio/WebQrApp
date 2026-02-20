/**
 * debug-restoration.js
 */
const fs = require('fs');
const path = require('path');

// Mock simple parts
class MockMesh {
    constructor() { this.material = { opacity: 0 }; }
}

const engineCode = fs.readFileSync(path.join(__dirname, 'utils/restoration-engine.js'), 'utf8');
const cleanCode = engineCode.replace(/export class/, 'class').replace(/import .* from .*;/g, '');

try {
    const RestorationEngine = eval(`(function(){ 
        const THREE = { 
            Group: class { add(){} },
            BoxGeometry: class {},
            MeshStandardMaterial: class {},
            MeshBasicMaterial: class {},
            Mesh: class { constructor(){ this.position={y:0}; this.material={opacity:0}; } },
            Scene: class { add(){} },
            WebGLRenderer: class { setSize(){}; setPixelRatio(){}; render(){}; get domElement(){ return {} } },
            PerspectiveCamera: class {},
            AmbientLight: class {},
            DirectionalLight: class {},
            Vector3: class { set(){}; copy(){} },
            Quaternion: class { setFromAxisAngle(){}; copy(){} },
            GridHelper: class { constructor(){ this.position={z:0}; this.rotation={x:0}; } }
        };
        const PoseFilters = {
            AdaptivePositionFilter: class { update(){ return {copy:()=>{}} } },
            QuaternionFilter: class { update(){ return {copy:()=>{}} } }
        };
        ${cleanCode}; 
        return RestorationEngine; 
    })()`);

    const engine = new RestorationEngine();
    engine.restoredModel = new MockMesh();
    engine.destroyedModel = new MockMesh();

    console.log('Initial restored opacity:', engine.restoredModel.material.opacity);
    console.log('Initial destroyed opacity:', engine.destroyedModel.material.opacity);

    engine.setRestorationLevel(0.5);
    console.log('Restoration level 0.5:');
    console.log(' - Restored:', engine.restoredModel.material.opacity);
    console.log(' - Destroyed:', engine.destroyedModel.material.opacity);

    if (engine.restoredModel.material.opacity === 0.5 && engine.destroyedModel.material.opacity === 0.25) {
        console.log('LOGIC TEST PASSED');
    } else {
        console.log('LOGIC TEST FAILED');
        process.exit(1);
    }
} catch (e) {
    console.error('ERROR during eval or execution:', e);
    process.exit(1);
}
