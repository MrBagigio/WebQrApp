import { jest } from '@jest/globals';

// Mock THREE.js
global.THREE = {
    Vector3: class { 
        constructor(x,y,z){this.x=x||0;this.y=y||0;this.z=z||0} 
        clone(){return new THREE.Vector3(this.x,this.y,this.z)} 
        set(x,y,z){this.x=x;this.y=y;this.z=z;return this;}
        copy(v){this.x=v.x;this.y=v.y;this.z=v.z;return this;}
        add(v){this.x+=v.x;this.y+=v.y;this.z+=v.z;return this;}
        sub(v){this.x-=v.x;this.y-=v.y;this.z-=v.z;return this;}
        multiplyScalar(s){this.x*=s;this.y*=s;this.z*=s;return this;}
        project(){return this;}
        toArray(){return [this.x,this.y,this.z];}
        lerp(v, alpha){this.x+=(v.x-this.x)*alpha;this.y+=(v.y-this.y)*alpha;this.z+=(v.z-this.z)*alpha;return this;}
        distanceTo(v){return Math.sqrt((this.x-v.x)**2+(this.y-v.y)**2+(this.z-v.z)**2);}
    },
    Quaternion: class { 
        constructor(x,y,z,w){this.x=x||0;this.y=y||0;this.z=z||0;this.w=w!==undefined?w:1} 
        clone(){return new THREE.Quaternion(this.x,this.y,this.z,this.w)} 
        setFromAxisAngle(){return this} 
        multiply(){return this} 
        copy(q){this.x=q.x;this.y=q.y;this.z=q.z;this.w=q.w;return this;}
        slerp(q, alpha){return this;}
        toArray(){return [this.x,this.y,this.z,this.w];}
    },
    Euler: class {},
    Matrix4: class {},
    Group: class { 
        constructor(){
            this.position=new THREE.Vector3();
            this.quaternion=new THREE.Quaternion();
            this.scale={set:()=>{}, clone:()=>({x:1,y:1,z:1})};
            this.add=()=>{};
            this.traverse=()=>{};
            this.visible=true;
        } 
    },
    Scene: class { add(){} },
    WebGLRenderer: class { constructor(){this.domElement=document.createElement('canvas');this.shadowMap={};this.xr={};} setPixelRatio(){} setSize(){} render(){} },
    PerspectiveCamera: class { constructor(){this.position={set:()=>{}};this.lookAt=()=>{}} updateProjectionMatrix(){} },
    AmbientLight: class {},
    DirectionalLight: class { constructor(){this.shadow={mapSize:{set:()=>{}},camera:{}};this.position={set:()=>{}}} },
    HemisphereLight: class {},
    Mesh: class { constructor(){this.position=new THREE.Vector3();this.rotation=new THREE.Euler();} },
    PlaneGeometry: class {},
    SphereGeometry: class {},
    MeshStandardMaterial: class {},
    AxesHelper: class { constructor(){this.position=new THREE.Vector3();this.quaternion=new THREE.Quaternion();} },
    ShadowMaterial: class {},
    FBXLoader: class { load(){} },
    PCFSoftShadowMap: 1,
    sRGBEncoding: 1,
    ACESFilmicToneMapping: 1
};

// Mock PoseFilters
global.PoseFilters = {
    PredictivePositionFilter: class { 
        constructor(config){this.config=config;} 
        reset(v){}
        update(v, dt, conf){return v.clone();}
        predict(dt){return null;}
    },
    QuaternionFilter: class { 
        constructor(config){this.config=config;} 
        reset(q){}
        update(q, dt, conf){return q.clone();}
    }
};

// Mock DOM
if (!document.getElementById('three-container')) {
    const container = document.createElement('div');
    container.id = 'three-container';
    document.body.appendChild(container);
}
global.window = {
    addEventListener: () => {},
    devicePixelRatio: 1
};
global.navigator = { userAgent: 'Node' };

import { RestorationEngine } from '../../utils/restoration-engine.js';

describe('RestorationEngine Anti-Regression Tests', () => {
    let engine;

    beforeEach(() => {
        engine = new RestorationEngine();
        engine.log = jest.fn();
        engine._initThree(); // Initialize Three.js objects
    });

    test('setModelXOffset updates _modelXOffset and applies transform', () => {
        engine._applyOffsets = jest.fn();
        engine.setModelXOffset(0.15);
        expect(engine._modelXOffset).toBe(0.15);
        expect(engine._applyOffsets).toHaveBeenCalled();
    });

    test('setModelZOffset updates _modelZOffset and applies transform', () => {
        engine._applyOffsets = jest.fn();
        engine.setModelZOffset(-0.2);
        expect(engine._modelZOffset).toBe(-0.2);
        expect(engine._applyOffsets).toHaveBeenCalled();
    });

    test('_applyTrackedPose uses posFilter and quatFilter when available', () => {
        const mockPosFilter = {
            reset: jest.fn(),
            update: jest.fn().mockReturnValue(new THREE.Vector3(1, 2, 3))
        };
        const mockQuatFilter = {
            reset: jest.fn(),
            update: jest.fn().mockReturnValue(new THREE.Quaternion(0, 0, 0, 1))
        };
        
        engine.posFilter = mockPosFilter;
        engine.quatFilter = mockQuatFilter;
        engine.camera = new THREE.PerspectiveCamera();
        engine.overlay = { width: 800, height: 600 };
        
        const poseful = [{
            id: 1,
            rvec: [0, 0, 0],
            tvec: [0, 0, 1],
            corners: [[0,0], [10,0], [10,10], [0,10]],
            confidence: 0.9
        }];
        
        engine._applyTrackedPose(poseful, null, 1000);
        
        // First pose should reset filters
        expect(mockPosFilter.reset).toHaveBeenCalled();
        expect(mockQuatFilter.reset).toHaveBeenCalled();
        
        // Second pose should update filters
        engine._applyTrackedPose(poseful, null, 1033);
        expect(mockPosFilter.update).toHaveBeenCalled();
        expect(mockQuatFilter.update).toHaveBeenCalled();
        
        // Target position should be updated from filter
        expect(engine._poseTargetPosition.x).toBe(1);
        expect(engine._poseTargetPosition.y).toBe(2);
        expect(engine._poseTargetPosition.z).toBe(3);
    });
});
