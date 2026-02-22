
import { jest } from '@jest/globals';

// Mock THREE.js
global.THREE = {
    Vector3: class { constructor(x,y,z){this.x=x||0;this.y=y||0;this.z=z||0} clone(){return new THREE.Vector3(this.x,this.y,this.z)} },
    Quaternion: class { constructor(x,y,z,w){this.x=x||0;this.y=y||0;this.z=z||0;this.w=w!==undefined?w:1} clone(){return new THREE.Quaternion(this.x,this.y,this.z,this.w)} setFromAxisAngle(){return this} multiply(){return this} },
    Euler: class {},
    Matrix4: class {},
    Group: class { constructor(){this.position=new THREE.Vector3();this.quaternion=new THREE.Quaternion();this.scale={set:()=>{}};this.add=()=>{};this.traverse=()=>{}} },
    Scene: class { add(){} },
    WebGLRenderer: class { constructor(){this.domElement={};this.shadowMap={};} setPixelRatio(){} setSize(){} render(){} },
    PerspectiveCamera: class { constructor(){this.position={set:()=>{}};this.lookAt=()=>{}} updateProjectionMatrix(){} },
    AmbientLight: class {},
    DirectionalLight: class { constructor(){this.shadow={mapSize:{set:()=>{}},camera:{}};this.position={set:()=>{}}} },
    HemisphereLight: class {},
    Mesh: class {},
    PlaneGeometry: class {},
    ShadowMaterial: class {},
    FBXLoader: class { load(){} },
    PCFSoftShadowMap: 1,
    sRGBEncoding: 1,
    ACESFilmicToneMapping: 1
};

// Mock PoseFilters
global.PoseFilters = {
    PredictivePositionFilter: class { constructor(config){this.config=config;} },
    QuaternionFilter: class { constructor(config){this.config=config;} }
};

// Mock DOM
global.document = {
    getElementById: () => ({ clientWidth: 100, clientHeight: 100, appendChild: ()=>{} }),
    createElement: () => ({ getContext: ()=>({}), width:0, height:0 }),
    addEventListener: () => {}
};
global.window = {
    addEventListener: () => {},
    devicePixelRatio: 1
};
global.navigator = { userAgent: 'Node' };

// Import the engine
import { RestorationEngine } from '../../utils/restoration-engine.js';

describe('RestorationEngine Refactoring Tests', () => {
    let engine;

    beforeEach(() => {
        engine = new RestorationEngine();
        // Mock internal log
        engine.log = jest.fn();
        // Mock setFilterParams to avoid creating real filters (though we mocked filters too)
        // But we want to test applyStabilityPreset which calls setFilterParams
    });

    test('applyStabilityPreset sets correct params for "minimal"', () => {
        engine.setFilterParams = jest.fn();
        engine.setAnchorBoost = jest.fn();
        engine.setUseQuaternionEKF = jest.fn();
        engine.setAnchorIds = jest.fn();
        engine.setAnchorAutoLockEnabled = jest.fn();
        engine.clearAnchorLock = jest.fn();
        engine.setAnchorLockEnabled = jest.fn();
        engine.applyStabilityPreset('minimal');
        
        expect(engine.setFilterParams).toHaveBeenCalledWith(expect.objectContaining({ positionSmoothing: 0.05 }));
        expect(engine.setAnchorBoost).toHaveBeenCalledWith(1.0);
    });

    test('applyStabilityPreset sets correct params for "mobile"', () => {
        engine.setFilterParams = jest.fn();
        engine.setAnchorBoost = jest.fn();
        engine.setUseQuaternionEKF = jest.fn();
        engine.setAnchorIds = jest.fn();
        engine.setAnchorAutoLockEnabled = jest.fn();
        engine.clearAnchorLock = jest.fn();
        engine.setAnchorLockEnabled = jest.fn();
        // Mock worker
        engine.worker = { postMessage: jest.fn() };
        
        // Mock worker config methods
        engine.setCornerSmoothing = jest.fn();
        engine.setCornerFlowEnabled = jest.fn();
        engine.setCornerFlowSSDThreshold = jest.fn();
        engine.setUseSubpixel = jest.fn();
        engine.setSubpixelParams = jest.fn();
        engine.setUseAprilTag = jest.fn();
        engine.setUseSolvePnP = jest.fn();
        engine.setUsePyrLKFlow = jest.fn();
        engine.setMarkerOutlierDistanceMeters = jest.fn();
        engine.setMarkerConfidenceThreshold = jest.fn();

        engine.applyStabilityPreset('mobile');
        
        expect(engine.setFilterParams).toHaveBeenCalledWith(expect.objectContaining({ positionSmoothing: 0.08 }));
        expect(engine.setAnchorBoost).toHaveBeenCalledWith(2.5);
    });

    test('_calculateAdaptiveParameters returns base values when adaptive is disabled', () => {
        engine._adaptiveTuningEnabled = false;
        if (typeof engine._calculateAdaptiveParameters === 'function') {
            const params = engine._calculateAdaptiveParameters(0.5); // High noise
            expect(params.adaptiveTrackWindow).toBeCloseTo(0.24); // Default
            expect(params.adaptiveOutlierDistance).toBeCloseTo(0.35); // Default
        }
    });

    test('_calculateAdaptiveParameters scales values when adaptive is enabled', () => {
        engine._adaptiveTuningEnabled = true;
        const baseWindow = 0.24;
        
        if (typeof engine._calculateAdaptiveParameters === 'function') {
            // Zero noise -> same as base (roughly)
            const paramsZero = engine._calculateAdaptiveParameters(0);
            expect(paramsZero.adaptiveTrackWindow).toBeCloseTo(baseWindow);

            // High noise -> larger window
            const paramsHigh = engine._calculateAdaptiveParameters(0.5);
            expect(paramsHigh.adaptiveTrackWindow).toBeGreaterThan(baseWindow);
        }
    });

    test('_getHardwareFocal uses explicit hardware info', () => {
        // Mock video track with method
        const mockTrack = {
            getSettings: () => ({ fov: 60, facingMode: 'environment' })
        };
        engine.video = {
            srcObject: {
                getVideoTracks: () => [mockTrack]
            }
        };
        
        const focal = engine._getHardwareFocal(1000); // 1000px width
        // focal = (1000/2) / tan(30deg) = 500 / 0.577 = ~866
        expect(focal).toBeCloseTo(866, 0);
    });
});
