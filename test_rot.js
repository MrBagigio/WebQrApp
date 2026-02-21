
const THREE = require('three');
const obj = new THREE.Object3D();
// FBXLoader fixes Z-up to Y-up
obj.rotation.set(-Math.PI / 2, 0, 0);
// Apply yaw correction around World Y
obj.rotateOnWorldAxis(new THREE.Vector3(0, 1, 0), Math.PI);
obj.updateMatrixWorld(true);
const localX = new THREE.Vector3(1, 0, 0).applyMatrix4(obj.matrixWorld);
const localY = new THREE.Vector3(0, 1, 0).applyMatrix4(obj.matrixWorld);
const localZ = new THREE.Vector3(0, 0, 1).applyMatrix4(obj.matrixWorld);
console.log('Z-up fixed with World Y yaw:');
console.log('Local X (Right) ->', localX.toArray().map(n => Math.round(n)));
console.log('Local Y (Forward) ->', localY.toArray().map(n => Math.round(n)));
console.log('Local Z (Up) ->', localZ.toArray().map(n => Math.round(n)));

