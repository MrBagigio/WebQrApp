
const THREE = require('three');
// Marker flat on table, camera looking slightly down.
// Let's say marker is identity in Three.js space (just for testing offset)
const markerQuat = new THREE.Quaternion(); 
const rotOffset = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0, 'XYZ'));
const houseQuat = markerQuat.clone().multiply(rotOffset);

const houseUp = new THREE.Vector3(0, 1, 0).applyQuaternion(houseQuat);
const houseForward = new THREE.Vector3(0, 0, -1).applyQuaternion(houseQuat);
console.log('House Up in Marker Space:', houseUp.toArray().map(n => Math.round(n)));
console.log('House Forward in Marker Space:', houseForward.toArray().map(n => Math.round(n)));

