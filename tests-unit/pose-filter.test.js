/** @jest-environment jsdom */

const THREE = require('three');
// expose THREE as global for the non-module pose-filter implementation
global.THREE = THREE;
// Load the pose-filter script into the jsdom global scope
require('../utils/pose-filter.js');

describe('Pose Filters - unit tests', () => {
  test('QuaternionFilter slerps toward measured quaternion', () => {
    const qf = new window.PoseFilters.QuaternionFilter({ timeConstant: 0.1 });
    const q0 = new THREE.Quaternion();
    qf.reset(q0);
    const measured = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0), Math.PI/4);
    const before = qf.current.clone();
    const res = qf.update(measured, 0.016, 1.0);
    expect(res).toBeDefined();
    expect(res.equals(before)).toBe(false);
  });

  test('AdaptivePositionFilter lerps toward measured position', () => {
    const pf = new window.PoseFilters.AdaptivePositionFilter({ smoothing: 0.2 });
    const v0 = new THREE.Vector3(0,0,0); pf.reset(v0);
    const measured = new THREE.Vector3(0.2, 0.1, -0.05);
    const res = pf.update(measured, 0.016, 1.0);
    expect(res.x).toBeGreaterThan(0);
    expect(res.y).toBeGreaterThan(0);
  });

  test('QuaternionEKF predict + update maintains normalized quaternion', () => {
    const ekf = new window.PoseFilters.QuaternionEKF();
    const measured = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0), 0.2);
    ekf.predict(new THREE.Vector3(0.01, 0.02, -0.005), 0.016);
    ekf.update(measured);
    const q = ekf.q;
    const len = Math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    expect(Math.abs(len - 1.0)).toBeLessThan(1e-6);
  });
});