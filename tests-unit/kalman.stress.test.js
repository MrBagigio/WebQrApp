/** @jest-environment jsdom */

// Simple stress test for KalmanFilter3D (copied implementation) to verify stability under noise
class KalmanFilter3D {
  constructor({ R = 0.01, Q = 0.001, initial = { x:0,y:0,z:0 } } = {}) {
    this.R = R; this.Q = Q; this.x = [initial.x, initial.y, initial.z]; this.P = [[1,0,0],[0,1,0],[0,0,1]];
  }
  update(measurement) {
    for (let i = 0; i < 3; i++) {
      const z = measurement[i];
      const K = this.P[i][i] / (this.P[i][i] + this.R);
      this.x[i] = this.x[i] + K * (z - this.x[i]);
      this.P[i][i] = (1 - K) * this.P[i][i] + Math.abs(this.Q);
    }
    return { x: this.x[0], y: this.x[1], z: this.x[2] };
  }
}

describe('Kalman stress', () => {
  test('Converges under noisy measurements', () => {
    const k = new KalmanFilter3D({ R:0.05, Q:0.001, initial: { x:0,y:0,z:0 } });
    // true signal at [0.1, -0.02, -0.15]
    const trueSignal = [0.1, -0.02, -0.15];
    let last = null;
    for (let i = 0; i < 5000; i++) {
      const noisy = trueSignal.map((v) => v + (Math.random()*2-1)*0.05);
      last = k.update(noisy);
    }
    // final estimate should be close to true signal
    expect(Math.abs(last.x - trueSignal[0])).toBeLessThan(0.02);
    expect(Math.abs(last.y - trueSignal[1])).toBeLessThan(0.02);
    expect(Math.abs(last.z - trueSignal[2])).toBeLessThan(0.02);
  }, 20000);
});