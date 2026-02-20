/**
 * pose-filter.js — AAA-grade adaptive filtering for AR pose tracking
 * Provides low-latency position/rotation filters with velocity prediction.
 *
 * Filters:
 *   QuaternionFilter       — Fast adaptive slerp (shortest-path safe)
 *   AdaptivePositionFilter — Legacy simple EMA (kept for compat)
 *   PredictivePositionFilter — Position filter with velocity estimation + inter-frame extrapolation
 *   QuaternionEKF          — Multiplicative Extended Kalman for orientation
 */
(function (global) {
  'use strict';

  // ─── QuaternionFilter ─────────────────────────────────────────────
  class QuaternionFilter {
    /**
     * @param {Object} opts
     * @param {number} opts.timeConstant  Smoothing time constant in seconds.
     *   Lower = more responsive (0.03–0.06 recommended for AR).
     */
    constructor({ timeConstant = 0.04 } = {}) {
      this.timeConstant = timeConstant;
      this.current = null; // THREE.Quaternion
    }

    reset(q) {
      this.current = q.clone ? q.clone() : new THREE.Quaternion(q.x, q.y, q.z, q.w);
    }

    /**
     * Update with a new measured quaternion.
     * @param {THREE.Quaternion} measured
     * @param {number} dt  Delta time (seconds)
     * @param {number} confidence  0–1
     * @returns {THREE.Quaternion}
     */
    update(measured, dt, confidence = 1.0) {
      if (!measured) return this.current;
      if (!this.current) return (this.current = measured.clone());

      const alpha = 1 - Math.exp(-dt / Math.max(1e-6, this.timeConstant));
      const adj = Math.min(1, alpha * confidence);

      // *** Ensure shortest-path slerp (avoid 360° flips) ***
      if (this.current.dot(measured) < 0) {
        measured = measured.clone();
        measured.x *= -1;
        measured.y *= -1;
        measured.z *= -1;
        measured.w *= -1;
      }

      if (typeof this.current.slerp === 'function') {
        this.current.slerp(measured, adj);
      } else {
        // Manual fallback
        this.current.x += (measured.x - this.current.x) * adj;
        this.current.y += (measured.y - this.current.y) * adj;
        this.current.z += (measured.z - this.current.z) * adj;
        this.current.w += (measured.w - this.current.w) * adj;
        this.current.normalize();
      }
      return this.current;
    }
  }

  // ─── AdaptivePositionFilter (legacy compat) ────────────────────────
  class AdaptivePositionFilter {
    constructor({ smoothing = 0.12 } = {}) {
      this.smoothing = smoothing;
      this.current = null;
    }
    reset(v) {
      this.current = v.clone ? v.clone() : new THREE.Vector3(v.x, v.y, v.z);
    }
    update(measured, dt, confidence = 1.0) {
      if (!measured) return this.current;
      if (!this.current) return (this.current = measured.clone());
      const alpha = 1 - Math.exp(-dt / Math.max(1e-6, this.smoothing));
      const adj = alpha * confidence;
      this.current.lerp(measured, adj);
      return this.current;
    }
  }

  // ─── PredictivePositionFilter ──────────────────────────────────────
  /**
   * Position filter with velocity estimation for smooth inter-frame
   * prediction. This is the KEY filter for fluid AR:
   *
   * - On detection frames: update position + estimate velocity
   * - On render frames (no detection): extrapolate using velocity
   *
   * @param {number} responsiveness  How fast to follow new measurements (0=frozen, 1=instant)
   * @param {number} velocitySmoothing  EMA factor for velocity estimation
   * @param {number} predictionFactor  How much to trust velocity extrapolation (0=none, 1=full)
   * @param {number} maxVelocity  Clamp velocity magnitude (m/s) to reject outliers
   */
  class PredictivePositionFilter {
    constructor({
      responsiveness = 0.85,
      velocitySmoothing = 0.6,
      predictionFactor = 0.4,
      maxVelocity = 2.0,
      maxPredictionDt = 0.033,
      maxPredictionStep = 0.02,
      velocityDamping = 0.9,
      positionDeadband = 0.0015
    } = {}) {
      this.responsiveness = responsiveness;
      this.velocitySmoothing = velocitySmoothing;
      this.predictionFactor = predictionFactor;
      this.maxVelocity = maxVelocity;
      this.maxPredictionDt = maxPredictionDt;
      this.maxPredictionStep = maxPredictionStep;
      this.velocityDamping = velocityDamping;
      this.positionDeadband = positionDeadband;
      this.current = null;        // THREE.Vector3
      this.velocity = null;       // THREE.Vector3
      this.lastMeasurement = null;
    }

    reset(v) {
      this.current = v.clone ? v.clone() : new THREE.Vector3(v.x, v.y, v.z);
      this.velocity = new THREE.Vector3(0, 0, 0);
      this.lastMeasurement = this.current.clone();
    }

    /**
     * Update with a new measurement from the detector.
     * @param {THREE.Vector3} measured
     * @param {number} dt  Seconds since last measurement
     * @param {number} confidence  0–1
     * @returns {THREE.Vector3}
     */
    update(measured, dt, confidence = 1.0) {
      if (!measured) return this.predict(dt);
      if (!this.current) {
        this.current = measured.clone();
        this.velocity = new THREE.Vector3(0, 0, 0);
        this.lastMeasurement = measured.clone();
        return this.current;
      }

      // Estimate instantaneous velocity from measurement delta
      if (dt > 0.001 && this.lastMeasurement) {
        const instantVel = new THREE.Vector3()
          .subVectors(measured, this.lastMeasurement)
          .divideScalar(dt);

        // Clamp to reject wild outliers (hand shake, detection jumps)
        instantVel.clampLength(0, this.maxVelocity);

        // Smooth velocity with EMA
        if (!this.velocity) {
          this.velocity = instantVel.clone();
        } else {
          this.velocity.lerp(instantVel, this.velocitySmoothing);
        }
      }

      this.lastMeasurement = measured.clone();

      const deltaToCurrent = new THREE.Vector3().subVectors(measured, this.current);
      if (deltaToCurrent.length() < this.positionDeadband) {
        if (this.velocity) this.velocity.multiplyScalar(this.velocityDamping);
        return this.current;
      }

      // Blend towards measurement: high responsiveness = fast follow
      const alpha = Math.min(1, this.responsiveness * confidence);
      this.current.lerp(measured, alpha);

      return this.current;
    }

    /**
     * Predict position without a new measurement.
     * Called during render frames between detections for ultra-smooth motion.
     * @param {number} dt  Seconds since last render
     * @returns {THREE.Vector3|null}
     */
    predict(dt) {
      if (!this.current) return null;
      if (this.velocity && this.predictionFactor > 0 && dt > 0) {
        const safeDt = Math.min(this.maxPredictionDt, Math.max(0, dt));
        const step = this.velocity.clone().multiplyScalar(safeDt * this.predictionFactor);
        step.clampLength(0, this.maxPredictionStep);
        this.current.add(step);

        if (this.velocityDamping < 1) {
          this.velocity.multiplyScalar(this.velocityDamping);
        }
      }
      return this.current;
    }
  }

  // ─── QuaternionEKF ────────────────────────────────────────────────
  /**
   * Minimal multiplicative Extended Kalman Filter for orientation.
   * State: quaternion q + gyro bias b(3).
   * Suitable for fusing angular-rate estimates with measured orientations.
   */
  class QuaternionEKF {
    constructor({ qInit = null, P = null, Q = null, R = null } = {}) {
      this.q = qInit ? qInit.clone() : new THREE.Quaternion();
      this.b = new THREE.Vector3(0, 0, 0);
      this.P = P || { ori: 0.01, bias: 0.0001 };
      this.Q = Q || { gyro: 1e-4, bias: 1e-6 };
      this.R = R || 0.01;
    }

    setState({ quat, bias }) {
      if (quat) this.q.copy(quat);
      if (bias) this.b.copy(bias);
    }

    predict(omega, dt) {
      const w = new THREE.Vector3().copy(omega).sub(this.b);
      const wx = w.x, wy = w.y, wz = w.z;
      const norm = Math.sqrt(wx * wx + wy * wy + wz * wz);
      const theta = norm * dt;
      let dq = new THREE.Quaternion();
      if (theta > 1e-6) {
        dq.setFromAxisAngle(new THREE.Vector3(wx / norm, wy / norm, wz / norm), theta);
      } else {
        dq.set(0, 0, 0, 1);
      }
      this.q.multiply(dq).normalize();
      this.P.ori += this.Q.gyro * dt;
      this.P.bias += this.Q.bias * dt;
    }

    update(measuredQuat) {
      // Error quaternion: measured * predicted^{-1}
      const qc = this.q.clone().conjugate();
      const qe = measuredQuat.clone().multiply(qc);
      // Small-angle approximation → 3-vector error
      const ex = qe.x, ey = qe.y, ez = qe.z;
      // Scalar Kalman gain (diagonal approx)
      const K = this.P.ori / (this.P.ori + this.R);
      // Apply correction as small rotation
      const corr = new THREE.Quaternion(ex * K, ey * K, ez * K, 1.0).normalize();
      this.q.multiply(corr).normalize();
      this.P.ori = (1 - K) * this.P.ori;
    }
  }

  // ─── Export ────────────────────────────────────────────────────────
  global.PoseFilters = {
    QuaternionFilter,
    AdaptivePositionFilter,
    PredictivePositionFilter,
    QuaternionEKF
  };

})(typeof window !== 'undefined' ? window : (typeof self !== 'undefined' ? self : (typeof globalThis !== 'undefined' ? globalThis : {})));
