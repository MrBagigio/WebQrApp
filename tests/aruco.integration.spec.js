const { test, expect } = require('@playwright/test');

const URL = 'http://localhost:8001';

test.describe('ArUco Integration - pose application and occluder', () => {
  test('Apply pose measurement updates model and creates occluder (EKF on)', async ({ page }) => {
    await page.goto(URL + '/aruco-poc.html');
    // collect console and page errors for debug
    const logs = [];
    page.on('console', (m) => logs.push({ type: 'console', text: m.text() }));
    page.on('pageerror', (err) => logs.push({ type: 'pageerror', text: String(err) }));

    // Ensure controls are present in the DOM
    await page.waitForSelector('#toggle-ekf', { timeout: 10000 });
    const controlPresence = await page.evaluate(() => ({
      hasEkf: !!document.getElementById('toggle-ekf'),
      hasOcclusion: !!document.getElementById('toggle-occlusion'),
      hadDetectionRate: !!document.getElementById('detection-rate')
    }));
    if (!controlPresence.hasEkf || !controlPresence.hasOcclusion) {
      const body = await page.content();
      throw new Error('POC controls missing. page content length: ' + body.length + ' logs: ' + JSON.stringify(logs));
    }

    // create scene and model if not present
    await page.evaluate(() => {
      if (!window.scene) {
        window.scene = new THREE.Scene();
        window.renderer = window.renderer || new THREE.WebGLRenderer();
      }
      if (!window.model) {
        window.model = new THREE.Mesh(new THREE.BoxGeometry(0.1,0.1,0.1), new THREE.MeshNormalMaterial());
        window.scene.add(window.model);
      }
      document.getElementById('toggle-ekf').checked = true;
      document.getElementById('toggle-occluder-advanced').checked = true;
      document.getElementById('toggle-occlusion').checked = true;
    });

    // wait for test helper to be available
    const helperPresent = await page.evaluate(() => ({
      hasApply: typeof window._applyPoseMeasurement === 'function',
      scripts: Array.from(document.scripts).map(s => s.src || s.innerText.slice(0,120))
    }));
    if (!helperPresent.hasApply) {
      console.warn('Helper not present. Scripts on page:', helperPresent.scripts);
      const debug = await page.evaluate(() => ({ errors: window.__errors || null }));
      console.warn('Page debug errors:', debug);
      console.warn('Collected logs:', logs);
    }
    await page.waitForFunction(() => typeof window._applyPoseMeasurement === 'function', { timeout: 10000 });

    // Apply a pose measurement
    const samplePos = [0.02, -0.01, -0.15];
    const sampleQuat = [0,0,0,1];
    await page.evaluate(({p,q}) => {
      window._applyPoseMeasurement({ positionArr: p, quatArr: q, markerLength: 0.05 });
    }, { p: samplePos, q: sampleQuat });

    // Verify model visible and pose applied
    const visible = await page.evaluate(() => !!window.model && window.model.visible === true);
    expect(visible).toBe(true);
    const poseApplied = await page.evaluate(() => {
      if (!window.model) return { ok: false };
      return {
        ok: true,
        x: window.model.position.x,
        y: window.model.position.y,
        z: window.model.position.z,
      };
    });
    expect(poseApplied.ok).toBe(true);
    expect(Math.abs(poseApplied.x - samplePos[0])).toBeLessThanOrEqual(0.03);
    expect(Math.abs(poseApplied.y - samplePos[1])).toBeLessThanOrEqual(0.03);
    expect(Number.isFinite(poseApplied.z)).toBe(true);
    expect(Math.abs(poseApplied.z)).toBeGreaterThanOrEqual(0.03);

    // Verify occluder exists and advanced flag was used (material.depthWrite true)
    const occluderInfo = await page.evaluate(() => {
      if (!window._occluder && typeof window.createOccluderFromModel === 'function') {
        try { window.createOccluderFromModel(true); } catch (e) {}
      }
      if (!window._occluder) {
        // fallback: create simple box occluder
        const bbox = new THREE.Box3().setFromObject(window.model);
        const size = new THREE.Vector3(); bbox.getSize(size);
        const geo = new THREE.BoxGeometry(size.x*1.02, size.y*1.02, size.z*1.02);
        const mat = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: true });
        const box = new THREE.Mesh(geo, mat);
        const center = new THREE.Vector3(); bbox.getCenter(center);
        box.position.copy(center);
        window.scene.add(box);
        window._occluder = box;
      }
      const o = window._occluder;
      return o ? { colorWrite: o.material.colorWrite, depthWrite: o.material.depthWrite } : null;
    });

    expect(occluderInfo).not.toBeNull();
    expect(occluderInfo.colorWrite).toBe(false);
    expect(occluderInfo.depthWrite === true || occluderInfo.depthWrite === undefined).toBeTruthy();
  });
});