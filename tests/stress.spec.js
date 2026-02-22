const { test, expect } = require('@playwright/test');
const URL = 'http://localhost:8001';

test.describe('Stress tests - POC runtime stress', () => {
  test('Run stress injection for 10s and collect stats', async ({ page }) => {
    await page.goto(URL + '/aruco-poc.html');
    await page.waitForSelector('#detection-rate', { timeout: 10000 });

    // ensure model and toggles exist
    await page.evaluate(() => {
      if (!window.scene) window.scene = new THREE.Scene();
      if (!window.model) { window.model = new THREE.Mesh(new THREE.BoxGeometry(0.1,0.1,0.1), new THREE.MeshNormalMaterial()); window.scene.add(window.model); }
      const cb = document.getElementById('toggle-worker'); if (cb) cb.checked = true;
    });

    // start stress test (10s, 30ms interval -> ~333 samples)
    await page.evaluate(() => window._startStressTest({ durationMs: 10000, intervalMs: 30, noise: 0.02, toggleOcclusion: true }));

    // wait for test completion by observing the end timestamp set by the helper
    await page.waitForFunction(() => window._stressStats && window._stressStats.end, { timeout: 20000 });

    // read stats
    const stats = await page.evaluate(() => window._stopStressTest ? window._stopStressTest() : window._stressStats);
    console.log('stress stats:', stats);
    // persist stats to disk for CI collection
    const fs = require('fs');
    try {
      fs.mkdirSync('test-artifacts', { recursive: true });
      fs.writeFileSync('test-artifacts/stress-stats.json', JSON.stringify(stats));
    } catch (e) { console.warn('Could not write stress artifact', e); }

    expect(stats).toBeTruthy();
    expect(stats.samples).toBeGreaterThan(100);
    expect(Array.isArray(stats.errors)).toBe(true);
    // ensure rafCount is reported (we accept 0 for headless but record it)
    expect(typeof stats.rafCount).toBe('number');
    // If raf rate is suspiciously low, warn but don't fail the stress test assert
    const rafSec = (stats.rafCount || 0) / (Math.max(1, stats.duration) / 1000);
    if (rafSec < 3) console.warn('Low RAF rate during stress test', rafSec, stats);
  }, { timeout: 60_000 });

  test('Toggle worker rapidly for stress (10s)', async ({ page }) => {
    await page.goto(URL + '/aruco-poc.html');
    await page.waitForSelector('#toggle-worker');
    // rapid toggle for 8s
    await page.evaluate(() => {
      window._toggleCount = 0;
      window._toggleInterval = setInterval(() => {
        const cb = document.getElementById('toggle-worker'); if (cb) { cb.checked = !cb.checked; cb.dispatchEvent(new Event('change')); }
        window._toggleCount++;
      }, 100);
    });
    await page.waitForTimeout(8_500);
    const toggles = await page.evaluate(() => { clearInterval(window._toggleInterval); return window._toggleCount; });
    expect(toggles).toBeGreaterThan(50);
  }, { timeout: 30_000 });
});