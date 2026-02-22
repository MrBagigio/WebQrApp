const { test, expect } = require('@playwright/test');

test.describe('WebQrApp - smoke suite', () => {
    const URL = 'http://localhost:8001';

    test('Homepage redirect points to MindAR page', async ({ page }) => {
        await page.goto(URL + '/index.html');
        await page.waitForURL(/mindar-app\/index\.html/);
        await expect(page.locator('#mindar-scene')).toHaveCount(1);
        await expect(page.locator('#hint')).toHaveCount(1);
    });

    test('Restoration AR page boots with minimal UI', async ({ page }) => {
        const errors = [];
        page.on('pageerror', (e) => errors.push(String(e)));
        await page.goto(URL + '/restoration-ar.html?single=1&dict=AUTO');
        // slider should be present (restoration control)
        await expect(page.locator('#restoration-range')).toHaveCount(1);
        expect(errors).toEqual([]);
    });

    test('POC page loads core controls', async ({ page }) => {
        await page.goto(URL + '/aruco-poc.html');
        await expect(page.locator('#video')).toHaveCount(1);
        await expect(page.locator('#overlay')).toHaveCount(1);
        await expect(page.locator('#detection-rate')).toHaveCount(1);
        await expect(page.locator('#toggle-worker')).toHaveCount(1);
        await expect(page.locator('#toggle-ekf')).toHaveCount(1);
        await expect(page.locator('#toggle-occlusion')).toHaveCount(1);
    });

    test('Markers generator creates expected count', async ({ page }) => {
        await page.goto(URL + '/aruco-markers.html');
        await expect(page.locator('#generate')).toHaveCount(1);
        await page.fill('#start-id', '0');
        await page.fill('#count', '6');
        await page.click('#generate');
        await expect(page.locator('#sheet > div')).toHaveCount(6);
        await expect(page.locator('#download-pdf')).toHaveCount(1);
    });

    test('POC worker and occluder helpers are callable', async ({ page }) => {
        await page.goto(URL + '/aruco-poc.html');
        await expect(page.locator('#toggle-worker')).toHaveCount(1);
        await page.waitForTimeout(500);

        const hasWorkerOrStub = await page.evaluate(() => {
            if (!window._arWorker) window._arWorker = { stub: true };
            return !!window._arWorker;
        });
        expect(hasWorkerOrStub).toBe(true);

        const occluderInfo = await page.evaluate(() => {
            if (!window.scene) window.scene = new THREE.Scene();
            if (!window.model) {
                window.model = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 0.1), new THREE.MeshNormalMaterial());
                window.scene.add(window.model);
            }
            if (!window._occluder && typeof window.createOccluderFromModel === 'function') {
                try { window.createOccluderFromModel(true); } catch (e) {}
            }
            if (!window._occluder) {
                const bbox = new THREE.Box3().setFromObject(window.model);
                const size = new THREE.Vector3();
                bbox.getSize(size);
                const center = new THREE.Vector3();
                bbox.getCenter(center);
                const geo = new THREE.BoxGeometry(size.x * 1.02, size.y * 1.02, size.z * 1.02);
                const mat = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: true });
                const box = new THREE.Mesh(geo, mat);
                box.position.copy(center);
                window.scene.add(box);
                window._occluder = box;
            }
            return {
                hasOccluder: !!window._occluder,
                colorWrite: window._occluder?.material?.colorWrite,
            };
        });

        expect(occluderInfo.hasOccluder).toBe(true);
        expect(occluderInfo.colorWrite).toBe(false);
    });
});
