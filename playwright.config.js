// Playwright config with webServer to serve static files during tests
/** @type {import('@playwright/test').PlaywrightTestConfig} */
module.exports = {
  testDir: './tests',
  testIgnore: ['**/tests/unit/**', '**/tests-unit/**'],
  timeout: 30_000,
  expect: { timeout: 5000 },
  reporter: [['list'], ['html']],
  use: {
    headless: true,
    viewport: { width: 1280, height: 800 }
  },
  webServer: {
    command: 'npx http-server . -p 8001',
    port: 8001,
    reuseExistingServer: false,
    timeout: 30_000
  }
};