'use strict';
const path = require('path');
const root = process.env.AGV_ROOT || path.resolve(__dirname, '..');
const { chromium } = require(path.join(root, '.playwright/node_modules/playwright'));
const assert = (ok, msg) => { if (!ok) throw new Error(msg); };

(async () => {
  const browser = await chromium.launch({ headless: true });
  const errors = [], badResponses = [], failedRequests = [], posts = [];
  const page = await browser.newPage({ viewport: { width: 1366, height: 768 } });
  page.on('console', m => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', e => errors.push(e.message));
  page.on('response', r => { if (r.status() >= 400) badResponses.push(`${r.status()} ${r.url()}`); });
  page.on('requestfailed', r => failedRequests.push(`${r.url()} ${r.failure()?.errorText || ''}`));
  page.on('request', r => { if (r.method() === 'POST') posts.push(r.url()); });

  const response = await page.goto('http://127.0.0.1:5000', { waitUntil: 'domcontentloaded' });
  assert(response && response.status() === 200, 'index HTTP bukan 200');
  await page.waitForFunction(() => document.querySelector('#sseLabel')?.textContent?.includes('Live'), null, { timeout: 10000 });
  const health = await page.evaluate(() => fetch('/api/health').then(r => r.json()));
  assert(health.ok && health.ros && health.read_only, 'backend QA wajib ROS read-only');
  const apiState = await page.evaluate(() => fetch('/api/state').then(r => r.json()));
  for (const key of ['connected.camera','connected.imu','connected.gnss','connected.neo3_mag','connected.esc_ready','connected.vesc_transport'])
    assert(apiState[key] === true, `hardware state ${key} bukan connected`);
  const clickDomain = async domain => {
    await page.locator(`.top-domain[data-domain="${domain}"]`).click();
    await page.waitForTimeout(350);
    assert(await page.locator(`#page-${domain}`).evaluate(e => e.classList.contains('active')), `${domain} page tidak aktif`);
  };

  await clickDomain('esc');
  const leftGraph = await page.locator('#vescChartLeft').boundingBox();
  const rightGraph = await page.locator('#vescChartRight').boundingBox();
  assert(leftGraph?.width > 300 && leftGraph?.height > 100, 'grafik ESC kiri tidak terlihat');
  assert(rightGraph?.width > 300 && rightGraph?.height > 100, 'grafik ESC kanan tidak terlihat');

  await clickDomain('navigation');
  const map = await page.locator('#mapCanvas').boundingBox();
  assert(map?.width > 500 && map?.height > 300, 'map Navigation tidak terlihat/terlalu kecil');

  await clickDomain('perception');
  await page.waitForFunction(() => document.querySelector('#cameraImage')?.naturalWidth > 0, null, { timeout: 10000 });
  const camera = await page.locator('#cameraImage').evaluate(e => ({ w: e.naturalWidth, h: e.naturalHeight }));
  assert(camera.w >= 320 && camera.h >= 180, 'frame kamera tidak valid');

  await clickDomain('navigation');
  await page.locator('#workspaceTabs .workspace-tab', { hasText: 'Parameters' }).first().click();
  await page.waitForTimeout(350);
  assert(await page.locator('#page-configuration').evaluate(e => e.classList.contains('active')), 'Configuration tidak aktif');
  assert(await page.locator('#configGridBody tr').count() > 1, 'Configuration grid kosong');

  await clickDomain('navigation');
  await page.locator('#workspaceTabs .workspace-tab', { hasText: 'Tests' }).first().click();
  await page.waitForTimeout(350);
  assert(await page.locator('#page-experiments').evaluate(e => e.classList.contains('active')), 'Test workspace tidak aktif');
  assert(await page.locator('#testPhaseTabs .test-phase-tab').count() > 0, 'Test phase tab kosong');
  assert(await page.locator('#testFamilyTabs .test-family-tab').count() > 0, 'Test family tab kosong');

  await page.locator('.top-domain[data-domain="overview"]').click();
  await page.locator('#workspaceTabs .workspace-tab', { hasText: 'Data Health' }).first().click();
  await page.waitForTimeout(350);
  assert(await page.locator('#dataHealthRows tr').count() >= 8, 'Data Health tidak lengkap');
  assert(await page.locator('#dataHealthRows .health-state-stale').count() === 0, 'Data Health masih memiliki channel STALE');

  const mobile = await browser.newPage({ viewport: { width: 390, height: 844 } });
  await mobile.goto('http://127.0.0.1:5000', { waitUntil: 'domcontentloaded' });
  await mobile.waitForTimeout(1500);
  const width = await mobile.evaluate(() => ({ scroll: document.documentElement.scrollWidth, client: document.documentElement.clientWidth }));
  assert(width.scroll <= width.client + 1, `mobile horizontal overflow ${width.scroll}/${width.client}`);
  await mobile.locator('.top-domain[data-domain="navigation"]').click();
  await mobile.waitForTimeout(300);
  const mobileMap = await mobile.locator('#mapCanvas').boundingBox();
  assert(mobileMap?.width > 300 && mobileMap?.height > 250, 'mobile map tidak terlihat');
  await mobile.close();
  const report = {
    http: response.status(), title: await page.title(), sse: (await page.textContent('#sseLabel')).trim(),
    map: { width: Math.round(map.width), height: Math.round(map.height) },
    escGraphs: 2, camera, dataHealthRows: await page.locator('#dataHealthRows tr').count(),
    dataHealthSummary: (await page.textContent('#dataHealthSummary')).trim(),
    autonomyReady: apiState['system.autonomy_ready'] === true,
    mobile: width, errors, badResponses, failedRequests, automaticPosts: posts
  };
  console.log(JSON.stringify(report, null, 2));
  assert(errors.length === 0, `console/page errors: ${errors.join(' | ')}`);
  assert(badResponses.length === 0, `HTTP >=400: ${badResponses.join(' | ')}`);
  assert(failedRequests.length === 0, `request failed: ${failedRequests.join(' | ')}`);
  assert(posts.length === 0, `read-only UI mengirim POST otomatis: ${posts.join(' | ')}`);
  await browser.close();
  console.log('PASS qa_ros_web_playwright');
})().catch(async e => {
  console.error('FAIL qa_ros_web_playwright:', e.stack || e);
  process.exit(1);
});
