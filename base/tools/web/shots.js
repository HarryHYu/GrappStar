// Headless screenshot driver: node shots.js job.json
// job: {url, width, height, routes: {url glob: local file} (served instead of the network), ready: "js expr" (default "window.AUD && AUD.ready()"), wait_ms,
//       shots: [{file, ops: [["fnName", ...args], ...], delay_ms}]}
// Each op calls window.AUD[fnName](...args) (or window[fnName] when AUD has no such function), then the page is
// captured. Prints a JSON line {ok, errors, info} at the end.
const { chromium } = require('playwright');
const fs = require('fs');

(async () => {
  const job = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  const page = await browser.newPage({ viewport: { width: job.width || 800, height: job.height || 600 }, deviceScaleFactor: job.scale || 1,
    colorScheme: job.color_scheme || 'light' });
  const errors = [];
  page.on('pageerror', e => errors.push('pageerror: ' + e.message));
  page.on('console', m => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
  for (const [pat, file] of Object.entries(job.routes || {})) await page.route(pat, r => r.fulfill({ path: file, contentType: 'application/javascript' }));
  await page.goto(job.url);
  await page.waitForFunction(job.ready || 'window.AUD && window.AUD.ready()', null, { timeout: job.timeout_ms || 120000 });
  if (job.wait_ms) await page.waitForTimeout(job.wait_ms);
  let info = null;
  try { info = await page.evaluate(() => (window.AUD && window.AUD.info) ? window.AUD.info() : null); } catch (e) { /* optional */ }
  for (const s of job.shots) {
    await page.evaluate(ops => {
      for (const [fn, ...args] of ops) {
        const f = (window.AUD && window.AUD[fn]) || window[fn];
        if (typeof f !== 'function') throw new Error('no function ' + fn);
        f(...args);
      }
    }, s.ops || []);
    await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
    if (s.delay_ms) await page.waitForTimeout(s.delay_ms);
    await page.screenshot({ path: s.file });
  }
  console.log(JSON.stringify({ ok: errors.length === 0, errors: errors.slice(0, 20), info }));
  await browser.close();
})().catch(e => { console.log(JSON.stringify({ ok: false, errors: [String(e && e.stack || e)] })); process.exit(1); });
