// Headless screenshot driver: node shots.js job.json
// job: {url, width, height, routes: {url glob: local file} (served instead of the network), ready: "js expr" (default "window.AUD && AUD.ready()"), wait_ms,
//       freeze (default true), eval: "js expr" (its JSON value is returned as result),
//       shots: [{file, ops: [["fnName", ...args], ...], delay_ms}]}
// The page runs with window.CAD_SHOT = true (pixel ratio 1, smaller shadow map). Each op calls window.AUD[fnName](...args)
// (or window[fnName] when AUD has no such function); then the pose is frozen (AUD.freeze: render once, stop redrawing)
// and the page is captured. Prints a JSON line {ok, errors, info, result} at the end.
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
  await page.addInitScript(() => { window.CAD_SHOT = true; });
  for (const [pat, file] of Object.entries(job.routes || {})) await page.route(pat, r => r.fulfill({ path: file, contentType: 'application/javascript' }));
  await page.goto(job.url);
  await page.waitForFunction(job.ready || 'window.AUD && window.AUD.ready()', null, { timeout: job.timeout_ms || 120000 });
  if (job.wait_ms) await page.waitForTimeout(job.wait_ms);
  let info = null;
  try { info = await page.evaluate(() => (window.AUD && window.AUD.info) ? window.AUD.info() : null); } catch (e) { /* optional */ }
  const freeze = job.freeze !== false;
  const setFreeze = on => page.evaluate(on => { if (window.AUD && window.AUD.freeze) window.AUD.freeze(on); }, on);
  for (const s of (job.shots || [])) {
    if (freeze) await setFreeze(false);
    await page.evaluate(ops => {
      for (const [fn, ...args] of ops) {
        const f = (window.AUD && window.AUD[fn]) || window[fn];
        if (typeof f !== 'function') throw new Error('no function ' + fn);
        f(...args);
      }
    }, s.ops || []);
    await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
    if (s.delay_ms) await page.waitForTimeout(s.delay_ms);
    if (freeze) await setFreeze(true);
    await page.screenshot({ path: s.file, timeout: job.shot_timeout_ms || 180000 });
  }
  let result = null;
  if (job.eval) result = await page.evaluate(job.eval);
  console.log(JSON.stringify({ ok: errors.length === 0, errors: errors.slice(0, 20), info, result }));
  await browser.close();
})().catch(e => { console.log(JSON.stringify({ ok: false, errors: [String(e && e.stack || e)] })); process.exit(1); });
