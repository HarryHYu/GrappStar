// node simshot.js page.html out_prefix ts1,ts2,... [cam]
const { chromium } = require('playwright');
(async () => {
  const [, , page, pre, tss, cam] = process.argv;
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--allow-file-access-from-files'] });
  const p = await b.newPage({ viewport: { width: 1600, height: 1000 } });
  const errs = []; p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()) });
  await p.goto('file://' + __dirname + '/' + page);
  await p.waitForFunction('window.READY', null, { timeout: 120000 });
  await p.evaluate(() => window.SIM_PAUSE());
  if (cam) await p.evaluate(c => window.SIM_CAM(c), cam);
  for (const t of tss.split(',')) {
    await p.evaluate(x => window.SIM_SEEK(+x), t); await p.waitForTimeout(1500);
    await p.screenshot({ path: `${pre}_${t}.png` });
  }
  if (errs.length) console.log('errors', errs.slice(0, 6));
  await b.close();
})();
