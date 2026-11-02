// node snap.js out.png "query" [w h]
const { chromium } = require('playwright');
(async () => {
  const [, , out, q, w, h] = process.argv;
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--allow-file-access-from-files'] });
  const p = await b.newPage({ viewport: { width: +w || 1400, height: +h || 900 } });
  const errs = []; p.on('pageerror', e => errs.push(e.message)); p.on('console', m => { if (m.type() === 'error') errs.push(m.text()) });
  await p.goto('file://' + __dirname + '/inspect.html?' + (q || ''));
  await p.waitForFunction('window.READY', null, { timeout: 60000 }); await p.waitForTimeout(800);
  await p.screenshot({ path: out }); if (errs.length) console.log('errors', errs.slice(0, 5));
  await b.close();
})();
