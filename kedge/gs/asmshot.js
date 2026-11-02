const { chromium } = require('playwright');
(async () => {
  const [, , page, pre, steps] = process.argv;
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const p = await b.newPage({ viewport: { width: 1500, height: 950 } });
  const errs = []; p.on('pageerror', e => errs.push(e.message));
  await p.goto('file://' + __dirname + '/' + page); await p.waitForFunction('window.READY', null, { timeout: 90000 });
  for (const s of steps.split(',')) { await p.evaluate(x => window.ASM_GO(+x), s); await p.waitForTimeout(3500); await p.screenshot({ path: `${pre}_${s}.png` }); }
  if (errs.length) console.log(errs.slice(0, 5)); await b.close();
})();
