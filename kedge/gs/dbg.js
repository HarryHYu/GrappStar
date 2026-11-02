const { chromium } = require('playwright');
(async () => {
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const p = await b.newPage({ viewport: { width: 800, height: 600 } });
  p.on('pageerror', e => console.log('ERR', e.message));
  await p.goto('file://' + __dirname + '/sim_dev.html'); await p.waitForFunction('window.READY', null, { timeout: 60000 });
  await p.waitForTimeout(1000);
  console.log(await p.evaluate(() => JSON.stringify({ L0: GS.P.SPRING_L0, PRE: GS.P.SPRING_PRE, A: GS.P.ANCHOR_U, vis: ['spring_env_R','cord_env_R'].map(k => !!GS.parts.find(q => q.name === k)) })));
  await b.close();
})();
