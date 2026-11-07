// node pubshots.js PUBDIR OUTDIR -> screenshots of the published viewer page (index.html + json) from several angles
const { chromium } = require('playwright');
const http = require('http'), fs = require('fs'), path = require('path');
const PUB = path.resolve(process.argv[2] || 'pub32'), OD = path.resolve(process.argv[3] || 'out32/review/pub');
fs.mkdirSync(OD, { recursive: true });
const srv = http.createServer((q, s) => {
  let u = decodeURIComponent(q.url.split('?')[0]); if (u === '/') u = '/index.html';
  fs.readFile(path.join(PUB, u), (e, d) => { if (e) { s.writeHead(404); s.end(); return; } s.writeHead(200); s.end(d); });
}).listen(8773);
(async () => {
  const b = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const p = await b.newPage({ viewport: { width: 1280, height: 800 } });
  await p.route('**/three.min.js', r => r.fulfill({ path: path.join(__dirname, 'vendor/three.min.js') }));
  await p.goto('http://localhost:8773/');
  await p.waitForFunction(() => window.AUD && window.AUD.ready(), null, { timeout: 120000 });
  await p.waitForTimeout(800);
  await p.screenshot({ path: path.join(OD, 'p0_page_default.png') });
  const P = await p.evaluate(() => P);
  const K = P.KICK_DEG, D = P.DEPLOY_DEG;
  const V = [['p1_folded_front34', 0, 0, [85, 0, 15], 320, 2.3, 0.35], ['p2_folded_rear34', 0, 0, [90, 0, 15], 320, -0.8, 0.35],
             ['p3_deployed_front34', K, D, [50, 0, -5], 340, 2.4, 0.3], ['p4_deployed_rear34', K, D, [60, 0, -5], 340, -0.9, 0.3],
             ['p5_deployed_side', K, D, [50, 0, -10], 320, 1.5707, 0.05], ['p6_folded_top', 0, 0, [90, 0, 0], 300, -1.5707, 1.5]];
  for (const [n, k, d, t, r, a, e] of V) {
    await p.evaluate(([k, d, t, r, a, e]) => { AUD.flags(true, true, true); AUD.cut('y', 999); AUD.pose(k, d); AUD.cam(t, r, a, e); }, [k, d, t, r, a, e]);
    await p.waitForTimeout(600);
    await p.screenshot({ path: path.join(OD, n + '.png') });
  }
  await b.close(); srv.close();
})().catch(e => { console.error(e); process.exit(1); });
