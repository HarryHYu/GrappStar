// node review.js OUT -> OUT/review/*.png : close-ups for the design review (viewer.html, same AUD API as shots.js)
const { chromium } = require('playwright');
const http = require('http'), fs = require('fs'), path = require('path');
const OUT = path.resolve(process.argv[2] || 'out32');
const ROOT = path.resolve(__dirname);
const RD = path.join(OUT, 'review'); fs.mkdirSync(RD, { recursive: true });
const srv = http.createServer((q, s) => {
  let u = decodeURIComponent(q.url.split('?')[0]);
  let f = u === '/' || u === '/index.html' ? path.join(ROOT, 'viewer.html') : path.join(OUT, u);
  if (u.includes('three.min.js')) f = path.join(ROOT, 'vendor/three.min.js');
  fs.readFile(f, (e, d) => { if (e) { s.writeHead(404); s.end(); return; } s.writeHead(200); s.end(d); });
}).listen(8772);
(async () => {
  const b = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const p = await b.newPage({ viewport: { width: 1100, height: 760 } });
  await p.route('**/three.min.js', r => r.fulfill({ path: path.join(ROOT, 'vendor/three.min.js') }));
  await p.goto('http://localhost:8772/index.html');
  await p.waitForFunction(() => window.AUD && window.AUD.ready(), null, { timeout: 120000 });
  await p.evaluate(() => document.body.classList.add('shot'));
  const P = await p.evaluate(() => J.P);
  const K = P.KICK_DEG, D = P.DEPLOY_DEG, yc = P.DRILL_PIV_Y, L = P.L;
  const V = [
    ['01_folded_34_front', 0, 0, [85, 0, 10], 300, 2.3, 0.35], ['02_folded_34_rear', 0, 0, [90, 0, 10], 300, -0.75, 0.35],
    ['03_deployed_34_front', K, D, [50, 0, -5], 330, 2.4, 0.3], ['04_deployed_side', K, D, [50, 0, -10], 320, 1.5707, 0.05],
    ['05_pod_left_folded', 0, 0, [70, yc, 10], 120, 1.2, 0.35], ['06_pod_left_deployed', K, D, [80, yc, -10], 150, 1.2, 0.25],
    ['07_pod_right_folded_low', 0, 0, [70, -yc, 5], 120, -1.9, 0.1], ['08_kick_hinge', 0, 0, [32, 0, 25], 90, 2.2, 0.45],
    ['09_foot_folded', 0, 0, [130, 0, 40], 120, 1.0, 0.25], ['10_kick_planted', K, D, [0, 0, 0], 170, 2.0, 0.2],
    ['11_tail_boom', 0, 0, [175, 0, 0], 110, -0.6, 0.35], ['12_nose', 0, 0, [10, 0, 5], 110, 2.6, 0.2],
    ['13_top_folded', 0, 0, [90, 0, 0], 300, -1.5707, 1.5], ['14_under_deployed', K, D, [60, 0, 0], 330, -1.5707, -1.45]];
  for (const [n, k, d, t, r, a, e] of V) {
    await p.evaluate(([k, d, t, r, a, e]) => { AUD.flags(true, true, true); AUD.cut('y', 999); AUD.pose(k, d); AUD.cam(t, r, a, e); }, [k, d, t, r, a, e]);
    await p.waitForTimeout(500);
    await p.screenshot({ path: path.join(RD, n + '.png') });
  }
  await b.close(); srv.close();
})().catch(e => { console.error(e); process.exit(1); });
