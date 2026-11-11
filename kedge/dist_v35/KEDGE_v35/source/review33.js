// node review33.js OUT [views.json] -> OUT/review/*.png : v33 close-ups (same viewer API as review.js). views.json: [[name, kick, deploy,
// [tx,ty,tz], dist, azimuth, elevation, cutAxis?, cutVal?], ...]; without it the default v33 list below. K / D in a view = the solved angles.
const { chromium } = require('playwright');
const http = require('http'), fs = require('fs'), path = require('path');
const OUT = path.resolve(process.argv[2] || 'out33');
const ROOT = path.resolve(__dirname);
const RD = path.join(OUT, 'review'); fs.mkdirSync(RD, { recursive: true });
const srv = http.createServer((q, s) => {
  let u = decodeURIComponent(q.url.split('?')[0]);
  let f = u === '/' || u === '/index.html' ? path.join(ROOT, 'viewer.html') : path.join(OUT, u);
  fs.readFile(f, (e, d) => { if (e) { s.writeHead(404); s.end(); return; } s.writeHead(200); s.end(d); });
}).listen(8774);
(async () => {
  const b = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const p = await b.newPage({ viewport: { width: 1100, height: 760 } });
  await p.route('**/three.min.js', r => r.fulfill({ path: path.join(ROOT, 'vendor/three.min.js') }));
  await p.goto('http://localhost:8774/index.html');
  await p.waitForFunction(() => window.AUD && window.AUD.ready(), null, { timeout: 120000 });
  await p.evaluate(() => document.body.classList.add('shot'));
  const P = await p.evaluate(() => J.P);
  const K = P.KICK_DEG, D = P.DEPLOY_DEG, yc = P.DRILL_PIV_Y, tip = P.DRILL_X + P.POD_TIP, hub = P.DRILL_X + P.POD_XS;
  let V = [
    ['15_auger_tip_folded', 0, 0, [tip - 12, yc, 12], 70, 1.0, 0.35], ['16_auger_hub_folded', 0, 0, [hub + 4, yc, 12], 70, 1.3, 0.3],
    ['17_auger_right_low', 0, 0, [hub + 20, -yc, 8], 120, -1.9, -0.05], ['18_pods_deployed_front_low', K, D, [60, 0, -12], 200, 3.0, 0.08],
    ['19_pods_deployed_rear', K, D, [70, 0, -10], 230, -0.6, 0.2], ['20_folded_front_axis', 0, 0, [80, 0, 5], 260, 3.1416, 0.02],
    ['21_folded_rear_axis', 0, 0, [80, 0, 5], 260, 0.0, 0.02], ['22_folded_side_left', 0, 0, [85, 0, 20], 280, 1.5707, 0.02],
    ['23_folded_high', 0, 0, [85, 0, 10], 300, 0.9, 0.9], ['24_foot_gimlet', 0, 0, [150, 0, 70], 90, 1.2, 0.2],
    ['25_pod_section', 0, 0, [85, yc, 12], 150, -1.5707, 1.45, 'z', 12.0]];
  if (process.argv[3]) V = JSON.parse(fs.readFileSync(process.argv[3], 'utf8'));
  for (const v of V) {
    let [n, k, d, t, r, a, e, ca, cv] = v;
    k = k === 'K' ? K : k; d = d === 'D' ? D : d;
    await p.evaluate(([k, d, t, r, a, e, ca, cv]) => { AUD.flags(true, true, true); AUD.cut(ca || 'y', ca ? cv : 999); AUD.pose(k, d); AUD.cam(t, r, a, e); }, [k, d, t, r, a, e, ca, cv]);
    await p.waitForTimeout(500);
    await p.screenshot({ path: path.join(RD, n + '.png') });
  }
  await b.close(); srv.close();
})().catch(e => { console.error(e); process.exit(1); });
