// node review33.js OUT [views.json] -> OUT/review/*.png : v33 close-ups (same viewer API as review.js). views.json: [[name, kick, deploy,
// [tx,ty,tz], dist, azimuth, elevation, cutAxis?, cutVal?], ...]; without it the default v33 list below. K / D in a view = the solved angles.
const { chromium } = require('playwright');
const http = require('http'), fs = require('fs'), path = require('path');
const OUT = path.resolve(process.argv[2] || 'out34');
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
  const K = P.KICK_DEG, D = P.DEPLOY_DEG, yc = P.DRILL_PIV_Y, dx = P.DRILL_X, lx = P.LEG_X;
  let V = [
    ['01_folded_34_front', 0, 0, [80, 0, 5], 300, 2.3, 0.35], ['02_folded_34_rear', 0, 0, [85, 0, 5], 300, -0.75, 0.35],
    ['03_deployed_34_front', 0, D, [90, 0, -8], 320, 2.4, 0.28], ['04_deployed_side', 0, D, [90, 0, -10], 320, 1.5707, 0.05],
    ['05_leg_178_side', K, 0, [20, 0, 10], 360, 1.5707, 0.08], ['06_leg_90_34', 90, 0, [60, 0, 40], 330, 2.2, 0.3],
    ['07_hinge_close', 0, 0, [33, 0, 15], 70, 2.3, 0.55], ['08_hinge_close_rear', 0, 0, [36, 0, 15], 70, -0.6, 0.6],
    ['09_tail_pad', 0, 0, [142, 0, 10], 90, -0.7, 0.5], ['10_pod_rear_folded', 0, 0, [130, yc, 5], 90, 0.9, 0.3],
    ['11_pod_rear_deployed', 0, D, [135, yc, -5], 110, 0.9, 0.2], ['12_nose', 0, 0, [10, 0, 5], 100, 2.6, 0.25],
    ['13_top_folded', 0, 0, [85, 0, 0], 290, -1.5707, 1.5], ['14_under_deployed', 0, D, [95, 0, 0], 320, -1.5707, -1.45],
    ['15_tail_end', 0, 0, [150, 0, 0], 110, 0.0, 0.15], ['16_flank_sockets', 0, 0, [146, 18, 0], 70, 1.2, 0.15],
    ['17_leg_45_hinge', 45, 0, [35, 0, 22], 110, 2.0, 0.35], ['18_leg_178_nose', K, 0, [15, 0, 12], 110, 2.4, 0.35],
    ['19_section_y0_folded', 0, 0, [80, 0, 0], 300, -1.5707, 0.02, 'y', 0.0], ['20_section_pivot', 0, 0, [dx, 0, 0], 120, 0.0, 0.0, 'x', dx],
    ['21_section_hinge', 0, 0, [lx, 0, 8], 90, 0.0, 0.0, 'x', lx], ['22_pods_deployed_rear_low', 0, D, [120, 0, -12], 200, -0.4, 0.05]];
  if (process.argv[3]) V = JSON.parse(fs.readFileSync(process.argv[3], 'utf8'));
  for (const v of V) {
    let [n, k, d, t, r, a, e, ca, cv, hide, flags] = v;
    k = k === 'K' ? K : k; d = d === 'D' ? D : d;
    await p.evaluate(([k, d, t, r, a, e, ca, cv, hide, flags]) => { const f = flags || [true, true, true]; AUD.flags(f[0], f[1], f[2]); AUD.hide(hide || []); AUD.cut(ca || 'y', ca ? cv : 999); AUD.pose(k, d); AUD.cam(t, r, a, e); }, [k, d, t, r, a, e, ca, cv, hide, flags]);
    await p.waitForTimeout(500);
    await p.screenshot({ path: path.join(RD, n + '.png') });
  }
  await b.close(); srv.close();
})().catch(e => { console.error(e); process.exit(1); });
