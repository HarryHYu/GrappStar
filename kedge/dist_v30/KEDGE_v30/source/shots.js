// node shots.js out27 -> out27/audit_sheet.png (24 angles) + out27/sections.png
const { chromium } = require('playwright');
const http = require('http'), fs = require('fs'), path = require('path');
const OUT = path.resolve(process.argv[2] || 'out27');
const ROOT = path.resolve(__dirname);
const srv = http.createServer((q, s) => {
  let u = decodeURIComponent(q.url.split('?')[0]);
  let f = u === '/' || u === '/index.html' ? path.join(ROOT, 'viewer.html') : path.join(OUT, u);
  if (u.includes('three.min.js')) f = path.join(ROOT, 'vendor/three.min.js');
  fs.readFile(f, (e, d) => { if (e) { s.writeHead(404); s.end(); return; } s.writeHead(200); s.end(d); });
}).listen(8771);
(async () => {
  const b = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'] });
  const p = await b.newPage({ viewport: { width: 640, height: 420 } });
  await p.route('**/three.min.js', r => r.fulfill({ path: path.join(ROOT, 'vendor/three.min.js') }));
  await p.goto('http://localhost:8771/index.html');
  await p.waitForFunction(() => window.AUD && window.AUD.ready(), null, { timeout: 120000 });
  await p.evaluate(() => document.body.classList.add('shot'));
  const P = await p.evaluate(() => J.P);
  const shots = [];
  const az = [0, 45, 90, 135, 180, 225, 270, 315].map(a => a * Math.PI / 180);
  // 8 folded, low elevation; 8 deployed, mid elevation; 8 mid-motion / special
  for (const a of az) shots.push(['fold az' + Math.round(a * 180 / Math.PI), 0, 0, [75, 0, 5], 300, a, 0.2]);
  for (const a of az) shots.push(['deployed az' + Math.round(a * 180 / Math.PI), P.KICK_DEG, P.DEPLOY_DEG, [45, 0, -5], 330, a, 0.3]);
  const mid = [[45, 0], [90, 0], [135, 0], [180, 0], [P.KICK_DEG, 12], [P.KICK_DEG, 25]];
  for (const [k, d] of mid) shots.push([`k${k} d${d}`, k, d, [45, 0, 5], 330, -1.57, 0.05]);
  shots.push(['top folded', 0, 0, [75, 0, 0], 300, -1.5707, 1.5]);
  shots.push(['under deployed', P.KICK_DEG, P.DEPLOY_DEG, [45, 0, 0], 330, -1.5707, -1.45]);
  const files = [];
  for (let i = 0; i < shots.length; i++) {
    const [n, k, d, t, r, a, e] = shots[i];
    await p.evaluate(([k, d, t, r, a, e]) => { AUD.flags(true, true, true); AUD.cut('y', 999); AUD.pose(k, d); AUD.cam(t, r, a, e); }, [k, d, t, r, a, e]);
    await p.waitForTimeout(350);
    const f = path.join(OUT, `_s${String(i).padStart(2, '0')}.png`); await p.screenshot({ path: f }); files.push([f, n]);
  }
  // sections
  const secs = [['Y=0 folded', 'y', 0.01, 0, 0, [75, 0, 0], 260, 1.5707, 0.0], ['Y=-8 deploy box', 'y', P.DEPLOY_Y, 0, 0, [75, 0, 0], 260, 1.5707, 0.0],
    ['Y=+13 kick hub', 'y', P.KICK_Y, 0, 0, [60, 0, 10], 220, 1.5707, 0.0], ['Y=29 pod deployed', 'y', 29.0, P.KICK_DEG, P.DEPLOY_DEG, [60, 0, -5], 260, 1.5707, 0.0],
    ['X=56 axle', 'x', 56.0, 0, 0, [56, 0, 5], 120, 0.0, 0.0], ['X=31 kick pivot', 'x', P.KICK_X, 0, 0, [31, 0, 10], 120, 0.0, 0.0],
    ['X boards', 'x', (P.X_BOARD[0]+P.X_BOARD[1])/2, 0, 0, [(P.X_BOARD[0]+P.X_BOARD[1])/2, 0, 0], 100, 0.0, 0.0], ['X cells', 'x', (P.X_CELL[0]+P.X_CELL[1])/2, 0, 0, [(P.X_CELL[0]+P.X_CELL[1])/2, 0, 10], 120, 0.0, 0.0],
    ['X=10 nose', 'x', 10.0, 0, 0, [10, 0, 0], 90, 0.0, 0.0], ['X tail', 'x', P.L-5, 0, 0, [P.L-5, 0, 0], 100, 0.0, 0.0],
    ['Z=7 axle level', 'z', 7.0, 0, 0, [75, 0, 7], 280, -1.5707, 1.5], ['Z=-10 low deck', 'z', -10.0, 0, 0, [75, 0, -10], 280, -1.5707, 1.5]];
  const sfiles = [];
  for (let i = 0; i < secs.length; i++) {
    const [n, ax, c, k, d, t, r, a, e] = secs[i];
    await p.evaluate(([ax, c, k, d, t, r, a, e]) => { AUD.flags(true, true, false); AUD.pose(k, d); AUD.cut(ax, c); AUD.cam(t, r, a, e); }, [ax, c, k, d, t, r, a, e]);
    await p.waitForTimeout(350);
    const f = path.join(OUT, `_c${String(i).padStart(2, '0')}.png`); await p.screenshot({ path: f }); sfiles.push([f, n]);
  }
  fs.writeFileSync(path.join(OUT, '_shots.json'), JSON.stringify({ shots: files, secs: sfiles }));
  await b.close(); srv.close();
})().catch(e => { console.error(e); process.exit(1); });
