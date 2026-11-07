#!/usr/bin/env node
// Browser test of the KEDGE dart web page (the PAGE string in ../kedge_dart/kedge_dart.ino) against the host simulator.
//
//   make && node test_page.js
//
// Starts ./kedge_sim on a free port, opens the page in headless Chromium (Playwright) and drives it with real pointer
// and keyboard input. Motor directions are read from the simulated H-bridge pins via /sim/state, so a check passes
// only if the motor really ran or really stopped. Simulator output is saved to build/test_page.log.
// Needs Node 18+ (global fetch) and the playwright package; nothing else.
//
// Each check prints PASS, FAIL, XFAIL (a known page or firmware bug, explained) or XPASS (a known bug that no longer
// shows: remove the xfail mark). The exit code is 1 if any check FAILs or XPASSes, else 0.
'use strict';
const { chromium } = require('playwright');
// the dart's event for the page's key-release stop (/key?c=%20&keep=1): stops manual motors only (kedge_logic.h keyRelease)
const KEY_STOP_EV = 'keys released, motors stopped';
const { spawn, execFileSync } = require('child_process');
const fs = require('fs');
const net = require('net');
const path = require('path');

const HERE = __dirname;
const SIM = path.join(HERE, 'kedge_sim');
const LOG = path.join(HERE, 'build', 'test_page.log');
const MOTORS = ['deploy', 'kick', 'drills', 'foot'];

let port, base, sim, browser, page;
const consoleMsgs = [], pageErrors = [];
const results = [];

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const now = () => Date.now();

function freePort() {
  return new Promise((resolve, reject) => {
    const s = net.createServer();
    s.once('error', reject);
    s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => resolve(p)); });
  });
}

// ---------------------------------------------------------------------------------------------------- HTTP helpers
async function get(p) {
  const r = await fetch(base + p);
  return { code: r.status, body: await r.text() };
}
async function simState() { return JSON.parse((await get('/sim/state')).body); }
async function status() { return JSON.parse((await get('/status')).body); }
async function simSet(q) { const r = await get('/sim/set?' + q); if (r.code !== 200) throw new Error('sim/set ' + q + ': ' + r.body); }
async function dirs() {
  const st = await simState();
  const d = {};
  for (const m of MOTORS) d[m] = st.motors[m].dir;
  return d;
}
const allStopped = (d) => MOTORS.every((m) => d[m] === 0);
const fmtDirs = (d) => MOTORS.map((m) => m + '=' + d[m]).join(' ');

function assert(cond, msg) { if (!cond) throw new Error(msg); }

// poll /sim/state until pred(dirs) holds; returns ms waited
async function waitDirs(pred, timeoutMs, what) {
  const t0 = now(); let d;
  for (;;) {
    d = await dirs();
    if (pred(d)) return now() - t0;
    if (now() - t0 > timeoutMs) throw new Error(`timed out after ${timeoutMs} ms waiting for ${what}; motors ${fmtDirs(d)}`);
    await sleep(15);
  }
}
// pred(dirs) must hold on every sample for durMs; returns the number of samples
async function holdsFor(pred, durMs, what) {
  const t0 = now(); let n = 0;
  while (now() - t0 < durMs) {
    const d = await dirs(); n++;
    if (!pred(d)) throw new Error(`${what} broke after ${now() - t0} ms; motors ${fmtDirs(d)}`);
    await sleep(20);
  }
  return n;
}
async function waitPage(fn, timeoutMs, what) {
  const t0 = now(); let v;
  for (;;) {
    v = await fn();
    if (v.ok) return now() - t0;
    if (now() - t0 > timeoutMs) throw new Error(`timed out after ${timeoutMs} ms waiting for ${what}; last: ${JSON.stringify(v.last)}`);
    await sleep(50);
  }
}
const statusLine = () => page.textContent('#s');
const logText = () => page.textContent('#o');

// all motors off, plant back at the start, and the "repeat" windows in the firmware (deadman + 150 ms) run out
async function settle(q = 'kick_pos=0&deploy_pos=0') {
  await get('/stop');
  if (q) await simSet(q);
  await sleep(600);
  assert(allStopped(await dirs()), 'motors did not settle');
}

// ---------------------------------------------------------------------------------------------------- page helpers
async function button(name) {
  const b = page.getByRole('button', { name, exact: true });
  const box = await b.boundingBox();
  assert(box, 'no button ' + name);
  return { x: box.x + box.width / 2, y: box.y + box.height / 2 };
}
async function pressButton(name) { const c = await button(name); await page.mouse.move(c.x, c.y); await page.mouse.down(); }
async function releaseButton() { await page.mouse.up(); }
async function clickButton(name) { const c = await button(name); await page.mouse.click(c.x, c.y); }
async function heldSize() { return page.evaluate('held.size'); }

async function openPage() {
  await page.goto(base + '/');
  await page.waitForSelector('#b button');
  await page.waitForFunction(() => document.getElementById('s').textContent.length > 0, null, { timeout: 3000 });
}

// ---------------------------------------------------------------------------------------------------- runner
async function check(name, fn, xfail) {
  const t0 = now();
  const notes = [];
  try {
    await fn((s) => notes.push(s));
    results.push({ name, res: xfail ? 'XPASS' : 'PASS', notes, ms: now() - t0, xfail });
  } catch (e) {
    results.push({ name, res: xfail ? 'XFAIL' : 'FAIL', notes, ms: now() - t0, err: e.message, xfail });
  }
  const r = results[results.length - 1];
  console.log(`${r.res.padEnd(5)} ${name} (${(r.ms / 1000).toFixed(1)} s)`);
  for (const n of notes) console.log('        ' + n);
  if (r.err) console.log('        error: ' + r.err);
  if (r.res === 'XFAIL' || r.res === 'XPASS') console.log('        known bug: ' + xfail);
  // never leave anything held into the next check
  try { await page.mouse.up(); } catch (e) { /* ignore */ }
  for (const k of ['Digit1', 'Digit2', 'Digit3', 'Digit4', 'Shift']) { try { await page.keyboard.up(k); } catch (e) { /* ignore */ } }
  try { await settle(); } catch (e) { console.log('        settle: ' + e.message); }
}

// ---------------------------------------------------------------------------------------------------- checks
async function check1(note) {
  await pressButton('kick out');
  const tOn = await waitDirs((d) => d.kick === 1, 500, 'kick out to start');
  note(`kick started ${tOn} ms after pointerdown`);
  const pos0 = (await simState()).kick_pos;
  const n = await holdsFor((d) => d.kick === 1 && d.deploy === 0 && d.drills === 0 && d.foot === 0, 2000, 'kick running while held');
  const pos1 = (await simState()).kick_pos;
  note(`kick ran on all ${n} samples during the 2 s hold, kick_pos ${pos0} -> ${pos1}`);
  assert(pos1 - pos0 > 1000, `kick hardly moved in 2 s (${pos0} -> ${pos1})`);
  await get('/arm?on=0');                            // marks the event log: a "stop" event after this came from the page
  await releaseButton();
  const tOff = await waitDirs((d) => d.kick === 0, 500, 'kick to stop after pointerup');
  const ev = (await status()).event;
  note(`kick stopped ${tOff} ms after pointerup; last event "${ev}"`);
  assert(ev === KEY_STOP_EV, `release did not send the key-release stop (event "${ev}"): the kick only stopped on the deadman`);
  await holdsFor(allStopped, 500, 'all stopped after release');
  assert((await heldSize()) === 0, 'page still thinks something is held');
}

// both motors at once: the firmware's pair check (kedge_logic.h, pairCheck) stops the worm motor (here the kick) for
// 150 ms every 550 ms to read the foot's current alone, so the kick's pins read "off" for those 150 ms while it is
// still commanded on. A sample therefore counts as "kick running" when /status says kick=1 (commanded) and the pins
// are either driving it or in such a short pause; "kick stopped" needs both the command and the pins off.
async function both() {
  const [st, s] = await Promise.all([simState(), status()]);
  const pin = {}; for (const m of MOTORS) pin[m] = st.motors[m].dir;
  return { pin, cmd: s.motors };
}
async function check2(note) {
  await simSet('kick_pos=0');
  await page.keyboard.down('Digit2');
  await page.keyboard.down('Digit4');
  const tOn = await waitDirs((d) => d.kick === 1 && d.foot === 1, 500, 'kick and foot to start');
  note(`kick and foot both running ${tOn} ms after the keydowns`);
  let t0 = now(), n = 0, pauseStart = null, maxPause = 0, pauses = 0;
  while (now() - t0 < 1500) {
    const { pin, cmd } = await both(); n++;
    assert(cmd.kick === 1 && cmd.foot === 1, `kick or foot no longer commanded while both held: cmd ${JSON.stringify(cmd)}`);
    assert(pin.foot === 1, `foot pins off while held: ${fmtDirs(pin)}`);
    assert(pin.kick === 1 || pin.kick === 0, `kick reversed: ${fmtDirs(pin)}`);
    if (pin.kick === 0) { if (pauseStart === null) { pauseStart = now(); pauses++; } maxPause = Math.max(maxPause, now() - pauseStart); }
    else pauseStart = null;
    assert(maxPause <= 250, `kick pins off for ${maxPause} ms while held (pair check pauses are 150 ms)`);
    await sleep(15);
  }
  note(`over 1.5 s (${n} samples) both stayed commanded on; foot pins always on; kick pins paused ${pauses}x, longest ${maxPause} ms (the pair check)`);
  await page.keyboard.up('Digit2');
  const tR = now();
  for (;;) {
    const { pin, cmd } = await both();
    assert(cmd.foot === 1 && pin.foot === 1, `releasing 2 stopped the foot too: cmd ${JSON.stringify(cmd)} pins ${fmtDirs(pin)}`);
    if (cmd.kick === 0 && pin.kick === 0) break;
    assert(now() - tR < 500, `kick still commanded ${now() - tR} ms after releasing 2`);
    await sleep(15);
  }
  note(`kick stopped ${now() - tR} ms after releasing 2 (no more repeats, so the 350 ms deadman ends it); foot still running`);
  const n2 = await holdsFor((d) => d.kick === 0 && d.foot === 1, 700, 'foot alone running while 4 held');
  note(`foot alone kept running on all ${n2} samples over 0.7 s`);
  await get('/arm?on=0');                            // marks the event log: a "stop" event after this came from the page
  await page.keyboard.up('Digit4');
  const tAll = await waitDirs(allStopped, 500, 'all to stop after releasing 4');
  const ev = (await status()).event;
  note(`all stopped ${tAll} ms after releasing 4; last event "${ev}"`);
  assert(ev === KEY_STOP_EV, `releasing the last key did not send the key-release stop (event "${ev}")`);
  await holdsFor(allStopped, 500, 'all stopped after releasing 4');
}

async function check3(note) {
  await simSet('kick_pos=2500');                     // "kick back" needs room to travel
  await sleep(100);
  await page.keyboard.down('Shift');
  await page.keyboard.down('Digit2');                // key '@' = kick back
  const tOn = await waitDirs((d) => d.kick === -1, 500, 'kick back to start on Shift+2');
  note(`Shift+2 ('@') started kick back after ${tOn} ms`);
  await holdsFor((d) => d.kick === -1, 500, 'kick back running');
  await page.keyboard.up('Shift');                   // Shift first: keyup of 2 will now carry e.key '2', not '@'
  await holdsFor((d) => d.kick === -1, 500, 'kick back still running after Shift released (2 still held)');
  note('kick back kept running after Shift was released with 2 still down');
  await get('/arm?on=0');
  await page.keyboard.up('Digit2');
  const tOff = await waitDirs((d) => d.kick === 0, 500, 'kick to stop after releasing 2');
  note(`kick stopped ${tOff} ms after releasing 2; last event "${(await status()).event}"`);
  await holdsFor(allStopped, 700, 'all stopped after release (no stuck repeat)');
  assert((await heldSize()) === 0, 'page still holds the key after keyup');
}

// a real window blur: focus moves into an iframe, so Chromium itself fires blur on the page's window
async function realBlur() {
  await page.evaluate(() => new Promise((res) => {
    const f = document.createElement('iframe'); f.id = 'blurframe'; f.srcdoc = '<p>x</p>';
    f.onload = () => { f.contentWindow.focus(); res(); };
    document.body.appendChild(f);
  }));
}
async function undoBlur() {
  await page.evaluate(() => { const f = document.getElementById('blurframe'); if (f) f.remove(); window.focus(); });
}

async function check4a(note) {
  let blurs = 0;
  await page.exposeFunction('__noteBlur', () => { blurs++; }).catch(() => {});
  await page.evaluate(() => addEventListener('blur', () => window.__noteBlur && window.__noteBlur()));
  await pressButton('foot in');
  await waitDirs((d) => d.foot === 1, 500, 'foot to start');
  await holdsFor((d) => d.foot === 1, 400, 'foot running while held');
  await get('/arm?on=0');
  await realBlur();
  const tOff = await waitDirs((d) => d.foot === 0, 500, 'foot to stop after blur');
  await sleep(50);
  note(`window blur (focus moved into an iframe, browser-fired: ${blurs} blur event) stopped the foot after ${tOff} ms; last event "${(await status()).event}"`);
  await holdsFor(allStopped, 700, 'all stopped while the pointer is still down after blur');
  await releaseButton();
  await undoBlur();
}

async function check4b(note) {
  await page.keyboard.down('Digit4');
  await waitDirs((d) => d.foot === 1, 500, 'foot to start');
  await holdsFor((d) => d.foot === 1, 400, 'foot running while held');
  await get('/arm?on=0');
  // headless Chromium never hides a page, so visibility is simulated: document.hidden true + a visibilitychange event
  await page.evaluate(() => {
    Object.defineProperty(document, 'hidden', { configurable: true, get: () => true });
    Object.defineProperty(document, 'visibilityState', { configurable: true, get: () => 'hidden' });
    document.dispatchEvent(new Event('visibilitychange'));
  });
  const tOff = await waitDirs((d) => d.foot === 0, 500, 'foot to stop after visibilitychange');
  note(`page hidden (simulated visibilitychange) stopped the foot after ${tOff} ms; last event "${(await status()).event}"`);
  await holdsFor(allStopped, 700, 'all stopped while the key is still down after hide');
  await page.evaluate(() => { delete document.hidden; delete document.visibilityState; });
  await page.keyboard.up('Digit4');
}

// a real pagehide: navigate away while key 1 (deploy down) is held. The firmware's deadman (350 ms) stops the motor
// whatever the page does; how far the deploy leg travelled after the navigation tells how long it really ran (the HTTP
// server can be busy meanwhile, see the report), and the event log tells whether the page's own STOP got out
const pagehideSeen = {};
async function check4c(note) {
  await page.keyboard.down('Digit1');
  await waitDirs((d) => d.deploy === 1, 500, 'deploy to start');
  await holdsFor((d) => d.deploy === 1, 300, 'deploy running while held');
  await get('/arm?on=0');
  const st0 = await simState();
  const t0 = now();
  await page.goto('about:blank');
  const tNav = now() - t0;
  const ta = now();
  const st1 = await simState();
  const tHttp = now() - ta;
  pagehideSeen.http = tHttp;
  await sleep(300);
  const st2 = await simState();
  const ev = (await status()).event;
  const rate = st0.motors.deploy.duty / 255;         // deploy_pos units per ms at this duty (sim_world.cpp, advance)
  const ranMs = Math.round((st2.deploy_pos - st0.deploy_pos) / rate);
  pagehideSeen.ev = ev;
  note(`navigated away (real pagehide) in ${tNav} ms; the next /sim/state took ${tHttp} ms to answer`);
  note(`deploy_pos ${st0.deploy_pos} -> ${st1.deploy_pos} -> ${st2.deploy_pos}: it ran about ${ranMs} ms after the last reading before the navigation; last event "${ev}"`);
  assert(st2.motors.deploy.dir === 0 && st1.deploy_pos === st2.deploy_pos, 'deploy still running after navigating away');
  assert(ranMs <= 600, `deploy ran ${ranMs} ms after the page went away`);
  await page.keyboard.up('Digit1');
  await openPage();
}
// the same navigation, 6 times: does the STOP sent from the pagehide handler reach the dart? (a race in the browser,
// so one try proves little). Also times the first HTTP answer after the navigation.
async function check4d(note) {
  const trials = [];
  if (pagehideSeen.ev !== undefined) trials.push({ ev: pagehideSeen.ev, http: pagehideSeen.http });
  while (trials.length < 6) {
    await page.keyboard.down('Digit4');
    await waitDirs((d) => d.foot === 1, 500, 'foot to start');
    await sleep(200 + 40 * trials.length);
    await get('/arm?on=0');
    await page.goto('about:blank');
    const ta = now();
    const ev = (await status()).event;
    trials.push({ ev, http: now() - ta });
    await page.keyboard.up('Digit4');
    await settle(null);
    await openPage();
  }
  const got = trials.filter((t) => t.ev === KEY_STOP_EV).length;
  const slow = trials.filter((t) => t.http > 1000);
  note(`STOP arrived after ${got} of ${trials.length} navigations (events: ${trials.map((t) => t.ev).join(', ')})`);
  note(`first HTTP answer after the navigation took: ${trials.map((t) => t.http + ' ms').join(', ')}`);
  if (slow.length) note(`${slow.length} of ${trials.length}: the server was held about 2 s by a connection the browser opened but never sent a request on (the sim's 2 s head timeout; the ESP32 WebServer waits up to 5 s)`);
  assert(got === trials.length, 'the page\'s pagehide STOP did not reach the dart every time; only the firmware deadman stopped the motor');
}

async function check5(note) {
  await clickButton('AUTO anchor');
  const tA = await waitPage(async () => { const s = await statusLine(); return { ok: /^ANCHORING\b/.test(s), last: s }; }, 2000, 'status line ANCHORING');
  const d = await dirs();
  note(`status line shows ANCHORING after ${tA} ms; motors ${fmtDirs(d)}`);
  assert(d.kick === 1, 'AUTO did not start the kick swing');
  await clickButton('RELEASE');
  const tR = await waitPage(async () => { const s = await logText(); return { ok: s.includes('refused: busy'), last: s.slice(-200) }; }, 1500, 'refusal text in the log');
  note(`RELEASE while anchoring: "refused: busy" in the log after ${tR} ms`);
  assert(/^ANCHORING\b/.test(await statusLine()) || (await status()).phase === 'ANCHORING', 'RELEASE changed the phase');
  await clickButton('STOP');
  const tS = await waitDirs(allStopped, 500, 'STOP to stop all motors');
  const tI = await waitPage(async () => { const s = await statusLine(); return { ok: /^IDLE\b/.test(s), last: s }; }, 1500, 'status line IDLE');
  note(`STOP: all motors off after ${tS} ms, status line IDLE after ${tI} ms (${(await status()).seq})`);
  await holdsFor(allStopped, 600, 'all stopped after STOP');
}

async function check5b(note) {
  // STOP while a run key is held: keyboard key 4 down, STOP clicked with the mouse (or a second finger on a phone)
  await page.keyboard.down('Digit4');
  await waitDirs((d) => d.foot === 1, 500, 'foot to start');
  await clickButton('STOP');
  const tS = await waitDirs((d) => d.foot === 0, 500, 'STOP to stop the foot');
  note(`STOP stopped the foot after ${tS} ms`);
  await holdsFor(allStopped, 1000, 'motors stay stopped after STOP while 4 is still physically held');
}

async function check6(note) {
  // drive the stop counter far up on this page, prove the firmware now drops this page's stale low-numbered run keys,
  // then reload: the new page counts from 1 again and must still run at once
  const oldP = await page.evaluate('P');
  await page.evaluate('N += 500');
  await page.keyboard.press('Space');                // STOP with n about 501
  await sleep(200);
  const stale = await get(`/key?c=2&ms=350&n=10&p=${oldP}`);
  await sleep(50);
  const dStale = await dirs();
  note(`old page p=${oldP}: STOP sent with n=${await page.evaluate('N')}; a run key with n=10 from it -> HTTP ${stale.code} "${stale.body.trim()}", kick dir ${dStale.kick} (dropped)`);
  assert(stale.code === 409 && stale.body.includes('sent before a stop') && dStale.kick === 0, 'the stop counter did not drop a stale run key; the reload check would prove nothing');
  await sleep(600);
  await page.reload();
  await page.waitForSelector('#b button');
  const newP = await page.evaluate('P');
  const t0 = now();
  await page.keyboard.down('Digit2');
  const tOn = await waitDirs((d) => d.kick === 1, 300, 'kick to start right after reload');
  note(`reloaded (p=${newP}, N=${await page.evaluate('N')}); key 2 ran the kick ${tOn} ms after keydown (${now() - t0} ms incl. polling)`);
  assert(newP !== oldP, 'page id did not change on reload');
  await holdsFor((d) => d.kick === 1, 600, 'kick running while 2 held after reload');
  await page.keyboard.up('Digit2');
  await waitDirs((d) => d.kick === 0, 500, 'kick to stop');
}

async function check7(note) {
  await sleep(900);                                  // at least two status polls
  const s = await statusLine();
  const j = await status();
  note('status line: ' + JSON.stringify(s));
  const first = s.split('\n')[0];
  assert(new RegExp('^' + j.phase + '\\b').test(first), `status line does not start with the phase ${j.phase}`);
  assert(/^(IDLE|ARMED|FLIGHT|LANDED|ANCHORING|ANCHORED|RELEASING|FAULT)\b/.test(first), 'no phase on the status line');
  assert(s.includes('INA219 ok'), 'status line lacks "INA219 ok"');
  assert(s.includes('BNO085 ok'), 'status line lacks "BNO085 ok"');
}

// v4.1: video buttons switch D10, the status line shows battery, orientation and video
async function check8(note) {
  await clickButton('video off');
  await waitPage(async () => { const v = (await simState()).video_pin; return { ok: v === 0, last: v }; }, 2000, 'video pin low after "video off"');
  await sleep(900);
  let s = await statusLine();
  note('status line: ' + JSON.stringify(s));
  assert(/battery ok [0-9.]+ V/.test(s), 'status line lacks "battery ok <V>"');
  assert(s.includes('upright'), 'status line lacks the orientation');
  assert(s.includes('video off'), 'status line lacks "video off"');
  await clickButton('video on');
  await waitPage(async () => { const v = (await simState()).video_pin; return { ok: v === 1, last: v }; }, 2000, 'video pin high after "video on"');
}

// ---------------------------------------------------------------------------------------------------- main
async function main() {
  if (!fs.existsSync(SIM)) execFileSync('make', ['-C', HERE], { stdio: 'inherit' });
  fs.mkdirSync(path.dirname(LOG), { recursive: true });
  port = await freePort();
  base = `http://127.0.0.1:${port}`;
  const logFd = fs.openSync(LOG, 'w');
  sim = spawn(SIM, ['--port', String(port)], { stdio: ['ignore', logFd, logFd] });
  const t0 = now();
  for (;;) {
    try { await status(); break; } catch (e) {
      if (sim.exitCode !== null || now() - t0 > 10000) throw new Error('simulator did not start, see ' + LOG);
      await sleep(100);
    }
  }
  console.log(`simulator on port ${port} (log ${LOG})`);

  browser = await chromium.launch();
  page = await browser.newPage();
  page.on('console', (m) => { if (m.type() === 'error' || m.type() === 'warning') consoleMsgs.push(`${m.type()}: ${m.text()} ${m.location().url || ''}`.trim()); });
  page.on('pageerror', (e) => pageErrors.push(e.message));
  await openPage();
  await settle();

  await check('1 hold "kick out" button 2 s runs; release stops within 0.5 s', check1);
  await check('2 keys 2 + 4 held together; release 2 stops kick only; release 4 stops all', check2);
  await check('3 Shift+2 (@), Shift released first: kick stops on releasing 2', check3);
  await check('4a holding a button, window blur stops the motors', check4a);
  await check('4b holding a key, page hidden (visibilitychange) stops the motors', check4b);
  await check('4c holding a key, navigating away (pagehide): the motor stops within 0.6 s', check4c);
  await check('4d on pagehide the page\'s own STOP reaches the dart (6 tries)', check4d);  // regression: fixed 3 Nov (keepalive STOP on pagehide / STOP button clears held keys)
  await check('5 AUTO -> ANCHORING, RELEASE refused in log, STOP stops all', check5);
  await check('5b STOP button while a run key is held keeps the motors stopped', check5b);  // regression: fixed 3 Nov (keepalive STOP on pagehide / STOP button clears held keys)
  await check('6 reload, then a held key works at once (page id resets the stop counter)', check6);
  await check('7 status line shows phase, INA219 ok, BNO085 ok', check7);
  await check('8 video off / on buttons switch D10; status line shows battery, orientation, video', check8);

  const network = consoleMsgs.filter((m) => m.includes('Failed to load resource'));
  const other = consoleMsgs.filter((m) => !m.includes('Failed to load resource'));
  console.log('\nbrowser console:');
  console.log(`  uncaught JS errors: ${pageErrors.length ? '' : 'none'}`);
  for (const e of pageErrors) console.log('    ' + e);
  console.log(`  other console errors/warnings: ${other.length ? '' : 'none'}`);
  for (const e of other) console.log('    ' + e);
  console.log(`  failed resource loads (logged by Chromium for HTTP errors): ${network.length ? '' : 'none'}`);
  const uniq = {};
  for (const e of network) uniq[e] = (uniq[e] || 0) + 1;
  for (const [e, n] of Object.entries(uniq)) console.log(`    ${n}x ${e}`);

  const count = (r) => results.filter((x) => x.res === r).length;
  console.log(`\n${count('PASS')} passed, ${count('FAIL')} failed, ${count('XFAIL')} expected failures (known bugs), ${count('XPASS')} unexpected passes`);
  return count('FAIL') + count('XPASS') + pageErrors.length ? 1 : 0;
}

main().then(async (rc) => {
  if (browser) await browser.close();
  if (sim) sim.kill('SIGTERM');
  process.exit(rc);
}, async (e) => {
  console.error('ERROR: ' + (e && e.stack || e));
  if (browser) await browser.close().catch(() => {});
  if (sim) sim.kill('SIGTERM');
  process.exit(2);
});
