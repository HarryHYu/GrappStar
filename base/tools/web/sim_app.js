// base/tools/web/sim_app.js: generic mission sim. Plays the per-part keyframe tracks in CAD_DATA (computed by
// tools/sim.py from the assembly kinematics over the mission timeline). No mechanism specific code.
CADV.boot(function () {
  'use strict';
  const D = window.CAD_DATA, $ = s => document.querySelector(s);
  const T = D.tracks.t, N = T.length, T0 = T[0], T_END = T[N - 1];
  const PH = (D.phases && D.phases.length) ? D.phases : [{ name: 'run', t0: T0, t1: T_END, text: '' }];
  const css = () => getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#eef1f4';
  const isDark = () => { const a = document.documentElement.dataset.theme; return a ? a === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches; };
  const V = CADV.build($('#view'), D, { hd: true, bg: css(), dark: isDark() });
  CADV.installAUD(V);
  $('#ttl').textContent = D.title + ' mission sim';
  $('#sub').textContent = `${V.list.length} parts, ${N} samples over ${(T_END - T0).toFixed(1)} s. Every pose comes from the assembly kinematics.`;
  document.title = D.title + ' mission sim';

  // ------------------------------------------------------------------ time
  function sampleOf(t) {
    if (t <= T[0]) return 0; if (t >= T[N - 1]) return N - 1;
    let lo = 0, hi = N - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= t) lo = m; else hi = m; }
    return lo + (t - T[lo]) / Math.max(1e-9, T[hi] - T[lo]);
  }
  let t = T0, playing = true, speed = 1, last = performance.now();

  // ------------------------------------------------------------------ timeline bar
  const bar = $('#bar'), segs = [];
  PH.forEach((p, i) => {
    const s = document.createElement('div'); s.className = 'seg';
    s.style.left = (100 * (p.t0 - T0) / (T_END - T0)) + '%'; s.style.width = (100 * (p.t1 - p.t0) / (T_END - T0)) + '%';
    s.title = p.name + ': ' + (p.text || ''); bar.appendChild(s); segs.push(s);
    const b = document.createElement('button'); b.textContent = p.name.replace(/_/g, ' '); b.title = p.text || '';
    b.onclick = () => { t = p.t0 + 1e-4; }; $('#phases').appendChild(b); p.btn = b;
  });
  const cur = document.createElement('div'); cur.className = 'cur'; bar.appendChild(cur);
  function scrub(e) { const r = bar.getBoundingClientRect(); t = T0 + (T_END - T0) * Math.max(0, Math.min(1, (e.clientX - r.left) / r.width)); }
  let dragging = false;
  bar.addEventListener('pointerdown', e => { dragging = true; bar.setPointerCapture(e.pointerId); scrub(e); });
  bar.addEventListener('pointermove', e => { if (dragging) scrub(e); });
  bar.addEventListener('pointerup', () => { dragging = false; });
  $('#play').onclick = () => { playing = !playing; if (playing && t >= T_END) t = T0; $('#play').textContent = playing ? 'pause' : 'play'; };
  $('#speed').onchange = e => { speed = +e.target.value; };
  document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.key === ' ') { e.preventDefault(); $('#play').click(); }
    if (e.key === 'ArrowRight') t = Math.min(T_END, t + 0.5); if (e.key === 'ArrowLeft') t = Math.max(T0, t - 0.5);
  });

  // ------------------------------------------------------------------ readout
  const cols = D.states || {}, keys = Object.keys(cols);
  function fmt(v) { return typeof v === 'number' ? (Number.isInteger(v) ? String(v) : v.toFixed(2)) : String(v); }
  function readout(s) {
    const i = Math.round(s);
    let h = `<span>t</span><span>${t.toFixed(2)} s</span>`;
    for (const k of keys) if (k !== 't') h += `<span>${k}</span><span>${fmt(cols[k][i])}</span>`;
    $('#ro').innerHTML = h;
  }

  // ------------------------------------------------------------------ parts list, tooltips
  const tip = $('#tip');
  function partText(p) {
    return `<b>${p.name}</b><br>${p.subsystem ? p.subsystem + ', ' : ''}${p.material}${p.qty > 1 ? ', qty ' + p.qty : ''}` +
      `${p.mass_g != null ? ', ' + p.mass_g + ' g' : ''}${p.bought_ref ? '<br>' + p.bought_ref : ''}${p.notes ? '<br>' + p.notes : ''}`;
  }
  for (const r of V.list) {
    const p = r.p, d = document.createElement('div'); d.className = 'pi';
    d.innerHTML = `<span class="sw" style="background:${p.color}"></span><div><div class="n">${p.name}</div>` +
      `<div class="d">${p.material}${p.bought_ref ? ', ' + p.bought_ref : ''}${p.notes ? '<br>' + p.notes : ''}</div></div>`;
    d.title = `${p.name}\n${p.material}${p.bought_ref ? ', ' + p.bought_ref : ''}${p.mass_g != null ? ', ' + p.mass_g + ' g' : ''}\n${p.notes || ''}`;
    d.onmouseenter = () => V.highlight([p.name]); d.onmouseleave = () => V.highlight([]);
    d.onclick = () => { r.hidden = !r.hidden; d.classList.toggle('off', r.hidden); V.show(null); };
    $('#plist').appendChild(d);
  }
  const cv = V.renderer.domElement;
  cv.addEventListener('pointermove', e => {
    if (e.buttons) { tip.style.display = 'none'; return; }
    const r = V.pick(e.clientX, e.clientY);
    if (!r) { tip.style.display = 'none'; return; }
    tip.innerHTML = partText(r.p); tip.style.display = 'block'; placeTip(e);
  });
  function placeTip(e) {          // keep the tooltip inside the window
    const w = tip.offsetWidth, h = tip.offsetHeight;
    let x = e.clientX + 14, y = e.clientY + 14;
    if (x + w > innerWidth - 8) x = e.clientX - 14 - w;
    if (y + h > innerHeight - 8) y = e.clientY - 14 - h;
    tip.style.left = Math.max(8, Math.min(innerWidth - w - 8, x)) + 'px'; tip.style.top = Math.max(8, Math.min(innerHeight - h - 8, y)) + 'px';
  }
  cv.addEventListener('pointerleave', () => { tip.style.display = 'none'; });

  // ------------------------------------------------------------------ view tools
  const VIEWS = { iso: [-55, 28], front: [0, 8], side: [-90, 8], top: [-90, 89], mission: [-62, 22] };
  function setView(name) {
    const v = VIEWS[name] || VIEWS.iso;
    if (name === 'mission') V.view(v[0], v[1], 0.95, V.boxTrack);      // the whole track: every pose over the mission
    else V.view(v[0], v[1], 0.9);
    followC = null; V.render();
  }
  document.querySelectorAll('[data-v]').forEach(b => b.onclick = () => setView(b.dataset.v));

  // ------------------------------------------------------------------ mechanisms: one mechanism on its own, still moving
  const MECHS = D.mechs || [], msel = $('#mech');
  if (msel) {
    for (const m of MECHS) { const o = document.createElement('option'); o.value = m.key; o.textContent = m.title; msel.appendChild(o); }
    msel.onchange = () => setMech(msel.value);
  }
  function setMech(key) {
    const m = MECHS.find(x => x.key === key) || null;
    if (msel) msel.value = m ? m.key : '';
    const names = m ? m.parts : null, keep = names ? new Set(names) : null;
    V.show(names);
    if (V.showLine) V.showLine(!m || m.key === 'tether');
    document.querySelectorAll('#plist .pi').forEach(d => { const n = d.querySelector('.n'); d.style.display = !keep || (n && keep.has(n.textContent)) ? '' : 'none'; });
    const mt = $('#mtxt'); if (mt) mt.textContent = m ? m.job : '';
    if (m) { const fr = names.filter(n => !/_dart$/.test(n)); V.view(-55, 28, 0.85, V.bbox(fr.length ? fr : names)); followC = null; }
    else setView('iso');
    V.render();
  }
  // follow a moving joint group (for example the dart in flight): the camera target moves with its parts
  const fsel = $('#follow'); let follow = null, followC = null;
  for (const m of V.movers) { const o = document.createElement('option'); o.value = m.name; o.textContent = m.name.replace(/_/g, ' ') + ' (' + m.parts.length + ')'; fsel.appendChild(o); }
  if (!V.movers.length) fsel.disabled = true;
  function setFollow(name) {
    // exact joint name, else the first moving group whose joint or part names contain the text (e.g. 'dart')
    follow = !name ? null : V.movers.find(m => m.name === name) || V.movers.find(m => m.name.includes(name) || m.parts.some(n => n.includes(name))) || null;
    fsel.value = follow ? follow.name : '';
    followC = null; followCam(); V.render();
  }
  fsel.onchange = () => setFollow(fsel.value);
  function followCam() {
    if (!follow) return;
    const bx = V.bbox(follow.parts), c = bx.getCenter(new THREE.Vector3());
    const g = V.camGet();
    if (!followC) V.cam(c.toArray(), Math.max(V.fitDist(bx, 1) * 3, V.radius * 1.5), g.az, g.el);   // close enough to see it
    else V.cam([g.t[0] + c.x - followC.x, g.t[1] + c.y - followC.y, g.t[2] + c.z - followC.z], g.r, g.az, g.el);
    followC = c;
  }
  $('#explode').oninput = e => V.explode(+e.target.value / 100);
  const B = V.boxTrack;           // cut range over the whole track, not just sample 0
  function applyCut() {
    const ax = $('#cax').value;
    if (ax === 'none') { V.cut(null); $('#cval').textContent = ''; return; }
    const lo = B.min[ax], hi = B.max[ax], c = lo + (hi - lo) * (+$('#cut').value / 1000);
    V.cut(ax, c, $('#cflip').checked); $('#cval').textContent = `${ax} = ${c.toFixed(1)} mm`;
  }
  $('#cax').onchange = applyCut; $('#cut').oninput = applyCut; $('#cflip').onchange = applyCut;
  const themes = ['auto', 'light', 'dark']; let th = 0;
  $('#theme').onclick = () => {
    th = (th + 1) % 3; if (th === 0) document.documentElement.removeAttribute('data-theme'); else document.documentElement.dataset.theme = themes[th];
    $('#theme').textContent = 'theme: ' + themes[th]; applyTheme();
  };
  function applyTheme() { V.setTheme(isDark(), css()); V.render(); }
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
  $('#tInfo').onclick = () => $('#info').classList.toggle('more');
  $('#tTools').onclick = () => { $('#tools').classList.toggle('open'); $('#toggles').classList.toggle('up'); };

  // ------------------------------------------------------------------ frame loop
  let lastPhase = -1, lastS = -1;
  function update() {
    const s = sampleOf(t); if (s === lastS) return; V.sample(s); lastS = s; followCam();
    let pi = PH.findIndex(p => t >= p.t0 && t < p.t1); if (pi < 0) pi = t >= T_END ? PH.length - 1 : 0;
    if (pi !== lastPhase) {
      lastPhase = pi; const p = PH[pi];
      $('#ptitle').textContent = `${pi + 1}. ${p.name.replace(/_/g, ' ')}`; $('#ptext').textContent = p.text || '';
      segs.forEach((g, j) => g.classList.toggle('on', j === pi)); PH.forEach((q, j) => q.btn.classList.toggle('on', j === pi));
    }
    readout(s);
    cur.style.left = (100 * (t - T0) / (T_END - T0)) + '%';
    $('#time').textContent = `${t.toFixed(2)} / ${T_END.toFixed(1)} s`;
  }
  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    if (playing && !dragging) {
      t += dt * speed;
      if (t > T_END) { if ($('#loop').checked) t = T0; else { t = T_END; playing = false; $('#play').textContent = 'play'; } }
    }
    if (!V.frozen()) update();
    V.renderIfNeeded(); requestAnimationFrame(frame);
  }
  V.view(-55, 28, 0.9);
  update(); V.render();
  { const hm = /mech=([\w-]+)/.exec(location.hash); if (hm) setMech(hm[1]); }
  window.SIM = {
    seek: x => { t = Math.max(T0, Math.min(T_END, +x)); update(); V.render(); },
    pause: () => { playing = false; $('#play').textContent = 'play'; },
    play: () => { playing = true; $('#play').textContent = 'pause'; },
    time: () => t, phases: PH,
    viewer: V, view: setView, follow: setFollow, movers: V.movers.map(m => m.name),
    freeze: on => { V.freeze(on !== false); if (on !== false) V.render(); },
    cut: (axis, frac, flip) => { $('#cax').value = axis || 'none'; $('#cut').value = Math.round(1000 * (frac == null ? 0.5 : frac)); $('#cflip').checked = !!flip; applyCut(); V.render(); },
  };
  window.SIM_SEEK = window.SIM.seek; window.SIM_PAUSE = window.SIM.pause; window.SIM_CUT = window.SIM.cut;
  window.SIM.mech = setMech; window.SIM_MECH = setMech;
  window.SIM_CLEAN = () => { document.querySelectorAll('.panel').forEach(p => { p.style.display = 'none'; }); V.render(); };   // picture only (shots)
  window.SIM_VIEW = window.SIM.view; window.SIM_FOLLOW = window.SIM.follow; window.SIM_FREEZE = window.SIM.freeze;
  requestAnimationFrame(frame);
  window.READY = true;
});
