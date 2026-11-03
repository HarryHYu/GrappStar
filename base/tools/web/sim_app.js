// base/tools/web/sim_app.js: generic mission sim. Plays the per-part keyframe tracks in CAD_DATA (computed by
// tools/sim.py from the assembly kinematics over the mission timeline). No mechanism specific code.
CADV.boot(function () {
  'use strict';
  const D = window.CAD_DATA, $ = s => document.querySelector(s);
  const T = D.tracks.t, N = T.length, T0 = T[0], T_END = T[N - 1];
  const PH = (D.phases && D.phases.length) ? D.phases : [{ name: 'run', t0: T0, t1: T_END, text: '' }];
  const css = () => getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#eef1f4';
  const V = CADV.build($('#view'), D, { hd: true, bg: css() });
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
    tip.innerHTML = partText(r.p); tip.style.display = 'block';
    tip.style.left = Math.min(innerWidth - 290, e.clientX + 14) + 'px'; tip.style.top = (e.clientY + 14) + 'px';
  });
  cv.addEventListener('pointerleave', () => { tip.style.display = 'none'; });

  // ------------------------------------------------------------------ view tools
  const VIEWS = { iso: [-55, 28], front: [0, 8], side: [-90, 8], top: [-90, 89] };
  document.querySelectorAll('[data-v]').forEach(b => b.onclick = () => { const v = VIEWS[b.dataset.v]; V.view(v[0], v[1], 0.9); });
  $('#explode').oninput = e => V.explode(+e.target.value / 100);
  const B = V.box0;
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
    $('#theme').textContent = 'theme: ' + themes[th]; V.scene.background = new THREE.Color(css());
  };
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { V.scene.background = new THREE.Color(css()); });
  $('#tInfo').onclick = () => $('#info').classList.toggle('more');
  $('#tTools').onclick = () => { $('#tools').classList.toggle('open'); $('#toggles').classList.toggle('up'); };

  // ------------------------------------------------------------------ frame loop
  let lastPhase = -1, lastS = -1;
  function update() {
    const s = sampleOf(t); if (s === lastS) return; V.sample(s); lastS = s;
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
    update(); V.renderIfNeeded(); requestAnimationFrame(frame);
  }
  V.view(-55, 28, 0.9);
  update(); V.render();
  window.SIM = {
    seek: x => { t = Math.max(T0, Math.min(T_END, +x)); update(); V.render(); },
    pause: () => { playing = false; $('#play').textContent = 'play'; },
    play: () => { playing = true; $('#play').textContent = 'pause'; },
    time: () => t, phases: PH,
    cut: (axis, frac, flip) => { $('#cax').value = axis || 'none'; $('#cut').value = Math.round(1000 * (frac == null ? 0.5 : frac)); $('#cflip').checked = !!flip; applyCut(); V.render(); },
  };
  window.SIM_SEEK = window.SIM.seek; window.SIM_PAUSE = window.SIM.pause; window.SIM_CUT = window.SIM.cut;
  requestAnimationFrame(frame);
  window.READY = true;
});
