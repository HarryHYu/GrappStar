// base/tools/web/assembly_app.js: generic assembly steps page. CAD_DATA.steps = [{title, text, parts, subsystem, auto}]
// built by tools/steps.py. Each step shows the parts added so far; the new parts fly in from an exploded offset.
CADV.boot(function () {
  'use strict';
  const D = window.CAD_DATA, $ = s => document.querySelector(s);
  const css = () => getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#eef1f4';
  const isDark = () => { const a = document.documentElement.dataset.theme; return a ? a === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches; };
  const V = CADV.build($('#view'), D, { hd: true, bg: css(), dark: isDark() });
  CADV.installAUD(V);
  // text steps (no new parts) stay: they show the current state
  const STEPS = D.steps.map(s => Object.assign({}, s, { parts: s.parts.filter(n => V.parts[n]) }));
  const NP = V.list.length;
  $('#ttl').textContent = D.title + ' assembly';
  $('#card').onclick = () => $('#card').classList.toggle('more');
  V.sample(0);
  V.setGround(true);

  // fly-in offsets: up and away from what is already built
  const centre = n => V.parts[n].mesh.geometry.boundingBox.getCenter(new THREE.Vector3()).applyMatrix4(V.parts[n].world);
  const R = V.radius;
  STEPS.forEach((st, i) => {
    const before = STEPS.slice(0, i).flatMap(s => s.parts);
    const c0 = new THREE.Vector3();
    if (before.length) { before.forEach(n => c0.add(centre(n))); c0.multiplyScalar(1 / before.length); } else c0.copy(V.center);
    st.off = {};
    for (const n of st.parts) {
      const d = centre(n).sub(c0); d.z = 0;
      const h = d.length() > 1e-6 ? d.normalize().multiplyScalar(0.35 * R) : new THREE.Vector3();
      st.off[n] = [h.x, h.y, (before.length ? 0.9 : 0.6) * R];
    }
  });

  const list = $('#steps');
  STEPS.forEach((st, i) => { const b = document.createElement('button'); b.textContent = (i + 1) + (st.label ? ' (' + st.label + ')' : '') + '. ' + st.title; b.title = st.text || ''; b.onclick = () => go(i); list.appendChild(b); });
  let cur = 0, t0 = performance.now(), userCam = false, anim = true, fixedF = null;
  const info = n => V.parts[n].p;
  function bill(st) {
    return st.parts.map(n => {
      const p = info(n);
      const ref = [p.bought_ref, p.notes].filter(Boolean).join('. ');
      return `<tr class="new"><td><span class="sw" style="background:${p.color}"></span>${n}${ref ? '<div class="d">' + ref + '</div>' : ''}</td>` +
        `<td>${p.qty || 1}</td><td>${p.material}${p.mass_g != null ? '<div class="d">' + p.mass_g + ' g</div>' : ''}</td></tr>`;
    }).join('');
  }
  function go(i, instant) {
    fixedF = null; V.freeze(false);
    cur = Math.max(0, Math.min(STEPS.length - 1, i)); t0 = performance.now() - (instant ? 1e5 : 0); anim = true;
    const st = STEPS[cur], sofar = STEPS.slice(0, cur + 1).reduce((a, s) => a + s.parts.length, 0);
    $('#st').textContent = `Step ${cur + 1}${st.label ? ' (' + st.label + ')' : ''}: ${st.title}`;
    $('#sd').textContent = st.text || '';
    $('#count').textContent = `Step ${cur + 1} of ${STEPS.length}. ${st.parts.length} new part${st.parts.length === 1 ? '' : 's'} (highlighted), ${sofar} of ${NP} fitted` + (st.subsystem ? `, ${st.subsystem}` : '');
    $('#bill').innerHTML = bill(st);
    [...list.children].forEach((b, j) => { b.classList.toggle('on', j === cur); b.classList.toggle('done', j < cur); });
    $('#prev').disabled = cur === 0; $('#next').disabled = cur === STEPS.length - 1;
    const shown = STEPS.slice(0, cur + 1).flatMap(s => s.parts);
    V.show(shown); userCam = false;
    const ghost = $('#ghost').checked;
    V.highlight(st.parts, ghost);
  }
  $('#prev').onclick = () => go(cur - 1); $('#next').onclick = () => go(cur + 1);
  $('#replay').onclick = () => go(cur); $('#recam').onclick = () => { userCam = false; anim = true; };
  $('#ghost').onchange = () => go(cur, true);
  document.addEventListener('keydown', e => { if (e.key === 'ArrowRight') go(cur + 1); if (e.key === 'ArrowLeft') go(cur - 1); });
  if (V.ctl) V.ctl.addEventListener('start', () => { userCam = true; });

  // tooltip
  const tip = $('#tip'), cv = V.renderer.domElement;
  cv.addEventListener('pointermove', e => {
    if (e.buttons) { tip.style.display = 'none'; return; }
    const r = V.pick(e.clientX, e.clientY); if (!r) { tip.style.display = 'none'; return; }
    const p = r.p;
    tip.innerHTML = `<b>${p.name}</b><br>${p.material}${p.bought_ref ? ', ' + p.bought_ref : ''}${p.notes ? '<br>' + p.notes : ''}`;
    tip.style.display = 'block';
    const w = tip.offsetWidth, h = tip.offsetHeight;          // keep the tooltip inside the window
    let x = e.clientX + 14, y = e.clientY + 14;
    if (x + w > innerWidth - 8) x = e.clientX - 14 - w;
    if (y + h > innerHeight - 8) y = e.clientY - 14 - h;
    tip.style.left = Math.max(8, Math.min(innerWidth - w - 8, x)) + 'px'; tip.style.top = Math.max(8, Math.min(innerHeight - h - 8, y)) + 'px';
  });

  // theme: auto (follows the system), light, dark
  const themes = ['auto', 'light', 'dark']; let th = 0;
  function applyTheme() { V.setTheme(isDark(), css()); V.render(); }
  $('#theme').onclick = () => {
    th = (th + 1) % 3; if (th === 0) document.documentElement.removeAttribute('data-theme'); else document.documentElement.dataset.theme = themes[th];
    $('#theme').textContent = 'theme: ' + themes[th]; applyTheme();
  };
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
  cv.addEventListener('pointerleave', () => { tip.style.display = 'none'; });

  // camera follows the parts fitted so far
  const camT = V.center.clone(); let camR = V.fitDist(V.box0, 0.9);
  function step(now) {
    const f = fixedF != null ? fixedF : Math.min(1, (now - t0) / 1300), e = 1 - Math.pow(1 - f, 3);
    if (anim) {
      const st = STEPS[cur];
      V.highlightLevel(0.12 + 0.4 * (1 - e));
      for (const r of V.list) r.off.set(0, 0, 0);
      for (const n of st.parts) { const o = st.off[n]; V.setOffset(n, [o[0] * (1 - e), o[1] * (1 - e), o[2] * (1 - e)]); }
      V.sample(0);
      if (!userCam) {
        const shown = STEPS.slice(0, cur + 1).flatMap(s => s.parts);
        for (const r of V.list) r.off.set(0, 0, 0);
        V.sample(0);
        const box = V.bbox(shown);
        for (const n of st.parts) { const o = st.off[n]; V.setOffset(n, [o[0] * (1 - e), o[1] * (1 - e), o[2] * (1 - e)]); }
        V.sample(0);
        const gt = box.getCenter(new THREE.Vector3()), gr = Math.max(V.radius * 0.8, V.fitDist(box, 1.0));
        const k = f >= 1 || fixedF != null ? 1 : 0.12;
        camT.lerp(gt, k); camR += (gr - camR) * k;
        const c = V.camGet(); V.cam(camT.toArray(), camR, c.az, c.el);
        if (f >= 1 && camT.distanceTo(gt) < 1e-3) anim = false;
      } else if (f >= 1) anim = false;
    }
  }
  function frame(now) {
    if (!V.frozen()) step(now);
    V.renderIfNeeded(); requestAnimationFrame(frame);
  }
  // shots: pose the current step at a fixed animation fraction f (0 = start of the fly-in, 1 = settled, camera
  // snapped), render once and stop redrawing until the next go()
  function freezeAt(f) {
    fixedF = Math.max(0, Math.min(1, +f)); anim = true; userCam = false;
    step(performance.now()); V.render(); V.freeze(true);
  }
  V.view(-50, 30, 0.9); camR = V.camGet().r;
  go(0);
  window.ASM_GO = (i, instant) => { go(+i, !!instant); };
  window.ASM_FREEZE = freezeAt;
  window.ASM = { go: window.ASM_GO, freeze: freezeAt, steps: STEPS, viewer: V };
  requestAnimationFrame(frame);
  window.READY = true;
});
