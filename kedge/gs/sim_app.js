// GrappStar mission sim: plays back timeline.json (computed from the mechanism and physics) on the real meshes.
(function () {
  const GS = window.GS, P = GS.P, TL = window.TL, S = TL.samples, PHS = TL.phases;
  const $ = s => document.querySelector(s);
  const V = GSV.build($('#view'), { hd: true, bg: 0xeef1f4, groundColor: 0xd9d4c7 });
  const { scene, cam, ctl, meshes, renderer } = V;
  renderer.shadowMap.autoUpdate = true;
  // field: ground at the rover wheels, a darker soil strip where the dart lands
  scene.children.filter(o => o.type === 'Mesh' && o.geometry.type === 'PlaneGeometry').forEach(o => o.position.z = TL.ground_z - 0.5);
  scene.children.filter(o => o.type === 'GridHelper').forEach(o => { o.position.z = TL.ground_z; o.scale.set(6, 6, 6); });
  const soil = new THREE.Mesh(new THREE.PlaneGeometry(4000, 1600), new THREE.MeshStandardMaterial({ color: 0xb59f7b, roughness: 1 }));
  soil.position.set(10800, 0, TL.ground_z - 0.2); soil.receiveShadow = true; scene.add(soil);
  // range marks every metre
  for (let m = 1; m <= 14; m++) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(6, 900, 1), new THREE.MeshBasicMaterial({ color: m % 5 ? 0xc9c3b4 : 0x9c937f }));
    b.position.set(m * 1000 + 300, 0, TL.ground_z + 0.3); scene.add(b);
  }
  // rig: everything on the rover moves with it during the tow
  const rig = new THREE.Group(); scene.add(rig);
  const dartNames = [];
  for (const k in meshes) {
    const m = meshes[k];
    if (m.userData.group === 'dart') { dartNames.push(k); continue; }
    scene.remove(m); rig.add(m);
  }
  // rover placeholder (chassis + wheels) under the base
  const rover = new THREE.Group(); rig.add(rover);
  const dk = TL.deck, bz = -P.BASE_T;
  const chMat = new THREE.MeshStandardMaterial({ color: 0x3b4450, roughness: 0.7, metalness: 0.2 });
  const ch = new THREE.Mesh(new THREE.BoxGeometry(760, 420, 70), chMat); ch.position.set(70, 0, bz - dk + 95); ch.castShadow = ch.receiveShadow = true; rover.add(ch);
  const whMat = new THREE.MeshStandardMaterial({ color: 0x1d1f22, roughness: 0.9 });
  const wheels = [];
  for (const [x, y] of [[-200, 250], [340, 250], [-200, -250], [340, -250]]) {
    const w = new THREE.Mesh(new THREE.CylinderGeometry(70, 70, 60, 40), whMat); w.position.set(x, y, bz - dk + 70); w.castShadow = true; rover.add(w); wheels.push(w);
  }
  const lbl = document.createElement('div'); lbl.className = 'tag'; lbl.textContent = 'rover (placeholder)'; $('#view').appendChild(lbl);
  // line: a thin tube rebuilt each frame
  const lineMat = new THREE.MeshStandardMaterial({ color: 0xffb020, roughness: 0.5, emissive: 0x442200, emissiveIntensity: 0.4 });
  let lineMesh = null;
  function setLine(pts) {
    if (lineMesh) { scene.remove(lineMesh); lineMesh.geometry.dispose(); lineMesh = null; }
    if (!pts || pts.length < 2 || !$('#tLine').checked) return;
    const curve = new THREE.CatmullRomCurve3(pts.map(p => new THREE.Vector3(p[0], p[1], p[2])), false, 'centripetal');
    lineMesh = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.min(400, pts.length * 6), 0.9, 6, false), lineMat);
    scene.add(lineMesh);
  }
  // line pack on the spool (reel frame): a translucent cylinder whose radius follows the line on the spool
  const packMat = new THREE.MeshStandardMaterial({ color: 0xffb020, roughness: 0.6, transparent: true, opacity: 0.85 });
  const pack = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, P.SP_W - 1, 48), packMat);
  pack.matrixAutoUpdate = false; rig.add(pack);
  // springs (stretching coils) and cords, from the rail frame
  for (const k of ['spring_env_R', 'spring_env_L', 'cord_env_R', 'cord_env_L']) if (meshes[k]) meshes[k].visible = false;
  const coilPts = [];
  const NC = 60, RC = (P.SPRING_OD - 1.2) / 2;
  for (let i = 0; i <= NC * 18; i++) { const t = i / (NC * 18), a = t * NC * 2 * Math.PI; coilPts.push(new THREE.Vector3(t * 300, RC * Math.cos(a), RC * Math.sin(a))); }
  const coilGeo = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(coilPts), NC * 18, 0.6, 5, false);
  const springMat = new THREE.MeshStandardMaterial({ color: 0xb8bec4, metalness: 0.7, roughness: 0.3 });
  const springs = [1, -1].map(sg => { const m = new THREE.Mesh(coilGeo, springMat); m.matrixAutoUpdate = false; m.castShadow = true; rig.add(m); return { m, sg }; });
  const cordMat = new THREE.LineBasicMaterial({ color: 0xd9b36b });
  const cords = [1, -1].map(sg => { const g = new THREE.BufferGeometry(); const l = new THREE.Line(g, cordMat); l.frustumCulled = false; rig.add(l); return { l, g, sg }; });
  const PUL_R = (85 - P.SPRING_W) / 2, PUL_W = (85 + P.SPRING_W) / 2;
  const trip = (() => { const g = new THREE.BufferGeometry(); const l = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x2060c0 })); l.frustumCulled = false; rig.add(l); return { l, g }; })();
  function tripCordAt(F) {
    const A = new THREE.Vector3(P.CORD_A[0], -44, P.CORD_A[1]).applyMatrix4(F.sear), B = new THREE.Vector3(P.CORD_F[0], P.CORD_F[1], P.CORD_F[2]);
    const slack = Math.max(0, P.CORD_L - A.distanceTo(B)), pts = [];
    for (let k = 0; k <= 12; k++) { const f = k / 12, p = A.clone().lerp(B, f); p.z -= Math.sin(Math.PI * f) * Math.sqrt(slack * A.distanceTo(B)) * 0.6; pts.push(p); }
    trip.g.setFromPoints(pts);
  }
  function springsAt(F, s) {
    const L = P.SPRING_L0 + P.SPRING_PRE + (P.P_R - s), uj = P.ANCHOR_U + L;
    for (const sp of springs) sp.m.matrix.copy(F.rail).multiply(new THREE.Matrix4().makeTranslation(P.ANCHOR_U, sp.sg * P.SPRING_Y, P.SPRING_W)).multiply(new THREE.Matrix4().makeScale(L / 300, 1, 1));
    for (const c of cords) {
      const pts = [new THREE.Vector3(uj, c.sg * P.SPRING_Y, P.SPRING_W)];
      for (let k = 0; k <= 16; k++) { const a = -Math.PI / 2 + Math.PI * k / 16; pts.push(new THREE.Vector3(P.PULLEY_U + PUL_R * Math.cos(a), c.sg * P.SPRING_Y, PUL_W + PUL_R * Math.sin(a))); }
      pts.push(new THREE.Vector3(s + P.SHOE_L, c.sg * P.SPRING_Y, 85));
      pts.forEach(p => p.applyMatrix4(F.rail)); c.g.setFromPoints(pts);
    }
  }
  // x-ray of the rail
  let xrayState = null;
  function xray(on) {
    if (xrayState === on) return; xrayState = on;
    for (const k of ['rail_beam', 'cheek_L', 'cheek_R', 'cheek_tie', 'frame_plate_L', 'frame_plate_R', 'reel_cover', 'base_1', 'base_2', 'base_3', 'base_4', 'base_5']) {
      const m = meshes[k]; if (!m) continue; m.material.transparent = on; m.material.opacity = on ? 0.28 : 1; m.material.depthWrite = !on;
    }
  }
  // ------------------------------------------------------------- sample interpolation
  const T_END = S[S.length - 1].ts;
  function idx(ts) { let lo = 0, hi = S.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m].ts <= ts) lo = m; else hi = m; } return lo; }
  const NUM = ['phi', 'th', 'a', 's', 'pin', 'sear', 'k', 'd', 'lo', 'T', 'rv', 'rpm', 'I', 'Tm', 'sp', 't'];
  function sampleAt(ts) {
    const i = idx(ts), A = S[i], B = S[Math.min(S.length - 1, i + 1)];
    const f = B.ts > A.ts ? Math.min(1, Math.max(0, (ts - A.ts) / (B.ts - A.ts))) : 0;
    const o = { ph: A.ph, md: A.md, lp: A.lp };
    for (const k of NUM) o[k] = A[k] + (B[k] - A[k]) * f;
    let dr = B.rot - A.rot; if (dr > 180) dr -= 360; if (dr < -180) dr += 360; o.rot = A.rot + dr * f;
    if (A.dart && B.dart) { o.dart = A.dart.map((x, j) => x + (B.dart[j] - x) * f); } else o.dart = A.dart || null;
    return o;
  }
  // ------------------------------------------------------------- camera
  const CAMS = {
    mech: [[620, -760, 470], [150, 0, 120]],
    reel: [[-160, -330, 330], [-20, -40, 180]],
    sear: [[150, -330, 170], [90, -40, 140]],
    over: [[1100, -1500, 900], [150, 0, 100]],
    field: [[4200, -9000, 3800], [5400, 0, 0]],
  };
  let camMode = 'auto', camGoal = null;
  const camPos = new THREE.Vector3(), camTgt = new THREE.Vector3();
  function autoCam(o, d) {
    const rv = o.rv;
    switch (o.ph) {
      case 'ready': return [[rv + 700, -900, 520], [rv + 150, 0, 140]];
      case 'lift': case 'launch': return [[rv + 520, -950, 420], [rv + 170, 0, 200]];
      case 'flight': return [[d.x - 1300, d.y - 2300, Math.max(500, d.z + 500)], [d.x + 400, 0, Math.max(0, d.z - 150)]];
      case 'land': return [[d.x - 700, d.y - 1200, d.z + 450], [d.x + 50, d.y, d.z]];
      case 'anchor': case 'release': return [[d.x + 260, d.y - 520, d.z + 300], [d.x + 30, d.y, d.z + 10]];
      case 'tow': return [[rv - 900, -1300, 800], [rv + 1800, 0, 50]];
      case 'cock': return [[rv + 260, -470, 230], [rv + 90, -30, 140]];
      case 'retrieve': {
        const near = Math.max(0, Math.min(1, (d.x - rv - 700) / 1200));    // close to the muzzle: frame the horn and the cradle
        const far = [[d.x - 500, d.y - 1100, d.z + 600], [d.x + 100, 0, d.z]], close = [[rv + 650, -520, 320], [rv + 330, 0, 150]];
        const L = (a, b) => a.map((v, i) => v + (b[i] - v) * (1 - near));
        return [L(far[0], close[0]), L(far[1], close[1])];
      }
      default: return [[rv + 800, -1100, 650], [rv + 150, 0, 120]];
    }
  }
  // ------------------------------------------------------------- UI
  let ts = 0, playing = true, speed = 1, last = performance.now(), first = true;
  const bar = $('#bar');
  for (const p of PHS) {
    const m = document.createElement('div'); m.className = 'mk'; m.style.left = (100 * p.ts / T_END) + '%'; m.title = p.title; bar.appendChild(m);
    const b = document.createElement('button'); b.textContent = p.title.replace(/^\d+\.\s*/, ''); b.onclick = () => { ts = p.ts + 0.001; first = true; }; $('#phases').appendChild(b);
  }
  bar.onclick = e => { const r = bar.getBoundingClientRect(); ts = T_END * (e.clientX - r.left) / r.width; first = true; };
  $('#play').onclick = () => { playing = !playing; $('#play').textContent = playing ? 'pause' : 'play'; };
  $('#speed').oninput = e => { speed = Math.pow(2, +e.target.value); $('#spv').textContent = speed.toFixed(2) + 'x'; };
  $('#cam').onchange = e => { camMode = e.target.value; first = true; };
  $('#tX').onchange = () => {};
  ctl.addEventListener('start', () => { if (camMode === 'auto') { camMode = 'free'; $('#cam').value = 'free'; } });
  // parts list
  const pl = $('#plist');
  const seen = {};
  for (const p of GS.parts) {
    if (p.group === 'dart' || seen[p.name]) continue; seen[p.name] = 1;
    const d = document.createElement('div'); d.className = 'pi';
    d.innerHTML = `<span class="sw" style="background:${p.color}"></span><b>${p.name}</b>${p.printed ? '' : ' <i>(bought)</i>'}<br><span>${p.note}</span>`;
    d.onmouseenter = () => { const m = meshes[p.name]; if (m) { m.material.emissive = new THREE.Color(0xffffff); m.material.emissiveIntensity = 0.5; } };
    d.onmouseleave = () => { const m = meshes[p.name]; if (m) m.material.emissiveIntensity = 0; };
    pl.appendChild(d);
  }
  // ------------------------------------------------------------- frame
  const tmp = new THREE.Matrix4(), v3 = new THREE.Vector3();
  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    if (playing) { ts += dt * speed; if (ts > T_END) ts = 0; }
    const o = sampleAt(ts);
    const st = { theta: o.th, phi: o.phi, alpha: o.a, s: o.s, rot: o.rot, drag: o.rot * 0, pin: o.pin, sear: o.sear, kick: o.k, dep: o.d, dart: null };
    const F = V.frames(st);
    for (const k in meshes) {
      const m = meshes[k], fr = m.userData.frame;
      if (m.userData.group === 'dart') continue;
      m.matrix.copy(F[fr] || F.world);
    }
    springsAt(F, o.s); tripCordAt(F);
    xray($('#tX').checked || ((o.ph === 'lift' || o.ph === 'launch' || o.ph === 'cock') && camMode === 'auto'));
    rig.position.x = o.rv; rig.updateMatrixWorld(true);
    wheels.forEach(w => w.rotation.y = o.rv / 70);
    // dart
    let Dw;
    if (o.dart) { Dw = new THREE.Matrix4().fromArray(o.dart).transpose(); }
    else { Dw = rig.matrixWorld.clone().multiply(F.dart); }
    st.dart = Dw; const FD = V.frames(st);
    for (const k of dartNames) { const m = meshes[k]; m.matrix.copy(FD[m.userData.frame] || Dw); }
    const tail = new THREE.Vector3(P.DART_L, 0, 0).applyMatrix4(Dw);
    const cgW = new THREE.Vector3(82.5, 0, 0).applyMatrix4(Dw);
    // line
    const eye = new THREE.Vector3(P.REEL_U, 0, P.DART_W).applyMatrix4(F.rail); eye.y = P.REEL_FACE_Y + 4; eye.applyMatrix4(rig.matrixWorld);
    const muz = new THREE.Vector3(P.U_MUZ, 0, P.DART_W - 6).applyMatrix4(F.rail).applyMatrix4(rig.matrixWorld);
    let pts;
    if (!o.dart) pts = [eye.toArray(), tail.toArray()];
    else if (o.ph === 'flight' && o.lp) {
      pts = [eye.toArray(), muz.toArray()].concat(o.lp.filter(p => p[0] > muz.x + 30).map(p => [p[0], p[1], p[2]])); pts.push(tail.toArray());
    } else {
      const sag = o.T > 1 ? 4 : 60;
      const a = muz, b = tail, mid = a.clone().lerp(b, 0.5); mid.z -= sag * Math.min(1, a.distanceTo(b) / 2000);
      pts = [eye.toArray(), a.toArray(), mid.toArray(), b.toArray()];
      if (o.ph === 'land') pts = [eye.toArray(), a.toArray(), b.toArray()];
    }
    setLine(pts);
    // spool pack
    const rPack = Math.sqrt(Math.pow(P.SP_CORE / 2, 2) + o.sp * 1000 * 1.05 / (Math.PI * (P.SP_W - 1)));
    pack.matrix.copy(F.reel).multiply(new THREE.Matrix4().makeTranslation(0, -16 - P.SP_W / 2, 0)).multiply(new THREE.Matrix4().makeScale(rPack, 1, rPack));
    // HUD
    const ph = PHS.filter(p => p.ts <= ts + 1e-6).pop() || PHS[0];
    $('#ptitle').textContent = ph.title; $('#ptext').textContent = ph.text;
    const md = o.md > 0 ? 'FORWARD (reel)' : o.md < 0 ? 'BACKWARD (launcher)' : 'stopped (self locking)';
    $('#ro').innerHTML = `motor <b>${md}</b><br>main shaft ${o.rpm.toFixed(0)} rpm · ${o.Tm.toFixed(2)} N m · ${o.I.toFixed(1)} A<br>program gear ${(((o.phi % 360) + 360) % 360).toFixed(1)}° · rail ${o.th.toFixed(1)}°<br>` +
      `carriage ${(P.P_R - o.s).toFixed(0)} mm back · sear ${o.sear.toFixed(1)}°<br>pickup pin ${o.pin > 0.5 ? 'OUT' : 'in'} · line out ${o.lo.toFixed(2)} m · tension ${o.T.toFixed(0)} N<br>` +
      `real time ${o.t.toFixed(2)} s` + (ph.id === 'launch' ? ' <b>(slow motion 40x)</b>' : '');
    bar.querySelector('.cur') ? 0 : bar.insertAdjacentHTML('beforeend', '<div class="cur"></div>');
    bar.querySelector('.cur').style.left = (100 * ts / T_END) + '%';
    // camera
    if (camMode === 'auto' || camMode in CAMS) {
      let g = camMode === 'auto' ? autoCam(o, cgW) : CAMS[camMode];
      if (camMode !== 'auto' && camMode !== 'field' && camMode !== 'over') g = [[g[0][0] + o.rv, g[0][1], g[0][2]], [g[1][0] + o.rv, g[1][1], g[1][2]]];
      const gp = new THREE.Vector3(...g[0]), gt = new THREE.Vector3(...g[1]);
      if (first) { camPos.copy(gp); camTgt.copy(gt); first = false; }
      const k = 1 - Math.pow(0.02, dt);
      camPos.lerp(gp, k); camTgt.lerp(gt, k);
      cam.position.copy(camPos); ctl.target.copy(camTgt);
    }
    ctl.update();
    // rover tag
    const tp = new THREE.Vector3(o.rv + 70, -230, bz - dk + 150).project(cam);
    lbl.style.left = ((tp.x + 1) / 2 * renderer.domElement.clientWidth) + 'px'; lbl.style.top = ((1 - tp.y) / 2 * renderer.domElement.clientHeight) + 'px';
    V.render();
    window.SIM_TS = ts; requestAnimationFrame(frame);
  }
  window.SIM_SEEK = x => { ts = x; first = true; };
  window.SIM_PAUSE = () => { playing = false; };
  window.SIM_CAM = m => { camMode = m; first = true; };
  requestAnimationFrame(frame);
  window.READY = true;
})();
