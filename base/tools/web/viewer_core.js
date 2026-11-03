// base/tools/web/viewer_core.js: generic three.js scene builder for the CAD pipeline.
// Input: window.CAD_DATA (built by tools/render.py build_data):
//   parts:  [{name, color, material, notes, subsystem, joint, bought_ref, printed, qty, mass_g, group, place[16]}]
//   meshes: {name: {q: [ox, oy, oz, s], v: b64 int16 xyz, f: b64 uint16 or uint32, h: hull vertex indices, wide: bool}}
//   tracks: {n, t[], joints: [{name, parent, kind, origin[3], axis[3], base[16] | null, v: b64 values * n | c}],
//            line: {cnt: b64 uint16 * n, p: b64 float32 xyz} (tether polyline per sample, optional)}
//   box_track: world bbox over all samples
// Group 0 is the world; group k is joint k - 1: world = parent * T(origin) * base * motion(value), the value
// (rev: degrees, slide: mm) interpolated linearly between samples. Part world matrix = group * place (row-major).
// CADV.build(container, data, opts) returns a viewer with sample(), cut(), explode(), cam(), highlight(), shot API.
(function () {
  'use strict';
  const V3 = THREE.Vector3, M4 = THREE.Matrix4, Q = THREE.Quaternion;
  const D2R = Math.PI / 180;

  function b64(s, T) {
    const bin = atob(s), u = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
    return new T(u.buffer);
  }
  function rowMajor(a) { const m = new M4(); m.set(...a); return m; }

  // Indexed mesh -> non-indexed geometry with creased normals (smooth below the crease angle, sharp above).
  function creased(pos, idx, creaseDeg) {
    const nf = idx.length / 3, nv = pos.length / 3, cosT = Math.cos((creaseDeg || 32) * D2R);
    const fn = new Float32Array(nf * 3), fa = new Float32Array(nf);
    for (let f = 0; f < nf; f++) {
      const a = idx[3 * f] * 3, b = idx[3 * f + 1] * 3, c = idx[3 * f + 2] * 3;
      const ux = pos[b] - pos[a], uy = pos[b + 1] - pos[a + 1], uz = pos[b + 2] - pos[a + 2];
      const vx = pos[c] - pos[a], vy = pos[c + 1] - pos[a + 1], vz = pos[c + 2] - pos[a + 2];
      let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      const l = Math.hypot(nx, ny, nz) || 1; fa[f] = l;
      fn[3 * f] = nx / l; fn[3 * f + 1] = ny / l; fn[3 * f + 2] = nz / l;
    }
    const cnt = new Uint32Array(nv + 1);
    for (let i = 0; i < idx.length; i++) cnt[idx[i] + 1]++;
    for (let i = 0; i < nv; i++) cnt[i + 1] += cnt[i];
    const adj = new Uint32Array(idx.length), fill = cnt.slice(0, nv);
    for (let i = 0; i < idx.length; i++) adj[fill[idx[i]]++] = (i / 3) | 0;
    const P = new Float32Array(nf * 9), N = new Float32Array(nf * 9);
    for (let f = 0; f < nf; f++) {
      const fx = fn[3 * f], fy = fn[3 * f + 1], fz = fn[3 * f + 2];
      for (let k = 0; k < 3; k++) {
        const v = idx[3 * f + k], o = 9 * f + 3 * k;
        P[o] = pos[3 * v]; P[o + 1] = pos[3 * v + 1]; P[o + 2] = pos[3 * v + 2];
        let sx = 0, sy = 0, sz = 0;
        for (let j = cnt[v]; j < cnt[v + 1]; j++) {
          const g = adj[j], gx = fn[3 * g], gy = fn[3 * g + 1], gz = fn[3 * g + 2];
          if (gx * fx + gy * fy + gz * fz >= cosT) { const w = fa[g]; sx += gx * w; sy += gy * w; sz += gz * w; }
        }
        const l = Math.hypot(sx, sy, sz) || 1;
        N[o] = sx / l; N[o + 1] = sy / l; N[o + 2] = sz / l;
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(P, 3));
    g.setAttribute('normal', new THREE.BufferAttribute(N, 3));
    return g;
  }

  function decodeMesh(md) {
    const q = b64(md.v, Int16Array), s = md.q[3], pos = new Float32Array(q.length);
    for (let i = 0; i < q.length; i += 3) { pos[i] = md.q[0] + q[i] * s; pos[i + 1] = md.q[1] + q[i + 1] * s; pos[i + 2] = md.q[2] + q[i + 2] * s; }
    const IT = md.wide ? Uint32Array : Uint16Array, idx = b64(md.f, IT);
    let hull = pos;
    if (typeof md.h === 'string') {
      const h = b64(md.h, IT); hull = new Float32Array(3 * h.length);
      for (let i = 0; i < h.length; i++) { hull[3 * i] = pos[3 * h[i]]; hull[3 * i + 1] = pos[3 * h[i] + 1]; hull[3 * i + 2] = pos[3 * h[i] + 2]; }
    }
    return { geo: creased(pos, idx, 32), hull };
  }

  // Joint-space tracks: joints [{parent, kind, origin, axis, base, v (b64 float32/64 per sample) | c (constant)}],
  // parents before children. group 0 = world, group k = joint k - 1.
  function decodeTracks(T) {
    return (T.joints || []).map(j => {
      const pre = new M4().makeTranslation(j.origin[0], j.origin[1], j.origin[2]);
      if (j.base) pre.multiply(rowMajor(j.base));
      const vals = typeof j.v === 'string' ? b64(j.v, j.f64 ? Float64Array : Float32Array) : null;
      return { name: j.name, parent: j.parent, kind: j.kind, axis: new V3(j.axis[0], j.axis[1], j.axis[2]), pre, vals, c: +j.c || 0, world: new M4() };
    });
  }
  function jointValue(J, s) {
    const a = J.vals; if (!a) return J.c;
    const n = a.length; if (n === 1) return a[0];
    s = Math.max(0, Math.min(n - 1, s));
    const i = Math.min(n - 2, Math.floor(s)), f = s - i;
    return a[i] + (a[i + 1] - a[i]) * f;
  }
  // Tether polyline per sample: {cnt: b64 uint16 per sample, p: b64 float32 xyz of all points in sample order}
  function decodeLine(L) {
    if (!L || typeof L.p !== 'string') return null;
    const cnt = b64(L.cnt, Uint16Array), p = b64(L.p, Float32Array), off = new Uint32Array(cnt.length + 1);
    for (let i = 0; i < cnt.length; i++) off[i + 1] = off[i] + cnt[i];
    let mx = 0; for (let i = 0; i < cnt.length; i++) mx = Math.max(mx, cnt[i]);
    return { cnt, p, off, max: mx };
  }

  // cut faces: back faces drawn flat with a screen-space hatch, so a sectioned solid reads as filled
  function capMaterial(color, planes) {
    const c = new THREE.Color(color).convertSRGBToLinear();
    const hsl = c.getHSL({}); if (hsl.l < 0.22) c.setHSL(hsl.h, hsl.s, 0.22);     // dark parts: keep the hatch readable
    const m = new THREE.MeshBasicMaterial({ color: c, side: THREE.BackSide, clippingPlanes: planes,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -4 });
    m.onBeforeCompile = sh => {
      sh.fragmentShader = sh.fragmentShader.replace('#include <dithering_fragment>',
        '#include <dithering_fragment>\n float hz = mod(gl_FragCoord.x + gl_FragCoord.y, 9.0);\n' +
        ' gl_FragColor.rgb = mix(gl_FragColor.rgb * 0.62, vec3(0.08), hz < 1.6 ? 0.55 : 0.0);');
    };
    return m;
  }

  // theme colours for the scene (the page background comes from the CSS tokens)
  function sceneTheme(dark) {
    return dark ? { hemiGround: 0x30343a, hemi: 0.55, grid1: 0x3a4048, grid2: 0x2a2f36, gridOp: 0.45, shadow: 0.45, key: 0.8 }
      : { hemiGround: 0x8a94a0, hemi: 0.7, grid1: 0xc3c8cf, grid2: 0xdde1e6, gridOp: 0.8, shadow: 0.22, key: 0.85 };
  }

  function build(container, data, opts) {
    opts = opts || {};
    const hd = !!opts.hd, shot = !!window.CAD_SHOT;       // CAD_SHOT: headless capture (shots.js), cheaper settings
    const dpr = window.devicePixelRatio || 1;
    const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    renderer.setPixelRatio(shot ? Math.min(2, dpr) : Math.min(2, dpr) * (hd && dpr < 2 ? 1.5 : 1));   // shots: no hd boost (ratio 1 unless the job sets a device scale)
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.localClippingEnabled = true;
    renderer.shadowMap.enabled = opts.shadows !== false;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.0;
    container.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const bg = new THREE.Color(opts.bg || '#eef1f4'); scene.background = bg;
    let dark = opts.dark != null ? !!opts.dark : bg.getHSL({}).l < 0.5;
    let TH = sceneTheme(dark);
    const cam = new THREE.PerspectiveCamera(30, container.clientWidth / Math.max(1, container.clientHeight), 0.5, 1e6);
    cam.up.set(0, 0, 1);
    const ctl = THREE.OrbitControls ? new THREE.OrbitControls(cam, renderer.domElement) : null;
    if (ctl) { ctl.enableDamping = false; ctl.screenSpacePanning = true; }

    // lights
    const hemi = new THREE.HemisphereLight(0xffffff, TH.hemiGround, TH.hemi); scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, TH.key); key.castShadow = true;
    const sm = shot ? 1024 : hd ? 4096 : 2048;
    key.shadow.mapSize.set(sm, sm); key.shadow.bias = -0.0005; key.shadow.normalBias = 0.02;
    scene.add(key); scene.add(key.target);
    const fill = new THREE.DirectionalLight(0xffffff, 0.35); scene.add(fill);
    const rim = new THREE.DirectionalLight(0xffffff, 0.2); scene.add(rim);

    // clipping plane: keeps coord <= value along the chosen axis
    const clip = new THREE.Plane(new V3(-1, 0, 0), 1e9), planes = [clip];

    // parts
    const joints = decodeTracks(data.tracks), nS = data.tracks.n;
    const I4 = new M4();
    const groupWorld = g => (g > 0 && joints[g - 1]) ? joints[g - 1].world : I4;
    const parts = {}, list = [];
    for (const p of data.parts) {
      const md = data.meshes[p.name]; if (!md) continue;
      const dm = decodeMesh(md), geo = dm.geo;
      const metal = p.material === 'steel' || (p.material === 'bought' && /brg|bearing|608|steel|shaft|screw|bolt|nut|pin/i.test(p.name + ' ' + (p.bought_ref || '')));
      const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(p.color || '#c8ccd2').convertSRGBToLinear(), roughness: metal ? 0.38 : 0.6,
        metalness: metal ? 0.55 : 0.04, clippingPlanes: planes, clipShadows: true });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = mesh.receiveShadow = true; mesh.matrixAutoUpdate = false;
      const capMat = capMaterial(p.color || '#c8ccd2', planes);
      const cap = new THREE.Mesh(geo, capMat);
      cap.matrixAutoUpdate = false; cap.visible = false; cap.renderOrder = 1;
      scene.add(mesh); scene.add(cap);
      geo.computeBoundingBox();
      const rec = { p, mesh, cap, mat, capMat, hull: dm.hull, place: rowMajor(p.place), group: p.group || 0, world: new M4(), off: new V3(), dir: new V3(), hidden: false };
      mesh.userData.rec = rec; parts[p.name] = rec; list.push(rec);
    }

    // tether line: unit cylinders (along y) instanced per segment, radius >= ~1.3 px on screen
    const LN = decodeLine(data.tracks.line);
    let line = null;
    if (LN && LN.max > 1) {
      const lg = new THREE.CylinderGeometry(1, 1, 1, 8, 1, false);
      const lm = new THREE.MeshStandardMaterial({ color: new THREE.Color('#ff9a1f').convertSRGBToLinear(), roughness: 0.5, metalness: 0,
        emissive: new THREE.Color('#ff9a1f').convertSRGBToLinear(), emissiveIntensity: 0.25 });
      line = new THREE.InstancedMesh(lg, lm, LN.max - 1); line.count = 0; line.frustumCulled = false; line.castShadow = true;
      line.renderOrder = 2; scene.add(line);
    }
    const linePts = [];
    function lineAt(s) {
      linePts.length = 0; if (!LN) return linePts;
      const n = LN.cnt.length; s = Math.max(0, Math.min(n - 1, s));
      const i = Math.min(Math.max(0, n - 2), Math.floor(s)), f = n > 1 ? s - i : 0, j = Math.min(n - 1, i + 1);
      const ci = LN.cnt[i], cj = LN.cnt[j], P = LN.p;
      if (ci === cj && j !== i) {
        const a = 3 * LN.off[i], b = 3 * LN.off[j];
        for (let k = 0; k < 3 * ci; k += 3) linePts.push(P[a + k] + (P[b + k] - P[a + k]) * f, P[a + k + 1] + (P[b + k + 1] - P[a + k + 1]) * f, P[a + k + 2] + (P[b + k + 2] - P[a + k + 2]) * f);
      } else {
        const m = f < 0.5 ? i : j, a = 3 * LN.off[m];
        for (let k = 0; k < 3 * LN.cnt[m]; k++) linePts.push(P[a + k]);
      }
      return linePts;
    }
    const lq = new Q(), lv = new V3(), lu = new V3(0, 1, 0), ls = new V3(), lmtx = new M4(), lmid = new V3();
    function updateLine() {
      if (!line) return;
      const pts = linePts, ns = pts.length / 3 - 1;
      const pxk = 2 * Math.tan(cam.fov * D2R / 2) / Math.max(1, renderer.domElement.clientHeight || container.clientHeight);
      let k = 0;
      for (let i = 0; i < ns; i++) {
        lv.set(pts[3 * i + 3] - pts[3 * i], pts[3 * i + 4] - pts[3 * i + 1], pts[3 * i + 5] - pts[3 * i + 2]);
        const len = lv.length(); if (len < 1e-6) continue;
        lmid.set(pts[3 * i] + lv.x / 2, pts[3 * i + 1] + lv.y / 2, pts[3 * i + 2] + lv.z / 2);
        const rad = Math.max(0.5, 1.3 * pxk * lmid.distanceTo(cam.position));      // 1 mm Dyneema, never thinner than ~2.6 px
        lq.setFromUnitVectors(lu, lv.multiplyScalar(1 / len)); ls.set(rad, len + rad, rad);
        line.setMatrixAt(k++, lmtx.compose(lmid, lq, ls));
      }
      line.count = k; line.instanceMatrix.needsUpdate = true;
    }

    // poses
    let curS = 0, expl = 0, needs = true, frozen = false;
    const mark = () => { needs = true; };
    const mot = new M4(), tm = new M4();
    function poseJoints(s) {
      for (const J of joints) {
        const v = jointValue(J, s);
        if (J.kind === 'rev') mot.makeRotationAxis(J.axis, v * D2R);
        else if (J.kind === 'slide') mot.makeTranslation(J.axis.x * v, J.axis.y * v, J.axis.z * v);
        else mot.identity();
        J.world.copy(J.parent >= 0 ? joints[J.parent].world : I4).multiply(J.pre).multiply(mot);
      }
    }
    function apply() {
      needs = true;
      poseJoints(curS);
      for (const r of list) {
        r.world.multiplyMatrices(groupWorld(r.group), r.place);
        r.mesh.matrix.copy(r.world);
        if (expl) { r.mesh.matrix.premultiply(tm.makeTranslation(r.dir.x * expl, r.dir.y * expl, r.dir.z * expl)); }
        if (r.off.lengthSq()) r.mesh.matrix.premultiply(tm.makeTranslation(r.off.x, r.off.y, r.off.z));
        r.cap.matrix.copy(r.mesh.matrix);
        r.mesh.matrixWorld.copy(r.mesh.matrix); r.cap.matrixWorld.copy(r.mesh.matrix);
      }
      lineAt(curS); updateLine();
    }
    function sample(s) { curS = +s; apply(); }

    // bounding box of the visible (or named) parts in the current pose: exact, from the transformed hull vertices
    function bbox(names) {
      const b = new THREE.Box3(), set = names ? new Set(names) : null;
      let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
      for (const r of list) {
        if (set ? !set.has(r.p.name) : !r.mesh.visible) continue;
        const e = r.mesh.matrix.elements, h = r.hull;
        for (let i = 0; i < h.length; i += 3) {
          const x = h[i], y = h[i + 1], z = h[i + 2];
          const X = e[0] * x + e[4] * y + e[8] * z + e[12], Y = e[1] * x + e[5] * y + e[9] * z + e[13], Z = e[2] * x + e[6] * y + e[10] * z + e[14];
          if (X < x0) x0 = X; if (X > x1) x1 = X; if (Y < y0) y0 = Y; if (Y > y1) y1 = Y; if (Z < z0) z0 = Z; if (Z > z1) z1 = Z;
        }
      }
      if (x0 > x1) b.set(new V3(-50, -50, 0), new V3(50, 50, 50)); else b.set(new V3(x0, y0, z0), new V3(x1, y1, z1));
      return b;
    }
    sample(0);
    const box0 = bbox(), c0 = box0.getCenter(new V3()), size0 = box0.getSize(new V3()), R0 = Math.max(10, size0.length() / 2);
    for (const r of list) {
      const c = r.mesh.geometry.boundingBox.getCenter(new V3()).applyMatrix4(r.world);
      r.dir.copy(c).sub(c0);
    }
    // whole track bbox (all samples, computed by tools/render.py) for the ground, grid, cut range and mission view
    const boxT = data.box_track ? new THREE.Box3(new V3(...data.box_track[0]), new V3(...data.box_track[1])).union(box0) : box0.clone();
    if (LN) for (let i = 0; i < LN.p.length; i += 3) boxT.expandByPoint(new V3(LN.p[i], LN.p[i + 1], LN.p[i + 2]));
    const cT = boxT.getCenter(new V3()), RT = Math.max(R0, boxT.getSize(new V3()).length() / 2);

    // ground at the lowest point of the whole track (origin is on the main shaft axis, the deck is below z = 0)
    // unless the data sets it; ground and grid cover the track bbox plus a margin
    const gz = (typeof data.ground_z === 'number' ? Math.min(data.ground_z, boxT.min.z) : boxT.min.z) - Math.max(0.5, R0 * 0.004);
    const gsize = Math.max(R0 * 40, RT * 6);
    const groundMat = new THREE.ShadowMaterial({ opacity: TH.shadow });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(gsize, gsize), groundMat);
    ground.position.set(cT.x, cT.y, gz); ground.receiveShadow = true; scene.add(ground);
    let grid = null, groundOn = true;
    function makeGrid() {
      if (grid) { scene.remove(grid); grid.geometry.dispose(); grid.material.dispose(); }
      const ext = Math.max(R0 * 6, RT * 2.4);
      let step = Math.pow(10, Math.floor(Math.log10(R0 / 2)));
      while (ext / step > 400) step *= 10;                 // keep the line count sane for metre-scale tracks
      const n = Math.ceil(ext / step);
      grid = new THREE.GridHelper(n * step, n, TH.grid1, TH.grid2);
      grid.rotation.x = Math.PI / 2; grid.position.set(cT.x, cT.y, gz - 0.01);
      grid.material.transparent = true; grid.material.opacity = TH.gridOp; grid.material.depthWrite = false;
      grid.visible = groundOn; scene.add(grid);
    }
    makeGrid();
    function lights() {
      const R = R0;                     // shadows sized to the machine (sharp), not to the whole flight path
      key.position.set(c0.x + R * 1.2, c0.y - R * 1.6, c0.z + R * 3.0); key.target.position.copy(c0);
      const sc = key.shadow.camera; sc.left = sc.bottom = -R * 1.6; sc.right = sc.top = R * 1.6; sc.near = R * 0.1; sc.far = R * 8; sc.updateProjectionMatrix();
      fill.position.set(c0.x - R * 2, c0.y + R * 1.5, c0.z + R * 0.8);
      rim.position.set(c0.x - R, c0.y + R * 2, c0.z - R * 1.5);
    }
    lights();
    function setTheme(isDark, bgColor) {
      dark = !!isDark; TH = sceneTheme(dark);
      if (bgColor) scene.background = new THREE.Color(bgColor);
      hemi.groundColor.set(TH.hemiGround); hemi.intensity = TH.hemi; key.intensity = TH.key;
      groundMat.opacity = TH.shadow; makeGrid(); needs = true; api.dark = dark;
    }

    let cutState = null;
    // camera helpers: target, distance, azimuth (deg from +x toward +y), elevation (deg)
    const orb = { t: c0.clone(), r: R0 * 3, az: -55, el: 28 };
    function placeCam() {
      const a = orb.az * D2R, e = orb.el * D2R, ce = Math.cos(e);
      cam.position.set(orb.t.x + orb.r * ce * Math.cos(a), orb.t.y + orb.r * ce * Math.sin(a), orb.t.z + orb.r * Math.sin(e));
      cam.near = Math.max(0.1, orb.r / 200); cam.far = orb.r * 50 + RT * 4; cam.updateProjectionMatrix();
      cam.lookAt(orb.t); if (ctl) { ctl.target.copy(orb.t); ctl.update(); }
      updateLine();
    }
    function camSet(t, r, az, el) {
      needs = true;
      if (t) orb.t.set(t[0], t[1], t[2]);
      if (r != null) orb.r = r; if (az != null) orb.az = az; if (el != null) orb.el = el; placeCam();
    }
    function fitDist(box, margin) {
      const s = box.getSize(new V3()), R = Math.max(1, s.length() / 2);
      const vf = cam.fov * D2R / 2, hf = Math.atan(Math.tan(vf) * cam.aspect);
      return R / Math.sin(Math.min(vf, hf)) * (margin || 1.0);
    }
    function view(az, el, margin, box) {
      box = box ? box.clone() : bbox();
      if (cutState && !box.isEmpty()) {           // frame the material that is left after the cut
        const k = cutState.axis, c = cutState.value;
        if (cutState.flip) box.min[k] = Math.min(box.max[k], Math.max(box.min[k], c));
        else box.max[k] = Math.max(box.min[k], Math.min(box.max[k], c));
      }
      camSet(box.getCenter(new V3()).toArray(), fitDist(box, margin || 0.95), az, el);
    }
    function camGet() { return { t: orb.t.toArray(), r: orb.r, az: orb.az, el: orb.el }; }
    if (ctl) ctl.addEventListener('change', () => {
      needs = true;
      const d = cam.position.clone().sub(ctl.target); orb.t.copy(ctl.target); orb.r = d.length();
      orb.el = Math.asin(Math.max(-1, Math.min(1, d.z / orb.r))) / D2R; orb.az = Math.atan2(d.y, d.x) / D2R;
      updateLine();
    });

    // section cut
    function cut(axis, value, flip) {
      needs = true;
      if (!axis || axis === 'none' || value == null) {
        cutState = null; clip.normal.set(-1, 0, 0); clip.constant = 1e9;
        for (const r of list) r.cap.visible = false;
        return;
      }
      const n = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] }[axis];
      const sg = flip ? 1 : -1;
      clip.normal.set(n[0] * sg, n[1] * sg, n[2] * sg); clip.constant = flip ? -value : value;
      cutState = { axis, value, flip: !!flip };
      for (const r of list) r.cap.visible = r.mesh.visible;
    }
    function explode(f) { expl = +f || 0; apply(); }
    let hlLevel = null;
    function highlightLevel(x) { hlLevel = x; for (const r of list) if (r.mat.emissiveIntensity > 0) r.mat.emissiveIntensity = x; needs = true; }
    function ghostMat(m, gh) {
      if (m.transparent === gh) return;
      m.transparent = gh; m.opacity = gh ? 0.25 : 1; m.depthWrite = !gh; m.needsUpdate = true;
    }
    function highlight(names, ghost) {
      needs = true;
      const set = new Set(names || []);
      for (const r of list) {
        const on = set.has(r.p.name);
        if (on) r.mat.emissive.copy(r.mat.color); else r.mat.emissive.set(0x000000);   // brighten in its own hue
        r.mat.emissiveIntensity = on ? (hlLevel == null ? 0.3 : hlLevel) : 0;
        const gh = !!ghost && set.size > 0 && !on;
        ghostMat(r.mat, gh); ghostMat(r.capMat, gh);             // cut caps of ghosted parts are ghosted too
        r.mesh.castShadow = !gh;
      }
    }
    function show(names) {           // null = all
      needs = true;
      const set = names ? new Set(names) : null;
      for (const r of list) { r.mesh.visible = !r.hidden && (!set || set.has(r.p.name)); r.cap.visible = r.mesh.visible && !!cutState; }
    }
    function setOffset(name, v) { const r = parts[name]; if (r) r.off.set(v[0], v[1], v[2]); needs = true; }
    function setGround(on) { groundOn = !!on; ground.visible = groundOn; grid.visible = groundOn; needs = true; }

    // picking (respects the cut)
    const ray = new THREE.Raycaster();
    function pick(clientX, clientY) {
      const rc = renderer.domElement.getBoundingClientRect();
      ray.setFromCamera({ x: (clientX - rc.left) / rc.width * 2 - 1, y: -((clientY - rc.top) / rc.height * 2 - 1) }, cam);
      const hits = ray.intersectObjects(list.filter(r => r.mesh.visible).map(r => r.mesh), false);
      for (const h of hits) if (clip.distanceToPoint(h.point) >= 0) return h.object.userData.rec;
      return null;
    }

    function resize() {
      const w = container.clientWidth, h = Math.max(1, container.clientHeight);
      renderer.setSize(w, h); cam.aspect = w / h; cam.updateProjectionMatrix(); needs = true;
    }
    window.addEventListener('resize', resize);
    function render() { renderer.render(scene, cam); needs = false; }
    function renderIfNeeded() { if (needs && !frozen) render(); }
    // freeze: stop redrawing from the animation loops (shots set a fixed pose, render once, then capture)
    function freeze(on) { frozen = !!on; if (!frozen) needs = true; }
    view(-55, 28);

    // moving joint groups with their parts (for "follow")
    const movers = [];
    joints.forEach((J, k) => {
      const names = list.filter(r => r.group === k + 1).map(r => r.p.name);
      let moving = false; for (let i = k + 1; i > 0 && !moving; i = joints[i - 1].parent + 1) moving = !!joints[i - 1].vals;
      if (names.length && moving) movers.push({ name: J.name, group: k + 1, parts: names });
    });

    const api = {
      renderer, scene, cam, ctl, parts, list, data, nSamples: nS, dark, box0, center: c0, radius: R0, boxTrack: boxT, joints, movers,
      sample, cut, explode, highlight, highlightLevel, show, setOffset, setGround, setTheme, pick, bbox, view, fitDist, render, renderIfNeeded, resize, mark, freeze,
      cam: camSet, camGet, cutState: () => cutState, sampleIndex: () => curS, line: () => linePts.slice(), frozen: () => frozen,
      showLine: on => { if (line) { line.visible = !!on; needs = true; } },
    };
    return api;
  }

  // window.AUD-like API for headless shots
  function installAUD(v) {
    window.AUD = {
      ready: () => !!v && v.list.length > 0,
      info: () => ({ box: [v.box0.min.toArray(), v.box0.max.toArray()], center: v.center.toArray(), radius: v.radius,
        box_track: [v.boxTrack.min.toArray(), v.boxTrack.max.toArray()], n: v.nSamples, parts: v.list.map(r => r.p.name) }),
      cam: (t, r, az, el) => { v.cam(t, r, az, el); v.render(); },
      view: (az, el, margin) => { v.view(az, el, margin); v.render(); },
      cut: (axis, value, flip) => { v.cut(axis, value, flip); v.render(); },
      sample: i => { v.sample(i); v.render(); },
      highlight: (names, ghost) => { v.highlight(names, ghost); v.render(); },
      explode: f => { v.explode(f); v.render(); },
      ground: on => { v.setGround(on); v.render(); },
      show: names => { v.show(names); v.render(); },
      theme: dark => { v.setTheme(dark); v.render(); },
      freeze: on => { v.freeze(on !== false); if (on !== false) v.render(); },
      render: () => v.render(),
    };
    return window.AUD;
  }

  // Start an app once the data is complete: CAD_DATA.ext_files (relative URLs of JSON files {id.piece: string})
  // are fetched first, and every {ext: id, n: pieces} placeholder in CAD_DATA.meshes and CAD_DATA.tracks is replaced
  // by the joined string, so large assemblies can keep their arrays next to the page.
  function resolveExt(o, store) {
    if (!o || typeof o !== 'object') return o;
    if (typeof o.ext === 'string' && typeof o.n === 'number') {
      let s = ''; for (let i = 0; i < o.n; i++) { const p = store[o.ext + '.' + i]; if (p == null) throw new Error('missing data piece ' + o.ext + '.' + i); s += p; }
      return s;
    }
    for (const k of Object.keys(o)) { const v = o[k]; if (v && typeof v === 'object') o[k] = resolveExt(v, store); }
    return o;
  }
  function boot(main) {
    const D = window.CAD_DATA, files = (D && (D.ext_files || D.mesh_files)) || [];
    const go = () => { try { main(); } catch (e) { showError(e); } };
    if (!files.length) { if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', go); else go(); return; }
    Promise.all(files.map(u => fetch(u).then(r => { if (!r.ok) throw new Error(u + ': ' + r.status); return r.json(); })))
      .then(parts => {
        const store = {}; for (const m of parts) Object.assign(store, m);
        D.meshes = resolveExt(D.meshes || {}, store); D.tracks = resolveExt(D.tracks, store); go();
      })
      .catch(showError);
  }
  function showError(e) {
    const d = document.createElement('div');
    d.style.cssText = 'position:fixed;left:16px;right:16px;top:16px;padding:10px;background:#fff3f0;color:#8a1c00;border:1px solid #e0a090;border-radius:6px;font:13px Arial;z-index:9';
    d.textContent = 'Could not load the model: ' + (e && e.message || e); document.body.appendChild(d);
    console.error(e);
  }

  window.CADV = { build, installAUD, b64, boot };
})();
