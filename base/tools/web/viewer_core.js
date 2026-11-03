// base/tools/web/viewer_core.js: generic three.js scene builder for the CAD pipeline.
// Input: window.CAD_DATA (built by tools/render.py build_data):
//   parts:  [{name, color, material, notes, subsystem, joint, bought_ref, printed, qty, mass_g, group, place[16]}]
//   meshes: {name: {q: [ox, oy, oz, s], v: b64 int16 xyz, f: b64 uint16 or uint32, wide: bool}}
//   tracks: {n, t[], groups: [{m[16]} static | {q: b64 int16 xyzw * n, p: b64 float32 xyz * n}]}
// Part world matrix at sample s = group(s) * place (row-major 4x4 in the data).
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
    const idx = b64(md.f, md.wide ? Uint32Array : Uint16Array);
    return creased(pos, idx, 32);
  }

  function decodeTracks(T) {
    const n = T.n;
    return T.groups.map(g => {
      if (g.m) return { stat: rowMajor(g.m) };
      const qi = b64(g.q, Int16Array), p = b64(g.p, Float32Array), qs = [], ps = [];
      for (let i = 0; i < n; i++) {
        qs.push(new Q(qi[4 * i] / 32767, qi[4 * i + 1] / 32767, qi[4 * i + 2] / 32767, qi[4 * i + 3] / 32767).normalize());
        ps.push(new V3(p[3 * i], p[3 * i + 1], p[3 * i + 2]));
      }
      return { qs, ps };
    });
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

  function build(container, data, opts) {
    opts = opts || {};
    const hd = !!opts.hd;
    const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1) * (hd && (window.devicePixelRatio || 1) < 2 ? 1.5 : 1));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.localClippingEnabled = true;
    renderer.shadowMap.enabled = opts.shadows !== false;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.0;
    container.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const bg = new THREE.Color(opts.bg || '#eef1f4'); scene.background = bg;
    const dark = bg.getHSL({}).l < 0.5;
    const cam = new THREE.PerspectiveCamera(30, container.clientWidth / Math.max(1, container.clientHeight), 0.5, 1e6);
    cam.up.set(0, 0, 1);
    const ctl = THREE.OrbitControls ? new THREE.OrbitControls(cam, renderer.domElement) : null;
    if (ctl) { ctl.enableDamping = false; ctl.screenSpacePanning = true; }

    // lights
    scene.add(new THREE.HemisphereLight(0xffffff, dark ? 0x30343a : 0x8a94a0, dark ? 0.55 : 0.7));
    const key = new THREE.DirectionalLight(0xffffff, 0.85); key.castShadow = true;
    key.shadow.mapSize.set(hd ? 4096 : 2048, hd ? 4096 : 2048); key.shadow.bias = -0.0005; key.shadow.normalBias = 0.02;
    scene.add(key); scene.add(key.target);
    const fill = new THREE.DirectionalLight(0xffffff, 0.35); scene.add(fill);
    const rim = new THREE.DirectionalLight(0xffffff, 0.2); scene.add(rim);

    // clipping plane: keeps coord <= value along the chosen axis
    const clip = new THREE.Plane(new V3(-1, 0, 0), 1e9), planes = [clip];

    // parts
    const groups = decodeTracks(data.tracks), nS = data.tracks.n;
    const parts = {}, list = [];
    for (const p of data.parts) {
      const md = data.meshes[p.name]; if (!md) continue;
      const geo = decodeMesh(md);
      const metal = p.material === 'steel' || (p.material === 'bought' && /brg|bearing|608|steel|shaft|screw|bolt|nut|pin/i.test(p.name + ' ' + (p.bought_ref || '')));
      const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(p.color || '#c8ccd2').convertSRGBToLinear(), roughness: metal ? 0.38 : 0.6,
        metalness: metal ? 0.55 : 0.04, clippingPlanes: planes, clipShadows: true });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = mesh.receiveShadow = true; mesh.matrixAutoUpdate = false;
      const cap = new THREE.Mesh(geo, capMaterial(p.color || '#c8ccd2', planes));
      cap.matrixAutoUpdate = false; cap.visible = false; cap.renderOrder = 1;
      scene.add(mesh); scene.add(cap);
      geo.computeBoundingBox();
      const rec = { p, mesh, cap, mat, place: rowMajor(p.place), grp: groups[p.group], world: new M4(), off: new V3(), dir: new V3(), hidden: false };
      mesh.userData.rec = rec; parts[p.name] = rec; list.push(rec);
    }

    // poses
    const tq = new Q(), tp = new V3(), one = new V3(1, 1, 1), tm = new M4();
    let curS = 0, expl = 0, needs = true;
    const mark = () => { needs = true; };
    function groupAt(g, s) {
      if (g.stat) return tm.copy(g.stat);
      const n = g.qs.length; s = Math.max(0, Math.min(n - 1, s));
      const i = Math.min(n - 2, Math.floor(s)), f = n > 1 ? s - i : 0;
      if (n === 1) return tm.compose(g.ps[0], g.qs[0], one);
      tq.copy(g.qs[i]).slerp(g.qs[i + 1], f); tp.copy(g.ps[i]).lerp(g.ps[i + 1], f);
      return tm.compose(tp, tq, one);
    }
    function apply() {
      needs = true;
      for (const r of list) {
        r.world.copy(groupAt(r.grp, curS)).multiply(r.place);
        r.mesh.matrix.copy(r.world);
        if (expl) { r.mesh.matrix.premultiply(new M4().makeTranslation(r.dir.x * expl, r.dir.y * expl, r.dir.z * expl)); }
        if (r.off.lengthSq()) r.mesh.matrix.premultiply(new M4().makeTranslation(r.off.x, r.off.y, r.off.z));
        r.cap.matrix.copy(r.mesh.matrix);
        r.mesh.matrixWorld.copy(r.mesh.matrix); r.cap.matrixWorld.copy(r.mesh.matrix);
      }
    }
    function sample(s) { curS = +s; apply(); }

    // bounding box of the visible parts in the current pose
    function bbox(names) {
      const b = new THREE.Box3(), tb = new THREE.Box3();
      for (const r of list) {
        if (names ? names.indexOf(r.p.name) < 0 : (!r.mesh.visible)) continue;
        tb.copy(r.mesh.geometry.boundingBox).applyMatrix4(r.mesh.matrix); b.union(tb);
      }
      if (b.isEmpty()) b.set(new V3(-50, -50, 0), new V3(50, 50, 50));
      return b;
    }
    sample(0);
    const box0 = bbox(), c0 = box0.getCenter(new V3()), size0 = box0.getSize(new V3()), R0 = Math.max(10, size0.length() / 2);
    for (const r of list) {
      const c = r.mesh.geometry.boundingBox.getCenter(new V3()).applyMatrix4(r.world);
      r.dir.copy(c).sub(c0);
    }

    // ground, grid, shadows sized to the assembly
    // ground at the bbox min z (origin is on the main shaft axis, the deck is below z = 0) unless the data sets it
    const gz = (typeof data.ground_z === 'number' ? Math.min(data.ground_z, box0.min.z) : box0.min.z) - Math.max(0.5, R0 * 0.004);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(R0 * 40, R0 * 40),
      new THREE.ShadowMaterial({ opacity: dark ? 0.45 : 0.22 }));
    ground.position.set(c0.x, c0.y, gz); ground.receiveShadow = true; scene.add(ground);
    const step = Math.pow(10, Math.floor(Math.log10(R0 / 2)));
    const gridN = Math.ceil(R0 * 6 / step);
    const grid = new THREE.GridHelper(gridN * step, gridN, dark ? 0x3a4048 : 0xc3c8cf, dark ? 0x2a2f36 : 0xdde1e6);
    grid.rotation.x = Math.PI / 2; grid.position.set(c0.x, c0.y, gz - 0.01); scene.add(grid);
    grid.material.transparent = true; grid.material.opacity = dark ? 0.45 : 0.8; grid.material.depthWrite = false;
    function lights() {
      const R = R0;
      key.position.set(c0.x + R * 1.2, c0.y - R * 1.6, c0.z + R * 3.0); key.target.position.copy(c0);
      const sc = key.shadow.camera; sc.left = sc.bottom = -R * 1.6; sc.right = sc.top = R * 1.6; sc.near = R * 0.1; sc.far = R * 8; sc.updateProjectionMatrix();
      fill.position.set(c0.x - R * 2, c0.y + R * 1.5, c0.z + R * 0.8);
      rim.position.set(c0.x - R, c0.y + R * 2, c0.z - R * 1.5);
    }
    lights();

    let cutState = null;
    // camera helpers: target, distance, azimuth (deg from +x toward +y), elevation (deg)
    const orb = { t: c0.clone(), r: R0 * 3, az: -55, el: 28 };
    function placeCam() {
      const a = orb.az * D2R, e = orb.el * D2R, ce = Math.cos(e);
      cam.position.set(orb.t.x + orb.r * ce * Math.cos(a), orb.t.y + orb.r * ce * Math.sin(a), orb.t.z + orb.r * Math.sin(e));
      cam.near = Math.max(0.1, orb.r / 200); cam.far = orb.r * 50; cam.updateProjectionMatrix();
      cam.lookAt(orb.t); if (ctl) { ctl.target.copy(orb.t); ctl.update(); }
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
      box = box || bbox();
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
    function highlight(names, ghost) {
      needs = true;
      const set = new Set(names || []);
      for (const r of list) {
        const on = set.has(r.p.name);
        if (on) r.mat.emissive.copy(r.mat.color); else r.mat.emissive.set(0x000000);   // brighten in its own hue
        r.mat.emissiveIntensity = on ? (hlLevel == null ? 0.3 : hlLevel) : 0;
        const gh = !!ghost && set.size > 0 && !on;
        if (r.mat.transparent !== gh) { r.mat.transparent = gh; r.mat.opacity = gh ? 0.25 : 1; r.mat.depthWrite = !gh; r.mat.needsUpdate = true; }
        r.mesh.castShadow = !gh;
      }
    }
    function show(names) {           // null = all
      needs = true;
      const set = names ? new Set(names) : null;
      for (const r of list) { r.mesh.visible = !r.hidden && (!set || set.has(r.p.name)); r.cap.visible = r.mesh.visible && !!cutState; }
    }
    function setOffset(name, v) { const r = parts[name]; if (r) r.off.set(v[0], v[1], v[2]); needs = true; }
    function setGround(on) { ground.visible = on; grid.visible = on; needs = true; }

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
    function renderIfNeeded() { if (needs) render(); }
    view(-55, 28);

    const api = {
      renderer, scene, cam, ctl, parts, list, data, nSamples: nS, dark, box0, center: c0, radius: R0,
      sample, cut, explode, highlight, highlightLevel, show, setOffset, setGround, pick, bbox, view, fitDist, render, renderIfNeeded, resize, mark,
      cam: camSet, camGet, cutState: () => cutState, sampleIndex: () => curS,
    };
    return api;
  }

  // window.AUD-like API for headless shots
  function installAUD(v) {
    window.AUD = {
      ready: () => !!v && v.list.length > 0,
      info: () => ({ box: [v.box0.min.toArray(), v.box0.max.toArray()], center: v.center.toArray(), radius: v.radius,
        n: v.nSamples, parts: v.list.map(r => r.p.name) }),
      cam: (t, r, az, el) => { v.cam(t, r, az, el); v.render(); },
      view: (az, el, margin) => { v.view(az, el, margin); v.render(); },
      cut: (axis, value, flip) => { v.cut(axis, value, flip); v.render(); },
      sample: i => { v.sample(i); v.render(); },
      highlight: (names, ghost) => { v.highlight(names, ghost); v.render(); },
      explode: f => { v.explode(f); v.render(); },
      ground: on => { v.setGround(on); v.render(); },
      show: names => { v.show(names); v.render(); },
      render: () => v.render(),
    };
    return window.AUD;
  }

  // Start an app once the data is complete: CAD_DATA.mesh_files (relative URLs of JSON files {name: mesh}) are
  // fetched and merged into CAD_DATA.meshes first, so large assemblies can keep the meshes next to the page.
  function boot(main) {
    const D = window.CAD_DATA, files = (D && D.mesh_files) || [];
    const go = () => { try { main(); } catch (e) { showError(e); } };
    if (!files.length) { if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', go); else go(); return; }
    Promise.all(files.map(u => fetch(u).then(r => { if (!r.ok) throw new Error(u + ': ' + r.status); return r.json(); })))
      .then(parts => { D.meshes = D.meshes || {}; for (const m of parts) Object.assign(D.meshes, m); go(); })
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
