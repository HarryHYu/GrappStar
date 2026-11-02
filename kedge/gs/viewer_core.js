// GrappStar ground station: shared three.js scene + kinematics (mirrors pose.py)
(function () {
  const GS = window.GS, P = GS.P;
  function b64(s, T) { const bin = atob(s); const u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return new T(u.buffer); }
  const M4 = THREE.Matrix4;
  function ry(aDeg, cx = 0, cz = 0) {
    // CCW in (x, z), x toward z (nose up). three's makeRotationY(a) maps x toward -z, so use -a
    const a = aDeg * Math.PI / 180;
    const m = new M4().makeRotationY(-a);
    return new M4().makeTranslation(cx, 0, cz).multiply(m).multiply(new M4().makeTranslation(-cx, 0, -cz));
  }
  function tr(x = 0, y = 0, z = 0) { return new M4().makeTranslation(x, y, z); }
  function frames(st) {
    const H = P.H, th = st.theta;
    const Rr = tr(0, 0, H).multiply(ry(th));
    const F = { world: new M4(), rail: Rr };
    F.pg = Rr.clone().multiply(tr(P.PG_U, 0, 0)).multiply(ry(st.phi));
    F.main = tr(0, 0, H).multiply(ry(st.alpha));
    F.main_pin = tr(0, 0, H).multiply(ry(th - st.phi * P.Z_PG / P.Z_PIN + (P.PIN_PHASE || 0)));
    F.sled = Rr.clone().multiply(tr(st.s, 0, 0));
    F.reel = Rr.clone().multiply(tr(P.REEL_U, P.REEL_FACE_Y, P.DART_W));
    F.rotor = F.reel.clone().multiply(ry(st.rot));
    F.drag = F.reel.clone().multiply(ry(st.drag));
    F.pin = F.rotor.clone().multiply(tr(-3.5 * (1 - st.pin), 0, 0));
    const sp = P.SEAR_PIV;
    F.sear = Rr.clone().multiply(ry(st.sear, sp[0], sp[1]));
    const L = P.DART_L;
    const Dm = new M4().set(-1, 0, 0, st.s + L, 0, -1, 0, 0, 0, 0, 1, P.DART_W, 0, 0, 0, 1);
    F.dart = st.dart ? st.dart : Rr.clone().multiply(Dm);
    for (const g in (P.DART_PIV || {})) {
      const pv = P.DART_PIV[g], ang = g.indexOf('kick') >= 0 ? (st.kick || 0) : -(st.dep || 0);
      F[g] = F.dart.clone().multiply(tr(pv[0], pv[1], pv[2])).multiply(ry(ang));
    }
    return F;
  }
  function thetaOf(phi) {
    const t = GS.theta, n = t.length - 1; let x = ((phi % 360) + 360) % 360 / 360 * n; const i = Math.floor(x), f = x - i;
    return t[i] * (1 - f) + t[Math.min(n, i + 1)] * f;
  }
  function sledPos(phi, fired) {
    const tab = P.RACK_TAB; phi = ((phi % 360) + 360) % 360;
    const x = phi >= 360 + tab[0][0] ? phi - 360 : phi;
    if (x >= tab[0][0] && x < P.PH_REL) {
      const i = Math.min(tab.length - 2, Math.floor((x - tab[0][0]) / 0.5)); const a = tab[i], b = tab[i + 1];
      return P.P_R - (a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]));
    }
    if (fired || x < tab[0][0]) return P.P_R;
    return P.S_SETTLED !== undefined ? P.S_SETTLED : P.P_R - (P.STROKE - (P.OVERTRAVEL || 0.5));
  }
  function interp(tab, x) {
    if (x <= tab[0][0]) return tab[0][1]; if (x >= tab[tab.length - 1][0]) return tab[tab.length - 1][1];
    let lo = 0, hi = tab.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tab[m][0] <= x) lo = m; else hi = m; }
    const a = tab[lo], b = tab[hi]; return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]);
  }
  function searAngle(th, s) {      // mesh-solved tables: keel contact and trip cord, whichever turns the hook further down
    let d = interp(P.SEAR_CORD_TAB, th);
    const kt = P.SEAR_KEEL_TAB; if (s >= kt[0][0] && s <= kt[kt.length - 1][0]) d = Math.min(d, interp(kt, s));
    return d;
  }
  function build(container, opts = {}) {
    const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1) * (opts.hd ? 1.5 : 1));
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding; renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.05;
    container.appendChild(renderer.domElement);
    const scene = new THREE.Scene(); scene.background = new THREE.Color(opts.bg || 0xf3f4f6);
    const cam = new THREE.PerspectiveCamera(35, container.clientWidth / container.clientHeight, 5, 200000);
    cam.up.set(0, 0, 1); cam.position.set(650, -900, 520);
    const ctl = new THREE.OrbitControls(cam, renderer.domElement); ctl.target.set(120, 0, 120); ctl.update();
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.55));
    const key = new THREE.DirectionalLight(0xffffff, 0.9); key.position.set(500, -700, 1200); key.castShadow = true;
    key.shadow.mapSize.set(4096, 4096); const sc = key.shadow.camera; sc.left = -900; sc.right = 900; sc.top = 900; sc.bottom = -900; sc.near = 10; sc.far = 4000; key.shadow.bias = -0.0004;
    scene.add(key); const fill = new THREE.DirectionalLight(0xffffff, 0.35); fill.position.set(-600, 500, 400); scene.add(fill);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(60000, 60000), new THREE.MeshStandardMaterial({ color: opts.groundColor || 0xe6e7e9, roughness: 1 }));
    ground.position.z = -8.01; ground.receiveShadow = true; scene.add(ground);
    const grid = new THREE.GridHelper(4000, 80, 0xc8c8c8, 0xdadada); grid.rotation.x = Math.PI / 2; grid.position.z = -8; scene.add(grid);
    const meshes = {};
    for (const p of GS.parts) {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(b64(p.v, Float32Array), 3));
      g.setIndex(new THREE.BufferAttribute(b64(p.f, Uint32Array), 1));
      g.computeVertexNormals();
      const metal = !p.printed && p.group === "hardware";
      const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(p.color), roughness: metal ? 0.35 : 0.62, metalness: metal ? 0.6 : 0.05, flatShading: false });
      const m = new THREE.Mesh(g, mat); m.castShadow = true; m.receiveShadow = true; m.matrixAutoUpdate = false;
      m.userData = p; scene.add(m); meshes[p.name] = m;
    }
    function pose(st) {
      const F = frames(st);
      for (const k in meshes) { const fr = meshes[k].userData.frame; meshes[k].matrix.copy(F[fr] || F.world); }
      return F;
    }
    function resize() { renderer.setSize(container.clientWidth, container.clientHeight); cam.aspect = container.clientWidth / container.clientHeight; cam.updateProjectionMatrix(); }
    window.addEventListener("resize", resize);
    return { renderer, scene, cam, ctl, meshes, pose, frames, thetaOf, render: () => renderer.render(scene, cam) };
  }
  window.GSV = { build, frames, thetaOf, ry, tr, sledPos, searAngle };
})();
