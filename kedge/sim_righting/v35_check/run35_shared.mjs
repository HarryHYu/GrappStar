// v35 rerun (pods hinged at the FRONT, x 50, folding aft along the flanks, deploying to a backward rake; the tail tie on the hull):
// the v34 shared deploy worm check, unchanged except that the layout comes from the job (v34 hard-coded 'tail').
// v34 shared deploy worm check. Headless runs of the righting sim (kedge/sim_righting/index.html simCore, extracted unchanged)
// through a subclass of its Sim that changes only what v34 changes:
//  axle 'rigid'    : both pods are ONE rigid body on ONE revolute joint, driven by ONE worm (the sim's worm model: stall 0.21 N m,
//                    3 deg backlash, gear spring 25 N m/rad); both pods' stops act together (2 x the stop spring)
//  axle 'coupled'  : each pod on its own joint, coupled to a massless axle through a torsion spring kC (N m/rad) with blA deg of
//                    residual backlash per pod (total, so +-blA/2), the axle driven by the one worm (same worm model); stops per pod.
//                    The springs are explicit like the sim's own: kC 25 is as stable as the sim; kC 40 gives impact spikes in
//                    about 15 % of runs; kC 60 diverges. (A hard-limit joint chain hull-axle-pods was tried and blew up at impact.)
//  axle 'separate' : the sim as published (one worm per pod), for reference
//  eye             : the real v34 lug, eyeX forward of and eyeY below the pivot (folded), at |y| eyeLat
//  bridle          : legs legLen from the two eyes to a sliding loop (equal leg tension, straight legs, no wrap)
//  firmware        : one deploy channel, so 'podRoll' moves both pods; the plant drives the axle until both tips touch or it
//                    stalls (no levelling with one pod); the drill phase relaxes the one worm when the axle stalls at the stops
//  records         : the worm torque every 1 ms step in every phase (tow split into its 1 s settle and the ramp to 55 N)
// Run from a folder with @dimforge/rapier3d-compat@0.14.0 installed:
//   node run35_shared.mjs '<job json>'   (job = {tag, out, cfg, D, LAYOUTS, cgUp, jobs:[{soil, n, seed0}]}; the exact jobs are in
//   v34_shared_worm.json "jobs"; summarize_shared.py turns the outputs into v34_shared_worm.json)
import fs from 'fs';
import RAPIER from '@dimforge/rapier3d-compat';
const HTML = process.env.KEDGE_SIM || '/home/user/GrappStar/kedge/sim_righting/index.html';
const html = fs.readFileSync(HTML, 'utf8');
const src = html.slice(html.indexOf('function simCore(RAPIER)'), html.indexOf('const { DEG, MM, DT, D, LAYOUTS'));
const simCore = new Function('return (' + src.trim().replace(/;\s*$/, '') + ')')();
await RAPIER.init();
const core = simCore(RAPIER);
const { Sim, D, DEG, MM, DT, V, add, sub, scl, dot, len, norm, qrot, qaxis, lx } = core;

const JOB = JSON.parse(process.argv[2]);
const CFG = Object.assign({ axle: 'rigid', kC: 100, blA: 0, eyeX: 3.9, eyeY: -6.8, eyeLat: 31.65, legLen: 78, origRope: false, tie: false, tieX: 152.0, tieY: -2.7, tieLat: 2.2 }, JOB.cfg || {});
//  tie             : (cfg.tie) the simple v34 tether: the line tied through two holes in the tail face (tieX, tieY, +-tieLat), on the HULL
//                    (legLen 3: the two holes act as one point)
for (const k of Object.keys(JOB.D || {})) Object.assign(core.D[k], JOB.D[k]);
for (const k of Object.keys(JOB.LAYOUTS || {})) Object.assign(core.LAYOUTS[k], JOB.LAYOUTS[k]);
const QI = { x: 0, y: 0, z: 0, w: 1 }, Q_Y2X = qaxis(V(0, 0, 1), -Math.PI / 2);
const dz = (d, b) => (d > b ? d - b : (d < -b ? d + b : 0));

/* a pod as the firmware and soil code see it: its own geometry, the drive's worm */
class PodView {
  constructor(name, drive, src, o) { Object.assign(this, o); this.name = name; this.drive = drive; this.src = src; this.drilling = false; this.anchor = null; this.soil = null; }
  get th() { return this.src.th; } get om() { return this.src.om; } get axisW() { return this.src.axisW; } get deg() { return this.src.th / DEG; }
  get phi() { return this.drive.phi; } get cmd() { return this.drive.cmd; } get stallTime() { return this.drive.stallTime; }
  get stalled() { return this.drive.stalled; } get tauG() { return this.drive.tauG; } get tauS() { return this.drive.tauS; } get target() { return this.drive.target; }
  setTarget(a, b) { this.drive.setTarget(a, b); }
  relax() { this.drive.relax(); }
}

/* one worm -> massless axle -> two pods, each through a torsion spring kC with a deadband (residual backlash) */
class SharedDrive {
  constructor(hA, hB, kC, blADeg) {
    this.name = 'D'; this.pods = [hA, hB];
    this.kG = D.worm.kGear; this.bl = D.worm.backlashDeg * DEG / 2; this.stall = D.worm.stall;
    this.kC = kC; this.blA = blADeg * DEG / 2; this.cC = 2 * 0.15 * Math.sqrt(kC * hA.Ip);
    this.speed = hA.speed; this.speedNow = this.speed; this.min = hA.min; this.max = hA.max;
    this.phi = 0; this.phiDot = 0; this.psi = 0; this.cmd = 0; this.target = 0;
    this.tauG = 0; this.tauGf = 0; this.tauS = 0; this.stalled = false; this.stallTime = 0; this.drive = 0;
  }
  get th() { return 0.5 * (this.pods[0].th + this.pods[1].th); }
  get deg() { return this.th / DEG; }
  setTarget(deg, degS) { this.target = deg * DEG; this.speedNow = degS ? degS * DEG : this.speed; this.stallTime = 0; }
  relax() { // as the sim: the worm backs off to 0.3 deg clear of the flank that stops a fold-back, here measured at the axle
    const p = Math.min(this.psi, this.max + this.blA);
    this.psi = p; this.phi = p + this.bl - 0.3 * DEG; this.target = this.phi; this.cmd = 0; this.tauG = 0; this.tauGf = 0;
  }
  measure(qh, wh) { for (const h of this.pods) h.measure(qh, wh); }
  update(dt, hull) {
    this.phiDot = 0;
    if (this.target !== null) { const e = this.target - this.phi; this.cmd = Math.abs(e) < 1e-5 ? 0 : Math.sign(e); }
    if (this.cmd) { // worm drive and stall sensing exactly as the sim's Hinge
      const load = Math.max(0, this.cmd * this.tauGf); this.drive = load;
      if (load < this.stall) {
        let st = this.cmd * this.speedNow * (1 - load / this.stall) * dt; const e = this.target - this.phi;
        if (Math.abs(st) > Math.abs(e)) st = e; this.phi += st; this.phiDot = st / dt;
      }
      if (load > 0.9 * this.stall) { this.stalled = true; this.stallTime += dt; } else { this.stalled = false; this.stallTime = 0; }
    } else { this.drive = 0; this.stalled = false; this.stallTime = 0; }
    const [A, B] = this.pods, kG = this.kG, kC = this.kC, bl = this.bl, ba = this.blA;
    // massless axle: gear torque = sum of the two coupling torques; g(psi) is non-increasing, take the zero nearest the last psi
    const g = p => kG * dz(this.phi - p, bl) + kC * dz(A.th - p, ba) + kC * dz(B.th - p, ba);
    const lo = Math.min(this.phi, A.th, B.th) - 0.5, hi = Math.max(this.phi, A.th, B.th) + 0.5;
    let a = lo, b = hi;
    for (let i = 0; i < 46; i++) { const m = 0.5 * (a + b); if (g(m) > 0) a = m; else b = m; }
    const zLo = a; a = lo; b = hi;
    for (let i = 0; i < 46; i++) { const m = 0.5 * (a + b); if (g(m) < 0) b = m; else a = m; }
    const zHi = b, psiPrev = this.psi;
    this.psi = Math.min(Math.max(psiPrev, zLo), zHi);
    const eG = dz(this.phi - this.psi, bl) !== 0, eA = dz(this.psi - A.th, ba) !== 0, eB = dz(this.psi - B.th, ba) !== 0;
    const den = (eG ? kG : 0) + (eA ? kC : 0) + (eB ? kC : 0);
    const psiDot = den > 0 ? ((eG ? kG * this.phiDot : 0) + (eA ? kC * A.om : 0) + (eB ? kC * B.om : 0)) / den : (this.psi - psiPrev) / dt;
    let tsum = 0, tsMax = 0;
    for (const P of [A, B]) {
      const d = dz(this.psi - P.th, ba);
      let tc = 0;
      if (d !== 0) { tc = kC * d + this.cC * (psiDot - P.om); if (tc * d < 0) tc = 0; }
      let ts = 0;   // this pod's own hard stops, as the sim's Hinge
      if (P.th > P.max) { ts = -P.kS * (P.th - P.max) - P.cS * P.om; if (ts > 0) ts = 0; }
      else if (P.th < P.min) { ts = P.kS * (P.min - P.th) - P.cS * P.om; if (ts < 0) ts = 0; }
      P.tauG = tc; P.tauS = ts;
      P.body.addTorque(scl(P.axisW, tc + ts), true); hull.addTorque(scl(P.axisW, -(tc + ts)), true);
      tsum += tc; if (Math.abs(ts) > Math.abs(tsMax)) tsMax = ts;
    }
    this.tauG = tsum; this.tauGf += (tsum - this.tauGf) * Math.min(1, dt / 0.02); this.tauS = tsMax;
  }
}

class SimV34 extends Sim {
  constructor(o) {
    super(o);
    this.axle = CFG.axle;
    const P = D.pod, f = this.LY.fold, lat = P.lat * MM, L = P.len * MM, rr = P.r * MM;
    const zS = s => (s === 'L' ? -1 : 1);
    const eyeIn = (s, zOrigin) => V(f * CFG.eyeX * MM, CFG.eyeY * MM, zS(s) * CFG.eyeLat * MM - zOrigin);
    if (CFG.axle === 'rigid') {
      const Hinge = this.leg.constructor, w = this.w, hb = this.hull, R = RAPIER;
      const q0 = hb.rotation(), p0 = hb.translation(), v0 = hb.linvel();
      const mu = this.pods.L.col.friction(), cgrp = this.pods.L.col.collisionGroups();
      for (const s of ['L', 'R']) w.removeRigidBody(this.pods[s].body);   // its collider and joint go with it
      const pivotL = V(lx(this.LY.pivotX), P.pivotY * MM, 0), pw = add(p0, qrot(q0, pivotL)), m = P.mass;
      const b = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(pw.x, pw.y, pw.z).setRotation(q0).setLinvel(v0.x, v0.y, v0.z)
        .setCanSleep(false).setAdditionalMassProperties(2 * m, V(f * L / 2, 0, 0), V(2 * 1.5e-6 + 2 * m * lat * lat, 2 * 2.6e-5 + 2 * m * lat * lat, 2 * 2.6e-5), QI));
      const cols = {};
      for (const s of ['L', 'R']) cols[s] = w.createCollider(R.ColliderDesc.capsule(L / 2 - rr, rr).setTranslation(f * L / 2, 0, zS(s) * lat).setRotation(Q_Y2X)
        .setDensity(0).setFriction(mu).setRestitution(0.02).setRestitutionCombineRule(R.CoefficientCombineRule.Min).setCollisionGroups(cgrp), b);
      const axisL = V(0, 0, -f), dirL = V(f, 0, 0);
      const j = w.createImpulseJoint(R.JointData.revolute(pivotL, V(0, 0, 0), axisL), hb, b, true); j.setContactsEnabled(false);
      const h = new Hinge(this, 'D', b, pivotL, axisL, dirL, { min: 0, max: P.stopDeg * DEG, speed: D.worm.podDegS * DEG, Ip: 2.0e-4 });
      h.kS = 2 * D.stopK; h.cS = 2 * 0.2 * Math.sqrt(h.kS * h.Ip); h.target = 0;   // the two pods' stops act together
      this.drive = h;
      const mk = s => new PodView(s, h, h, { body: b, col: cols[s], pivotL: V(pivotL.x, pivotL.y, zS(s) * lat), axisL, dirL, tipL: V(f * L, 0, zS(s) * lat), tipR: rr });
      this.pods = { L: mk('L'), R: mk('R') };
      this.joints = [h, this.leg];
      this.eyeB = { L: { body: b, p: eyeIn('L', 0) }, R: { body: b, p: eyeIn('R', 0) } };
    } else if (CFG.axle === 'coupled') {
      const hA = this.pods.L, hB = this.pods.R;
      const dr = this.drive = new SharedDrive(hA, hB, CFG.kC, CFG.blA);
      const mk = h => new PodView(h.name, dr, h, { body: h.body, col: h.col, pivotL: h.pivotL, axisL: h.axisL, dirL: h.dirL, tipL: h.tipL, tipR: h.tipR });
      this.pods = { L: mk(hA), R: mk(hB) };
      this.joints = [dr, this.leg];
      this.eyeB = { L: { body: hA.body, p: eyeIn('L', zS('L') * lat) }, R: { body: hB.body, p: eyeIn('R', zS('R') * lat) } };
    } else {
      this.eyeB = { L: { body: this.pods.L.body, p: eyeIn('L', zS('L') * lat) }, R: { body: this.pods.R.body, p: eyeIn('R', zS('R') * lat) } };
    }
    if (CFG.tie) this.eyeB = { L: { body: this.hull, p: V(lx(CFG.tieX), CFG.tieY * MM, -CFG.tieLat * MM) }, R: { body: this.hull, p: V(lx(CFG.tieX), CFG.tieY * MM, CFG.tieLat * MM) } };
    this.recd = {};
  }
  blankPeak() { const p = super.blankPeak(); p.D = { m: 0, s: 0, p: 0 }; return p; }
  eyePts() { return [this.worldPt(this.eyeB.L.body, this.eyeB.L.p), this.worldPt(this.eyeB.R.body, this.eyeB.R.p)]; }
  eyeMid() { if (CFG.origRope) return super.eyeMid(); const [a, b] = this.eyePts(); return scl(add(a, b), 0.5); }
  bridle(eL, eR) { // sliding loop: the ring sits on the spheroid (foci = eyes, leg sum 2 legLen) where its normal points at the reel
    const m = scl(add(eL, eR), 0.5), ev = sub(eL, eR), c = len(ev) / 2, e = scl(ev, 1 / (2 * c));
    const a = CFG.legLen * MM, b2 = a * a - c * c;
    let d = norm(sub(this.reel, m)), ring = m;
    for (let k = 0; k < 2; k++) {
      const dp = dot(d, e), dq = sub(d, scl(e, dp));
      ring = add(m, scl(add(scl(e, a * a * dp), scl(dq, b2)), 1 / Math.sqrt(a * a * dp * dp + b2 * dot(dq, dq))));
      d = norm(sub(this.reel, ring));
    }
    const uL = norm(sub(ring, eL)), uR = norm(sub(ring, eR)), t = this.T / Math.max(len(add(uL, uR)), 1e-6);
    return { ring, uL, uR, t, d };
  }
  ropeForce() {
    if (CFG.origRope) return super.ropeForce();
    this.T += (this.Tset - this.T) * Math.min(1, DT / 0.08);
    const [eL, eR] = this.eyePts(), br = this.bridle(eL, eR);
    this.ropeLen = len(sub(this.reel, br.ring)); this.ropeF = scl(br.d, this.T); this.legT = br.t; this.ring = br.ring;
    this.eyeB.L.body.addForceAtPoint(scl(br.uL, br.t), eL, true);
    this.eyeB.R.body.addForceAtPoint(scl(br.uR, br.t), eR, true);
  }
  runAnchor(dt) {
    if (this.axle === 'separate') return super.runAnchor(dt);
    const c = this.ctl, h = this.drive; c.anchorT += dt;
    if (!c.drillStart) { // plant: one channel, so the axle goes down until both tips touch or it stalls
      let touching = 0;
      for (const s of ['L', 'R']) { const p = this.pods[s], tip = this.worldPt(p.body, p.tipL); if (tip.y < p.tipR + 0.0015) touching++; }
      if (touching === 2 || h.stallTime > 0.3 || c.anchorT > 12) {
        this.result.plantTouch = touching; h.setTarget(h.phi / DEG); c.drillStart = true; this.startDrilling();
      }
      return;
    }
    const thMax = Math.max(this.pods.L.th, this.pods.R.th);
    if ((thMax > (D.pod.stopDeg - 0.8) * DEG && h.stallTime > 0.2) || c.anchorT > 30) {
      h.relax();
      for (const s of ['L', 'R']) {
        const p = this.pods[s], A = p.soil.Anow || this.worldPt(p.body, p.tipL);
        p.soil.fixed = true; p.soil.A = V(A.x, A.y, A.z); p.anchor = true;
        this.note(`${s} drill at ${p.deg.toFixed(1)} deg, worm backed off, anchor set ${(-this.worldPt(p.body, p.tipL).y / MM).toFixed(0)} mm deep`);
      }
      this.result.anchored = 2; this.result.anchorTimeout = c.anchorT > 30;
      this.setPhase('tow'); this.towStart();
    }
  }
  endTow() {
    super.endTow();
    const r = this.result, pk = this.peak;
    r.motorPeak = Math.max(pk.L.m, pk.R.m, pk.K.m, pk.D.m); r.stopPeak = Math.max(pk.L.s, pk.R.s, pk.D.s);
  }
  step() {
    super.step();
    const ph = this.phase === 'tow' && !(this.tow && this.tow.start) ? 'tow_settle' : this.phase;
    const e = this.recd[ph] || (this.recd[ph] = { D: 0, K: 0, S: 0, Dt: null, DT: null, n21: 0 });
    const dj = this.axle === 'separate' ? Math.max(Math.abs(this.pods.L.tauG), Math.abs(this.pods.R.tauG)) : Math.abs(this.drive.tauG);
    if (dj > e.D) { e.D = dj; e.Dt = +this.t.toFixed(3); e.DT = +this.T.toFixed(1); }
    if (dj > D.worm.stall) e.n21++;   // ms above the stall torque
    e.K = Math.max(e.K, Math.abs(this.leg.tauG));
    const sj = this.axle === 'separate' ? Math.max(Math.abs(this.pods.L.tauS), Math.abs(this.pods.R.tauS)) : Math.abs(this.drive.tauS);
    e.S = Math.max(e.S, sj);
  }
}

function runOne(opts, maxT = 120) {
  const Ld = core.landingFor(opts.pose, opts.seed);
  if (opts.gamma != null) Ld.gammaDeg = opts.gamma;   // v35: a fixed arrival path angle (e.g. from the flight check), the seed's roll, yaw and spin
  const s = new SimV34({ ...opts, ...Ld });
  while (!s.finished && s.phase !== 'stuck' && s.t < maxT && !s.result.blowup) s.step();
  const r = { ...s.result, t: +s.t.toFixed(2), phase: s.phase, roll0: Ld.rollDeg, gamma0: Ld.gammaDeg, peak: JSON.parse(JSON.stringify(s.peak)), rec: s.recd,
    th: s.axle === 'separate' ? [s.pods.L.deg, s.pods.R.deg] : [s.pods.L.deg, s.pods.R.deg], events: s.events.map(e => `${e.t.toFixed(2)} ${e.s}`) };
  s.free();
  return r;
}

const out = [];
for (const job of JOB.jobs) {
  const res = [];
  for (let i = 0; i < (job.n || 40); i++) {
    const opts = { pose: job.pose || 'random', seed: (job.seed0 ?? 1000) + i, soil: job.soil, dist: 3, cgUp: job.cgUp ?? JOB.cgUp ?? -3.3, mass: 'brief', layout: job.layout || JOB.layout || 'tail', gamma: job.gamma };
    res.push({ seed: opts.seed, ...runOne(opts, 120) });
  }
  out.push({ job, res });
  process.stderr.write(`done ${JOB.tag || ''} ${JSON.stringify(job)}\n`);
}
fs.writeFileSync(JOB.out, JSON.stringify({ tag: JOB.tag, cfg: CFG, D: JOB.D, out }));
