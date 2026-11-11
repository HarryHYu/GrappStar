// Instrumented headless run of the KEDGE v34 landing / righting / anchoring sim for the render motion.
// The sim code is NOT copied: simCore is cut from kedge/sim_righting/index.html and the v34 subclasses (PodView,
// SharedDrive, SimV34) are cut from kedge/sim_righting/v34_check/run34_shared.mjs at run time, exactly as
// run34_shared.mjs itself does with index.html, so the physics is byte-identical to the validated batch.
// This file only adds a recorder (hull pose, leg / pod angles, drilling flags, line tension, rope end, phases, moves).
//
//   node sim_dump.mjs '<job json>'
//   job = { out, cfg, D, LAYOUTS, cgUp, soil, scan: [seeds] }            -> per-seed summary (moves, righting, anchor)
//   job = { out, cfg, D, LAYOUTS, cgUp, soil, seed, dump: true, dtFine, tFine, dtCoarse, maxT } -> per-step trace
//   job = { ..., inits: [{id, init}] }                                    -> per-start summary (landing search)
//   job = { ..., init, dump: true, ... }                                  -> per-step trace from a given start state
//   init = { p: body origin, q: [w,x,y,z], v: velocity of point cg, w: angular velocity, cg, reel } in sim world
// Needs @dimforge/rapier3d-compat@0.14.0 resolvable from the working directory (NODE_PATH or a local node_modules).
import fs from 'fs';
import path from 'path';
import { createRequire } from 'module';

const ROOT = process.env.GS_ROOT || path.resolve(path.dirname(new URL(import.meta.url).pathname), '..', '..');
const HTML = path.join(ROOT, 'kedge/sim_righting/index.html');
const RUN34 = path.join(ROOT, 'kedge/sim_righting/v34_check/run34_shared.mjs');
const require = createRequire(path.join(process.cwd(), 'x.js'));
const RAPIER = (await import(require.resolve('@dimforge/rapier3d-compat'))).default;

const html = fs.readFileSync(HTML, 'utf8');
const src = html.slice(html.indexOf('function simCore(RAPIER)'), html.indexOf('const { DEG, MM, DT, D, LAYOUTS'));
const simCore = new Function('return (' + src.trim().replace(/;\s*$/, '') + ')')();
await RAPIER.init();
const core = simCore(RAPIER);
const { Sim, D, DEG, MM, DT, V, add, sub, scl, dot, cross, len, norm, qrot, qmul, qconj, qaxis, lx } = core;

const JOB = JSON.parse(process.argv[2]);
const CFG = Object.assign({ axle: 'rigid', kC: 100, blA: 0, eyeX: 3.9, eyeY: -6.8, eyeLat: 31.65, legLen: 78, origRope: false, tie: false, tieX: 152.0, tieY: -2.7, tieLat: 2.2 }, JOB.cfg || {});
for (const k of Object.keys(JOB.D || {})) Object.assign(core.D[k], JOB.D[k]);
for (const k of Object.keys(JOB.LAYOUTS || {})) Object.assign(core.LAYOUTS[k], JOB.LAYOUTS[k]);
const QI = { x: 0, y: 0, z: 0, w: 1 }, Q_Y2X = qaxis(V(0, 0, 1), -Math.PI / 2);
const dz = (d, b) => (d > b ? d - b : (d < -b ? d + b : 0));

// the v34 classes, unchanged, from run34_shared.mjs
const r34 = fs.readFileSync(RUN34, 'utf8');
const cls = r34.slice(r34.indexOf('/* a pod as the firmware'), r34.indexOf('function runOne'));
const { SimV34 } = new Function('RAPIER', 'core', 'CFG', 'Sim', 'D', 'DEG', 'MM', 'DT', 'V', 'add', 'sub', 'scl', 'dot', 'len', 'norm',
  'qrot', 'qaxis', 'lx', 'QI', 'Q_Y2X', 'dz', cls + '\nreturn { SimV34 };')(RAPIER, core, CFG, Sim, D, DEG, MM, DT, V, add, sub, scl, dot,
  len, norm, qrot, qaxis, lx, QI, Q_Y2X, dz);

const r6 = x => +x.toFixed(6);

class Rec extends SimV34 {
  constructor(o) {
    super(o);
    if (o.init) this.applyInit(o.init);
    this.moveLog = []; this.phaseLog = [{ t: 0, p: this.phase }]; this.lastPhase = this.phase;
  }
  // start from a given state (the render flight's end): every body moved rigidly with the hull (the joints are
  // relative, so they stay closed), velocities = the rigid-body field v + w x (com - cg); the reel at the base exit
  applyInit(I) {
    const hb = this.hull, p0 = hb.translation(), q0 = hb.rotation();
    const pd = V(...I.p), qd = { w: I.q[0], x: I.q[1], y: I.q[2], z: I.q[3] };
    const qr = qmul(qd, qconj(q0));
    const bodies = [...new Set([hb, this.leg.body, this.pods.L.body, this.pods.R.body])];
    const v = V(...I.v), w = V(...I.w), cg = V(...I.cg);
    for (const b of bodies) {
      const p = b.translation(), q = b.rotation();
      const pn = add(pd, qrot(qr, sub(p, p0))), qn = qmul(qr, q);
      b.setTranslation(pn, true); b.setRotation(qn, true);
    }
    for (const b of bodies) {
      const c = b.worldCom();
      const vb = add(v, cross(w, sub(V(c.x, c.y, c.z), cg)));
      b.setLinvel(vb, true); b.setAngvel(w, true);
    }
    if (I.reel) this.reel = V(...I.reel);
    if (I.gravity) this.w.gravity = V(...I.gravity);
  }
  assess() {
    const n0 = this.ctl.tries; super.assess();
    if (this.ctl.tries > n0) this.moveLog.push({ t: r6(this.t), move: this.ctl.moveName, tilt: r6(this.tiltDeg()) });
  }
  step() {
    super.step();
    if (this.phase !== this.lastPhase) { this.phaseLog.push({ t: r6(this.t), p: this.phase }); this.lastPhase = this.phase; }
  }
  sample() {
    const h = this.hull, p = h.translation(), q = h.rotation(), v = h.linvel(), w = h.angvel();
    const P = this.pods, L = P.L, R = P.R;
    const e = this.eyePts ? this.eyeMid() : this.eyeMid();
    const tipL = this.worldPt(L.body, L.tipL), tipR = this.worldPt(R.body, R.tipL);
    return [r6(this.t), this.phase, r6(p.x), r6(p.y), r6(p.z), r6(q.w), r6(q.x), r6(q.y), r6(q.z), r6(v.x), r6(v.y), r6(v.z),
      r6(w.x), r6(w.y), r6(w.z), r6(this.leg.th / DEG), r6(L.th / DEG), r6(R.th / DEG), L.drilling ? 1 : 0, R.drilling ? 1 : 0,
      L.anchor ? 1 : 0, R.anchor ? 1 : 0, r6(this.T), r6(e.x), r6(e.y), r6(e.z), r6(tipL.y), r6(tipR.y), r6(this.tiltDeg())];
  }
}
const COLS = ['t', 'phase', 'px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'leg_deg', 'podL_deg', 'podR_deg',
  'drillL', 'drillR', 'anchorL', 'anchorR', 'T', 'eye_x', 'eye_y', 'eye_z', 'tipL_y', 'tipR_y', 'tilt'];

function make(seed, init) {
  // with an init state the preset landing only builds the bodies; applyInit then moves them to the given state
  const Ld = init ? { rollDeg: 0, gammaDeg: 45, yawDeg: 0, spin: V() } : core.landingFor(JOB.pose || 'random', seed);
  const opts = { pose: JOB.pose || 'random', seed, soil: JOB.soil || 'loam', dist: 3, cgUp: JOB.cgUp ?? -3.3, mass: 'brief', layout: 'tail', init };
  return { s: new Rec({ ...opts, ...Ld }), Ld };
}

function summary(s, Ld, seed) {
  return { seed, righted: s.result.righted, rightT: s.result.rightT, moves: s.moveLog, phases: s.phaseLog, anchored: s.result.anchored,
    held: s.result.held, movedAt: s.result.movedAt, stuck: s.result.stuck, blowup: s.result.blowup, t: r6(s.t), landing: Ld,
    events: s.events.map(e => `${e.t.toFixed(3)} ${e.s}`), th: [s.pods.L.deg, s.pods.R.deg] };
}

const maxT = JOB.maxT || 120;
if (JOB.inits) {
  const out = [];
  for (const it of JOB.inits) {
    const { s, Ld } = make(0, it.init);
    while (!s.finished && s.phase !== 'stuck' && s.t < maxT && !s.result.blowup) s.step();
    const r = summary(s, Ld, it.id); r.id = it.id; r.rest = { tilt: r6(s.tiltDeg()) };
    const h = s.hull.translation(); r.final_p = [r6(h.x), r6(h.y), r6(h.z)];
    out.push(r); s.free();
  }
  fs.writeFileSync(JOB.out, JSON.stringify(out));
} else if (JOB.scan) {
  const out = [];
  for (const seed of JOB.scan) {
    const { s, Ld } = make(seed);
    while (!s.finished && s.phase !== 'stuck' && s.t < maxT && !s.result.blowup) s.step();
    out.push(summary(s, Ld, seed)); s.free();
    process.stderr.write(`seed ${seed} done\n`);
  }
  fs.writeFileSync(JOB.out, JSON.stringify(out));
} else {
  const { s, Ld } = make(JOB.seed, JOB.init);
  const rows = [s.sample()];
  const dtF = JOB.dtFine || 0.002, tF = JOB.tFine || 3.0, dtC = JOB.dtCoarse || 0.01;
  let next = dtF;
  while (!s.finished && s.phase !== 'stuck' && s.t < maxT && !s.result.blowup) {
    s.step();
    if (s.t >= next - 1e-9) { rows.push(s.sample()); next += s.t < tF ? dtF : dtC; }
  }
  rows.push(s.sample());
  const meta = { cfg: CFG, D: JOB.D, LAYOUTS: JOB.LAYOUTS, cgUp: JOB.cgUp, soil: JOB.soil, seed: JOB.seed, init: JOB.init || null, DT, reel: s.reel,
    hull: core.D.hull, pod: core.D.pod, leg: core.D.leg, worm: core.D.worm, axes: 'sim world: x launch direction, y up, z right; body: x nose, y top, z right, origin on the hull axis at the CG station (cgX mm from the nose)' };
  fs.writeFileSync(JOB.out, JSON.stringify({ meta, summary: summary(s, Ld, JOB.seed), cols: COLS, rows }));
  s.free();
}
