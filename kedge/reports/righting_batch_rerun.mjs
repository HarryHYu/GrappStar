// Headless rerun of the KEDGE righting sim batch: extracts simCore() from kedge/sim_righting/index.html unchanged.
import fs from 'fs';
import RAPIER from '@dimforge/rapier3d-compat';
const html = fs.readFileSync('/home/user/GrappStar/kedge/sim_righting/index.html', 'utf8');
const a = html.indexOf('function simCore(RAPIER)');
const b = html.indexOf('const { DEG, MM, DT, D, LAYOUTS');
const src = html.slice(a, b);
const simCore = new Function('return (' + src.trim().replace(/;\s*$/, '') + ')')();
await RAPIER.init();
const core = simCore(RAPIER);
const args = JSON.parse(process.argv[2]);
const out = [];
for (const job of args.jobs) {
  const res = [];
  for (let i = 0; i < (job.n || 50); i++) {
    const opts = { pose: job.pose || 'random', seed: (job.seed0 ?? 1000) + i, soil: job.soil, dist: 3, cgUp: job.cgUp, mass: job.mass, layout: job.layout };
    const r = core.runHeadless(opts, 120);
    res.push({ seed: opts.seed, righted: r.righted, rightT: r.rightT, moves: r.moves, stuck: r.stuck, anchored: r.anchored, held: r.held, movedAt: r.movedAt,
      motorPeak: r.motorPeak, stopPeak: r.stopPeak, aligned: r.alignTried || false, blowup: r.blowup, roll0: r.landing.rollDeg, gamma0: r.landing.gammaDeg, pitchPeak: r.pitchPeak, liftPeak: r.liftPeak });
  }
  out.push({ job, res });
  process.stderr.write(`done ${JSON.stringify(job)}\n`);
}
fs.writeFileSync(args.out, JSON.stringify(out));
