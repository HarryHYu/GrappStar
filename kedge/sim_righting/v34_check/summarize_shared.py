"""Summarise run34_shared.mjs outputs into v34_shared_worm.json (dart-physics, 9 Nov 2026).
  python3 summarize_shared.py OUT.json RAW1.json [RAW2.json ...]
Per configuration and soil: righted / stuck / held 55 N / moved-at tension, the deploy worm torque in the tow (every 1 ms step, the
review's 0.55 N m metric; also the sim's own 100 Hz tow peak), the same per phase, the phase holding each run's maximum, the leg
worm, and a compact row per run: [seed, righted, held N, tow peak, impact peak (flight+landing), righting peak, anchor peak] (N m)."""
import json, statistics as st, sys

PH = ['flight', 'landing', 'settle', 'righting', 'align', 'anchor', 'tow_settle', 'tow']
STALL = 0.21


def ph(r, p):
    v = r['rec'].get(p, {}).get('D', 0.0)
    if p == 'tow':   # the step that ends the tow is logged as 'done'
        v = max(v, r['rec'].get('done', {}).get('D', 0.0))
    return v


def summ(path):
    d = json.load(open(path)); out = {}
    for soil in ('loam', 'sand'):
        rs = [r for j in d['out'] if j['job']['soil'] == soil for r in j['res']]
        if not rs:
            continue
        tw = [r for r in rs if r['rec'].get('tow')]
        tow = [ph(r, 'tow') for r in tw]
        mp = [r['motorPeak'] for r in tw]
        rt = [r['rightT'] for r in rs if r['righted']]
        o = dict(runs=len(rs), righted=sum(r['righted'] for r in rs), stuck=sum(bool(r['stuck']) for r in rs),
                 blowup=sum(bool(r['blowup']) for r in rs), anchored=sum(r.get('anchored') == 2 for r in rs),
                 anchor_by_timeout=sum(bool(r.get('anchorTimeout')) for r in rs),
                 right_s_median=round(st.median(rt), 1) if rt else None, right_s_max=round(max(rt), 1) if rt else None,
                 held_55N=sum(r['held'] >= 55 for r in rs), moved_at_N=sorted(round(r['movedAt'], 1) for r in rs if r.get('movedAt') is not None))
        o['deploy_worm_tow_Nm'] = dict(max=round(max(tow), 3), median=round(st.median(tow), 3), runs_over_stall=sum(v > STALL for v in tow), runs=len(tow))
        o['deploy_worm_tow_sim_100Hz_Nm'] = dict(max=round(max(mp), 3), median=round(st.median(mp), 3), runs_over_stall=sum(v > STALL for v in mp))
        o['deploy_worm_by_phase_Nm'] = {}
        for p in PH:
            v = [ph(r, p) for r in rs if p in r['rec']]
            if v:
                o['deploy_worm_by_phase_Nm'][p] = dict(max=round(max(v), 3), median=round(st.median(v), 3), runs_over_stall=sum(x > STALL for x in v), runs=len(v))
        c1, c2 = {}, {}
        for r in rs:
            m = max(PH, key=lambda p: ph(r, p)); c1[m] = c1.get(m, 0) + 1
            m = max([p for p in PH if p not in ('flight', 'landing')], key=lambda p: ph(r, p)); c2[m] = c2.get(m, 0) + 1
        o['phase_of_each_runs_max'] = c1; o['phase_of_each_runs_max_after_impact'] = c2
        o['leg_worm_max_Nm'] = {p: round(max(r['rec'].get(p, {}).get('K', 0) for r in rs), 3) for p in PH if any(p in r['rec'] for r in rs)}
        w = max(tw, key=lambda r: ph(r, 'tow'))
        o['worst_tow_run'] = dict(seed=w['seed'], Nm=round(ph(w, 'tow'), 3), t_s=w['rec']['tow'].get('Dt'), tension_N=w['rec']['tow'].get('DT'),
                                  pods_end_deg=[round(x, 2) for x in w['th']])
        o['spike_runs_over_4Nm'] = sum(max(v.get('D', 0) for v in r['rec'].values()) > 4 for r in rs)
        o['per_run'] = [[r['seed'], int(r['righted']), r['held'], round(ph(r, 'tow'), 3), round(max(ph(r, 'flight'), ph(r, 'landing')), 3),
                         round(ph(r, 'righting'), 3), round(ph(r, 'anchor'), 3)] for r in rs]
        out[soil] = o
    return dict(tag=d['tag'], cfg=d['cfg'], D=d.get('D'), soils=out)


if __name__ == '__main__':
    res = dict(what="v34 shared deploy worm: kedge/sim_righting/index.html simCore through run34_shared.mjs (one deploy worm for both pods, "
                    "real eye lug, 78 mm sliding-loop bridle), 80 random loam landings (seeds 1000-1039, 2000-2039) and 20 sand (1000-1019), rover 3 m",
               columns_per_run="[seed, righted, held N, tow worm peak, impact worm peak (flight + landing), righting peak, anchor peak] (N m)",
               stall_Nm=STALL, configs={})
    res['jobs'] = {}
    for p in sys.argv[2:]:
        s = summ(p); res['configs'][s['tag']] = s
        try:   # the job that made it (run34_shared.mjs argument), when it sits beside the output as .job
            res['jobs'][s['tag']] = json.load(open(p[:-5] + '.job'))
        except OSError:
            pass
    json.dump(res, open(sys.argv[1], 'w'), indent=1)
    for k, s in res['configs'].items():
        for soil, o in s['soils'].items():
            print(f"{k:45s} {soil}: righted {o['righted']}/{o['runs']} stuck {o['stuck']} held55 {o['held_55N']} tow {o['deploy_worm_tow_Nm']} spikes {o['spike_runs_over_4Nm']}")
