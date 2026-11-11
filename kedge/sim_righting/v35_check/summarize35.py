"""Summarise run35_shared.mjs outputs into v35_righting_tow.json (v35, 11 Nov 2026; from v34_check/summarize_shared.py, plus the
deploy STOP torque per phase (both stops together on the rigid axle), the landing attitude per run, and the named landing poses).
  python3 summarize35.py OUT.json RAW1.json [RAW2.json ...]
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


def phs(r, p):
    v = r['rec'].get(p, {}).get('S', 0.0)
    if p == 'tow':
        v = max(v, r['rec'].get('done', {}).get('S', 0.0))
    return v


def summ(path):
    d = json.load(open(path)); out = {}
    groups = {}
    for j in d['out']:
        k = j['job']['soil'] + ('' if j['job'].get('pose', 'random') == 'random' else ' poses') + (' fixed paths' if j['job'].get('gamma') is not None else '')
        groups.setdefault(k, []).extend(dict(r, pose=j['job'].get('pose', 'random')) for r in j['res'])
    for soil, rs in groups.items():
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
        stw = [phs(r, 'tow') for r in tw]
        o['deploy_stop_tow_Nm_both'] = dict(max=round(max(stw), 3), median=round(st.median(stw), 3), runs=len(stw))
        o['deploy_stop_by_phase_Nm_both'] = {p: dict(max=round(max(v), 3), median=round(st.median(v), 3)) for p in PH for v in [[phs(r, p) for r in rs if p in r['rec']]] if v}
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
        o['per_run'] = [[r['pose'], r['seed'], round(r['roll0'], 1), round(r['gamma0'], 1), int(r['righted']), r['rightT'] and round(r['rightT'], 1), r['held'],
                         round(ph(r, 'tow'), 3), round(phs(r, 'tow'), 3), round(max(ph(r, 'flight'), ph(r, 'landing')), 3), round(ph(r, 'righting'), 3),
                         [round(x, 2) for x in r['th']]] for r in rs]
        out[soil] = o
    return dict(tag=d['tag'], cfg=d['cfg'], D=d.get('D'), soils=out)


if __name__ == '__main__':
    res = dict(what="v35 righting and tow: kedge/sim_righting/index.html simCore through run35_shared.mjs, layout 'front' (pods hinged at x 50, "
                    "folding aft, deploying to the backward rake), one deploy worm for both pods on the rigid axle, the line tied in the tail face "
                    "(on the hull); 80 random loam landings (seeds 1000-1039, 2000-2039), 20 sand (1000-1019), and the named landing poses, rover 3 m",
               columns_per_run="[pose, seed, landing roll deg, landing path deg down, righted, righting s, held N, tow worm peak N m, tow stop peak N m (both stops), "
                               "impact worm peak (flight + landing) N m, righting worm peak N m, pods at the end deg]",
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
            print(f"{k:20s} {soil:12s}: righted {o['righted']}/{o['runs']} stuck {o['stuck']} held55 {o['held_55N']} moved {o['moved_at_N'][:6]} tow worm {o['deploy_worm_tow_Nm']} stop {o['deploy_stop_tow_Nm_both']} spikes {o['spike_runs_over_4Nm']}")
