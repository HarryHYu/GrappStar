"""Whole-base integration audit (cad-frame's integration auditor role).

python cad/parts/_frame_tests/integration.py [--dt 1.0] [--jobs 2]   (from base/)
Runs tools.audit on every subsystem module present (frame, drum, launch, launch_aim, reel, ... = every module in
cad/parts with parts/joints/kin), then writes base/cad/parts/frame_INTEGRATION.md: offending pairs with owners,
worst state (phase, t), overlap volume; failed clearances; floating parts; static fails; subsystem check failures.
"""
import argparse
import importlib
import json
import os
import subprocess
import sys
import time

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, BASE)
OUT_MD = os.path.join(BASE, "cad", "parts", "frame_INTEGRATION.md")


def subsystems():
    from tools.assembly import subsystem_names
    out = []
    for n in subsystem_names():
        if n == "demo":
            continue
        try:
            m = importlib.import_module(f"cad.parts.{n}")
        except Exception as e:                      # a module that does not even import is a finding too
            out.append((n, f"IMPORT ERROR {type(e).__name__}: {e}"))
            continue
        if all(hasattr(m, f) for f in ("parts", "joints", "kin")):
            out.append((n, None))
    return out


def origin_check():
    """Markdown lines: per subsystem, the lowest part z in the rest state and the root joint origins, against
    LAYOUT.deck_top_z (BASE_SPEC section 2: origin on the main axis)."""
    import numpy as np
    from tools.params import load_params
    from tools.assembly import Assembly
    from cad.lib import prim
    P = load_params()
    deck = (getattr(P, "LAYOUT", {}) or {}).get("deck_top_z", -165.0)
    names = [n for n, e in subsystems() if e is None]
    L = ["## Frame-origin check", "", f"Spec: main axis z = 0, deck top z = {deck}. Lowest part bottom per subsystem (rest state):", "",
         "| subsystem | lowest part z | root joints (parent None) | verdict |", "|---|---|---|---|"]
    try:
        asm = Assembly.load(P, names)
    except Exception as e:
        return L + [f"| (all) | load failed: {type(e).__name__}: {e} | | |", ""]
    W = asm.world({})
    for sub in names:
        zs = [prim.bbox(prim.place(p.solid, W[n]))[0][2] for n, p in asm.parts.items() if p.subsystem == sub]
        zs = [z for z in zs if np.isfinite(z)]
        roots = [f"{j.name} {tuple(round(v, 1) for v in j.origin)}" for j in asm.joints.values()
                 if j.parent is None and j.name.startswith(sub)][:3]
        zmin = min(zs) if zs else float("nan")
        ok = abs(zmin - deck) < 2.0 or zmin > deck
        verdict = "ok" if ok and zmin < 50 else ("parts below the deck" if zmin < deck - 2 else "check: lowest part far above the deck (deck at z 0?)")
        L.append(f"| {sub} | {zmin:.1f} | {'; '.join(roots)} | {verdict} |")
    return L + [""]


def owner(part, parts_info):
    return parts_info.get(part, {}).get("subsystem", "?")


def write_md(rep, subs, args, git_rev, secs):
    pinfo = {r["name"]: r for r in rep.get("static", [])}
    L = []
    L.append("# Whole-base integration audit (cad-frame)")
    L.append("")
    L.append(f"Run {time.strftime('%Y-%m-%d %H:%M')} Sydney on `{git_rev}` (base-v1 merged into w/cad-frame), "
             f"`tools.audit --dt {args.dt} --jobs {args.jobs}{'' if args.refine else ' --no-refine'}`, {rep.get('n_parts')} parts, {rep.get('n_states')} states, "
             f"{secs:.0f} s. Verdict: **{'CLEAN' if rep.get('clean') else 'NOT CLEAN'}**.")
    L.append("")
    L.append("Subsystems: " + ", ".join(n + ("" if e is None else f" ({e})") for n, e in subs))
    L.append("")
    try:
        L += origin_check()
    except Exception as e:
        L += [f"origin check failed: {e}", ""]
    if rep.get("errors"):
        L.append("## Assembly errors")
        L += [f"- {e}" for e in rep["errors"]]
        L.append("")
    cols = sorted(rep.get("collisions", []), key=lambda c: -c["max_vol_mm3"])
    L.append(f"## Collisions ({len(cols)})")
    L.append("")
    if cols:
        L.append("| part A (owner) | part B (owner) | worst overlap mm3 | states hit | worst state: phase, t | note |")
        L.append("|---|---|---|---|---|---|")
        for c in cols:
            L.append(f"| {c['a']} ({owner(c['a'], pinfo)}) | {c['b']} ({owner(c['b'], pinfo)}) | {c['max_vol_mm3']} | "
                     f"{c['n_states']} | {c.get('phase')}, {c.get('t')} | {c.get('note', '')} |")
    else:
        L.append("None.")
    L.append("")
    cl = [c for c in rep.get("clearances", []) if not c.get("ok")]
    L.append(f"## Failed clearances ({len(cl)})")
    L += [f"- {c.get('a')} / {c.get('b')}: gap {c.get('gap')} < {c.get('min_mm')} ({c.get('why', '')})" for c in cl] or ["None."]
    L.append("")
    fl = rep.get("floating", [])
    L.append(f"## Floating or ungrounded parts ({len(fl)})")
    L += [f"- {f.get('part')} ({owner(f.get('part'), pinfo)}): {f.get('why', '')}" for f in fl] or ["None."]
    L.append("")
    sf = rep.get("static_fails", [])
    L.append(f"## Static fails ({len(sf)})")
    L += [f"- {f['part']} ({owner(f['part'], pinfo)}): {'; '.join(f['fails'])}" for f in sf] or ["None."]
    L.append("")
    ck = rep.get("checks", [])
    L.append(f"## Subsystem check failures ({len(ck)})")
    L += [f"- {c.get('subsystem')}: {c.get('fail')}" for c in ck] or ["None."]
    L.append("")
    lim = rep.get("limits", [])
    if lim:
        L.append(f"## Joint limits ({len(lim)})")
        L += [f"- {x}" for x in lim]
        L.append("")
    L.append("Method: exact manifold intersection of every part pair over the mission timeline plus each subsystem's "
             "audit_states; pairs that never change relative pose are checked once. Allowed contacts are not listed.")
    with open(OUT_MD, "w") as fh:
        fh.write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dt", type=float, default=1.0)
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="/tmp/claude-0/integ")
    ap.add_argument("--refine", action="store_true", help="let the audit refine the timeline (slow: 10k+ states)")
    a = ap.parse_args()
    subs = subsystems()
    names = [n for n, e in subs if e is None]
    t = time.time()
    r = subprocess.run([sys.executable, "-W", "ignore", "-m", "tools.audit", a.out, "--only", ",".join(names),
                        "--dt", str(a.dt), "--jobs", str(a.jobs)] + ([] if a.refine else ["--no-refine"]),
                       cwd=BASE, capture_output=True, text=True)
    secs = time.time() - t
    path = os.path.join(a.out, "audit.json")
    if not os.path.exists(path):
        with open(OUT_MD, "w") as fh:
            fh.write("# Whole-base integration audit (cad-frame)\n\nThe audit did not run:\n\n```\n" + r.stdout[-3000:] +
                     r.stderr[-3000:] + "\n```\n")
        print(r.stdout[-2000:], r.stderr[-2000:])
        return 1
    rep = json.load(open(path))
    rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=BASE, capture_output=True, text=True).stdout.strip()
    write_md(rep, subs, a, rev, secs)
    print(r.stdout[-1500:])
    print("wrote", OUT_MD)
    return 0


if __name__ == "__main__":
    sys.exit(main())
