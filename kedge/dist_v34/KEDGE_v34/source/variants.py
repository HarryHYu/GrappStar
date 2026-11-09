"""Run design variants through generate -> solve angles -> sweep -> physics; write a comparison table."""
import json, os, subprocess, sys, time
V = json.loads(open(sys.argv[1]).read())
rows = []
for name, ov in V.items():
    out = "var_" + name; t0 = time.time()
    env = dict(os.environ, KOV=json.dumps(ov), KV=name)
    r = subprocess.run(["timeout", "1500", "python3", "kedge_v27.py", out], env=env, capture_output=True, text=True)
    if r.returncode: rows.append(dict(name=name, error=r.stdout[-400:] + r.stderr[-400:])); print(name, "GEN FAIL", r.stderr[-300:]); continue
    warn = [l for l in r.stdout.splitlines() if "WARN" in l or "wt=False" in l]
    subprocess.run(["python3", "solve_deg.py", out], capture_output=True)
    subprocess.run(["timeout", "1800", "python3", "audit27.py", out, "2", "1"], capture_output=True)
    subprocess.run(["timeout", "900", "python3", "physics.py", out], capture_output=True)
    A = json.load(open(out + "/audit.json")); Ph = json.load(open(out + "/physics.json")); P = json.load(open(out + "/parts.json"))["P"]
    rows.append(dict(name=name, overrides=ov, clean=not A["collisions"] and not A["ground_below"] and not A["belly_outside_r21"] and not warn,
                     collisions=list(A["collisions"].keys())[:6], warn=warn[:3], L=P["L"], kick_deg=P["KICK_DEG"], deploy_deg=P["DEPLOY_DEG"],
                     drill_depth=A["auger_depth_mm"]["drill_leg_L_auger"], foot_depth=A["auger_depth_mm"]["foot_auger"],
                     folded_len=round(A["folded_bbox"][3] - A["folded_bbox"][0], 1), belly=A["belly_outside_r21"],
                     mass_g=Ph["mass_g"], cg_x=Ph["cg_mm"][0], margin_cal=Ph["flight"]["static_margin_calibers_side"],
                     hold_loam_N=Ph["anchoring"]["by_soil"]["moist loam"]["total_N"], hold_sand_N=Ph["anchoring"]["by_soil"]["dry sand"]["total_N"],
                     hold_clay_N=Ph["anchoring"]["by_soil"]["firm clay"]["total_N"], minutes=round((time.time() - t0) / 60, 1)))
    print(json.dumps(rows[-1]))
    json.dump(rows, open("variants_" + os.path.basename(sys.argv[1]), "w"), indent=1)
