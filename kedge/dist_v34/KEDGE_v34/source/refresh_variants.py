import json, glob, subprocess, os
rows = []
for f in sorted(glob.glob("variants_var*.json")): rows += json.load(open(f))
for r in rows:
    d = "var_" + r["name"]
    if "error" in r or not os.path.exists(d + "/audit.json"): continue
    subprocess.run(["timeout", "600", "python3", "physics.py", d], capture_output=True)
    Ph = json.load(open(d + "/physics.json"))
    r.update(mass_g=Ph["mass_g"], cg_x=Ph["cg_mm"][0], margin_cal=Ph["flight"]["static_margin_calibers_side"],
             hold_loam_N=Ph["anchoring"]["by_soil"]["moist loam"]["total_N"], hold_sand_N=Ph["anchoring"]["by_soil"]["dry sand"]["total_N"],
             hold_clay_N=Ph["anchoring"]["by_soil"]["firm clay"]["total_N"])
    print(r["name"], r["drill_depth"], r["hold_loam_N"], flush=True)
json.dump(rows, open("variants_all_refreshed.json", "w"), indent=1)
