"""KEDGE v34 mass model for dart-physics: whole-dart mass, CG and inertia (folded) from OUT/mass_props.json (dart-cad's per-part
masses, centroids and unit-density inertias), with or without the belly putty and with the bought (catalogue, not weighed) masses
scaled; plus the righting-sim groups (hull / deploy pods / leg).

  python3 mass34.py [OUT] [RES]     -> RES/cg34.json   (defaults out34, phys34)

Cases: nominal (as mass_props.json), no putty, and for each of them every bought part x1.1 or x0.9 (uniform), and the adversarial
splits that move the CG furthest: bought parts above the CG x1.1 and below x0.9 ("z up"), the reverse ("z down"), and the same
fore / aft ("x aft", "x fwd"). The putty, printed parts and the EXTRA estimates (wiring 12 g, glue 3 g, bridle 0.5 g) are not scaled.
"""
import json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load(out):
    J = json.load(open(os.path.join(out, "mass_props.json")))
    PP = {p["name"]: p for p in json.load(open(os.path.join(out, "parts.json")))["parts"]}

    def root(n):
        while PP[n].get("parent"):
            n = PP[n]["parent"]
        return n
    rows = []
    for n, p in J["parts"].items():
        g = p.get("printed_g", p.get("bought_g", 0.0))
        f = p["folded"]
        I = np.array(f["inertia_unit_density_mm5"]) * (g / p["volume_mm3"]) if p["volume_mm3"] > 0 else np.zeros((3, 3))
        rows.append(dict(name=n, g=g, c=np.array(f["centroid_mm"], float), I=I, grp=PP[root(n)].get("joint") or "hull",
                         kind="printed" if p["printable"] else ("putty" if n.startswith("ballast") else "bought")))
    for nm, g, c in J["extras"]:
        rows.append(dict(name=nm, g=g, c=np.array(c, float), I=np.zeros((3, 3)), grp="hull", kind="extra"))
    return rows, J


def case(rows, putty=True, scale=None):
    """Mass (g), CG (mm), inertia about the CG (g mm2, CAD axes: x along the hull, y left, z up) and the sim groups."""
    rs = [r for r in rows if putty or r["kind"] != "putty"]
    m = np.array([r["g"] * (scale(r) if (scale and r["kind"] == "bought") else 1.0) for r in rs])
    C = np.array([r["c"] for r in rs])
    M = m.sum(); cg = (m[:, None] * C).sum(0) / M
    I = np.zeros((3, 3))
    for r, mi in zip(rs, m):
        d = r["c"] - cg
        I += (r["I"] * (mi / r["g"]) if r["g"] else 0) + mi * (d @ d * np.eye(3) - np.outer(d, d))
    grp = {}
    for k in ("hull", "deploy", "leg"):
        sel = np.array([r["grp"] == k for r in rs])
        mk = m[sel].sum()
        grp[k] = dict(mass_g=round(float(mk), 2), cg_mm=[round(float(x), 2) for x in (m[sel, None] * C[sel]).sum(0) / mk])
    return dict(mass_g=float(M), cg_mm=cg, I_gmm2=I, groups=grp)


def cases(rows):
    nom = case(rows)
    cz, cx = nom["cg_mm"][2], nom["cg_mm"][0]
    S = {"bought x1.0": None, "bought x1.1": lambda r: 1.1, "bought x0.9": lambda r: 0.9,
         "bought z up": lambda r: 1.1 if r["c"][2] > cz else 0.9, "bought z down": lambda r: 0.9 if r["c"][2] > cz else 1.1,
         "bought x aft": lambda r: 1.1 if r["c"][0] > cx else 0.9, "bought x fwd": lambda r: 0.9 if r["c"][0] > cx else 1.1}
    out = {}
    for putty in (True, False):
        for k, f in S.items():
            out[("putty" if putty else "no putty") + ", " + k] = case(rows, putty, f)
    return out


def sim_params(c, nom, base=dict(hull_mass=0.191, cgX=83.4, cgUp=-3.3, pod_mass=0.026, leg_mass=0.0055)):
    """Righting-sim overrides for a case: the v34 check's values shifted by this case's change of each group."""
    h, h0 = c["groups"]["hull"], nom["groups"]["hull"]
    return dict(hull_mass=round(base["hull_mass"] + (h["mass_g"] - h0["mass_g"]) / 1000, 5),
                cgX=round(base["cgX"] + h["cg_mm"][0] - h0["cg_mm"][0], 2), cgUp=round(base["cgUp"] + h["cg_mm"][2] - h0["cg_mm"][2], 2),
                pod_mass=round(base["pod_mass"] + (c["groups"]["deploy"]["mass_g"] - nom["groups"]["deploy"]["mass_g"]) / 2000, 5),
                leg_mass=round(base["leg_mass"] + (c["groups"]["leg"]["mass_g"] - nom["groups"]["leg"]["mass_g"]) / 1000, 5))


def main(out, res):
    rows, J = load(out)
    C = cases(rows)
    nom = C["putty, bought x1.0"]
    R = {}
    for k, c in C.items():
        R[k] = dict(mass_g=round(c["mass_g"], 1), cg_x=round(float(c["cg_mm"][0]), 2), cg_z=round(float(c["cg_mm"][2]), 2),
                    Iyy_pitch_kgm2=float("%.4g" % (c["I_gmm2"][1, 1] * 1e-9)), Izz_yaw_kgm2=float("%.4g" % (c["I_gmm2"][2, 2] * 1e-9)),
                    Ixx_roll_kgm2=float("%.4g" % (c["I_gmm2"][0, 0] * 1e-9)), groups=c["groups"], sim=sim_params(c, nom))
    zs = [v["cg_z"] for v in R.values()]; xs = [v["cg_x"] for v in R.values()]
    R["range"] = dict(cg_z_mm=[min(zs), max(zs)], cg_x_mm=[min(xs), max(xs)],
                      check=dict(mass_props_totals=J["totals"]["folded"], this_model_nominal=[R["putty, bought x1.0"]["mass_g"],
                                 R["putty, bought x1.0"]["cg_x"], R["putty, bought x1.0"]["cg_z"]]))
    os.makedirs(res, exist_ok=True)
    json.dump(R, open(os.path.join(res, "cg34.json"), "w"), indent=1)
    return R


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out34")
    res = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "phys34")
    R = main(out, res)
    for k, v in R.items():
        if k != "range":
            print(f"{k:28s} {v['mass_g']:6.1f} g  CG x {v['cg_x']:6.2f} z {v['cg_z']:+5.2f}  Iyy {v['Iyy_pitch_kgm2']:.3e}  sim {v['sim']}")
    print(R["range"])
