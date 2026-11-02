"""Demo subsystem: two meshing spur gears on keyed shafts in 608 bearings on a small plate.
An example of the module contract (base/cad/README.md) and the test case of the whole pipeline.
Not part of the base: the tools skip it unless asked (--only demo)."""
import math

from cad.lib import Part, Joint, Allow, Clearance, prim, tf

M = 2.0            # module
Z1, Z2 = 20, 30
FACE = 10.0
PLATE_T = 8.0
BRG_W = 7.0


def _g():
    try:
        from cad.lib import gears
        return gears
    except ImportError:
        return None


def centre_distance():
    g = _g()
    if g is not None and hasattr(g, "center_distance"):
        return g.center_distance(M, Z1, Z2)
    return M * (Z1 + Z2) / 2 + 0.1


def _gear(z):
    g = _g()
    if g is not None:
        return g.spur_gear(M, z, FACE, bore=12.0, bore_kind="key")
    r = M * z / 2 - M * 1.25                     # placeholder disc at the root circle
    return prim.cyl(r, FACE) - prim.cyl(6.15, FACE + 2, z=-1)


def _plate(cd):
    pl = prim.rbox(cd + 60, 60, PLATE_T, 6)
    for x in (0.0, cd):
        pl = pl - prim.cyl(11.15, BRG_W + 1, x=x - cd / 2, z=PLATE_T - BRG_W)     # 608 seat (lib seat later)
        pl = pl - prim.cyl(7.0, 20, x=x - cd / 2, z=-5)
    return pl.translate((cd / 2, 0, 0))


def _bearing():
    return prim.tube(11.0, 4.0, BRG_W)


def _shaft():
    s = prim.cyl(4.0, BRG_W, z=-BRG_W)                       # 8 mm journal in the 608
    s = s + prim.cyl(5.5, 2.0)                               # shoulder on the inner race
    s = s + prim.cyl(6.0, FACE + 4, z=2.0)                   # 12 mm gear seat
    return s


def parts(P):
    cd = centre_distance()
    out = [Part("demo_plate", _plate(cd), "PLA", joint=None, place=tf.I4, print_tf=tf.I4, color="#9aa7b4",
                notes="Base plate with two 608 seats")]
    for i, (x, z) in enumerate(((0.0, Z1), (cd, Z2)), 1):
        out.append(Part(f"demo_brg_{i}", _bearing(), "bought", joint=None, place=tf.T(x, 0, PLATE_T - BRG_W),
                        color="#8a8f96", bought_ref="608ZZ 8x22x7", notes="608 bearing"))
        out.append(Part(f"demo_shaft_{i}", _shaft(), "PPA-CF", joint=f"demo_g{i}", place=tf.T(0, 0, PLATE_T),
                        print_tf=tf.T(0, 0, BRG_W), color="#3a3f45", notes="Keyed shaft"))
        out.append(Part(f"demo_gear_{i}", _gear(z), "PPA-CF", joint=f"demo_g{i}", place=tf.T(0, 0, PLATE_T + 4.0),
                        print_tf=tf.I4, color="#d97a00" if i == 1 else "#3d7cc9", notes=f"Spur gear m{M:g} z{z}"))
    return out


def joints(P):
    cd = centre_distance()
    off = 180.0 / Z2 if Z2 % 2 == 0 else 0.0
    return [Joint("demo_g1", None, "rev", (0, 0, 0), (0, 0, 1)),
            Joint("demo_g2", None, "rev", (cd, 0, 0), (0, 0, 1), gear=("demo_g1", -Z1 / Z2, off))]


def kin(P, S):
    return {"demo_g1": S.get("wiper_deg", 0.0) * 0.1}


def allowed(P):
    return [Allow("demo_brg_*", "demo_plate", "608 press fit in the seat"),
            Allow("demo_shaft_1", "demo_brg_1", "journal in the inner race"),
            Allow("demo_shaft_2", "demo_brg_2", "journal in the inner race"),
            Allow("demo_shaft_1", "demo_gear_1", "keyed fit"),
            Allow("demo_shaft_2", "demo_gear_2", "keyed fit")]


def clearances(P):
    return [Clearance("demo_gear_*", "demo_plate", 1.0, "gears clear the plate"),
            Clearance("demo_gear_1", "demo_gear_2", 0.0, "gear mesh without interference")]


def audit_states(P):
    return [dict(phase="demo_sweep", t=0.0, wiper_deg=k * 360.0 / Z1 / 8 * 10) for k in range(9)]


def grounded(P):
    return ["demo_plate"]


def steps(P):
    return [dict(title="Press in the bearings", text="Press one 608 into each seat of the plate, flush with the top.",
                 parts=["demo_plate", "demo_brg_1", "demo_brg_2"]),
            dict(title="Fit the shafts and gears", text="Slide each gear onto its shaft key, then drop the shafts into the bearings.",
                 parts=["demo_shaft_1", "demo_gear_1", "demo_shaft_2", "demo_gear_2"])]
