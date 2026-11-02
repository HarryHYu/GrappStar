"""Demo subsystem: two meshing spur gears on keyed shafts in 608 bearings on a small plate.
An example of the module contract (base/cad/README.md) and the test case of the whole pipeline.
Not part of the base: the tools skip it unless asked (--only demo)."""
import math

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits, gears, bearings, shafts

M = 2.0            # module
Z1, Z2 = 20, 30
FACE = 10.0
PLATE_T = 8.0
BRG_W = 7.0


SHAFT_D = 12.0
KEY_W = 4.0        # DIN 6885 key for a 12 mm shaft
KEY_HUB = 1.8
GEAR_Z0 = 2.0      # gear bottom above the plate top


def centre_distance():
    return gears.center_distance(M, Z1, Z2)


def _gear(z):
    return gears.spur_gear(M, z, FACE, bore=SHAFT_D + fits.SHAFT_IN_BORE, bore_kind="key",
                           bore_opts=dict(key_w=KEY_W, key_depth=KEY_HUB))


def _plate(cd):
    pl = prim.rbox(cd + 60, 60, PLATE_T, 6).translate((cd / 2, 0, 0))
    for x in (0.0, cd):
        pl = pl - prim.place(bearings.seat_608(depth=BRG_W), tf.T(x, 0, PLATE_T))
    return pl


def _shaft():
    """Body d12 from z = 0 (sits on the 608 inner race) up through the gear, 8 mm journal below."""
    return shafts.keyed_shaft(GEAR_Z0 + FACE + 2.0, d=SHAFT_D, key_w=KEY_W, key_h=KEY_HUB - 0.2,
                              key_z=(GEAR_Z0, GEAR_Z0 + FACE + 1.5), end_lo=(8.0, BRG_W), chamfer=0.6)


def parts(P):
    cd = centre_distance()
    out = [Part("demo_plate", _plate(cd), "PLA", joint=None, place=tf.I4, print_tf=tf.I4, color="#9aa7b4",
                notes="Base plate with two 608 seats")]
    for i, (x, z) in enumerate(((0.0, Z1), (cd, Z2)), 1):
        out.append(Part(f"demo_brg_{i}", bearings.bearing_608(), "bought", joint=None, place=tf.T(x, 0, PLATE_T),
                        color="#8a8f96", bought_ref="608ZZ 8x22x7", notes="608 bearing"))
        out.append(Part(f"demo_shaft_{i}", _shaft(), "PPA-CF", joint=f"demo_g{i}", place=tf.T(0, 0, PLATE_T),
                        print_tf=tf.T(0, 0, BRG_W), color="#3a3f45",
                        notes="Keyed shaft, 8 mm journal in the 608"))
        out.append(Part(f"demo_gear_{i}", _gear(z), "PPA-CF", joint=f"demo_g{i}", place=tf.T(0, 0, PLATE_T + GEAR_Z0),
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
