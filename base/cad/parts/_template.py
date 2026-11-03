"""Starter for a subsystem module. Copy this file to cad/parts/<name>.py (for example reel.py) and edit.

The assembly skips this file: subsystem modules are the names without an underscore (base/cad/README.md section 3),
so _template.py and helpers like reel_kin.py are never loaded as subsystems.

The example: one PLA bearing block standing on the deck with a 608 seat on the main shaft axis (Y), a 608 bearing
pressed into it, and a small arm turning in the bearing. One example of every function the contract knows.

World frame (binding, base/spec/BASE_SPEC.md section 2): right-handed, mm. Origin ON the main shaft axis
(x = 0, z = 0 on the axis), y = 0 at the motor adapter plate face. +X forward (launch azimuth), +Y left looking
forward, +Z up. The main shaft axis is the Y axis. The deck top is at z = -(axis height).

Naming: every part and joint name is globally unique snake_case and starts with your subsystem name and an
underscore ("reel_spool", "reel_capstan_brg_1"). Change PREFIX below to your module name when you copy this file.

Check just this subsystem (from base/):
    python -m tools.audit out/<name> --only <name>       # prints CLEAN or NOT CLEAN, writes out/<name>/audit.json
    bash run_all.sh <name>                               # the whole chain (audit, render, sim, pack, guide)
"""
from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits, bearings, shafts  # noqa: F401

PREFIX = "tmpl"            # = your module name (reel, drum, launch, frame)


def _p(P, name, default):
    """Read a number from params with a default. The spec is still moving: a name may be missing or None."""
    v = getattr(P, name, default)
    return default if v is None else v


def _n(s):
    """Part / joint name with the subsystem prefix."""
    return f"{PREFIX}_{s}"


# ---------------------------------------------------------------------------------------------- numbers
# Read every shared number from P with a default (P is base/spec/params.py, see tools/params.py). Local numbers that
# nobody else needs can stay here as constants.
BLOCK_W = 40.0             # block width along X
BLOCK_T = 10.0             # block thickness along Y (7 seat + 3 lip)
BLOCK_TOP = 16.0           # block top above the axis
LEVER_L = 30.0


def _dims(P):
    axis_h = _p(P, "TMPL_AXIS_H", 30.0)      # replace with the axis height from P.LAYOUT once v1.0 has it
    y_face = _p(P, "TMPL_Y", 40.0)           # +Y face of the block (the seat mouth) on the main shaft
    return axis_h, y_face


def _seat_tf(y_face):
    """Library cutters and bearing models use axis +Z with the mouth at z = 0. Rx(-90) turns local +Z into world +Y,
    so the seat axis lies on the main shaft (Y) with the mouth on the block's +Y face."""
    return tf.T(0, y_face, 0) @ tf.Rx(-90)


# ---------------------------------------------------------------------------------------------- contract
def parts(P):
    """Every part of the subsystem. Solids are manifold3d.Manifold in the part's local frame; place maps local to
    the joint frame (joint None = world). print_tf maps local to the print bed: flat face down at z = 0."""
    axis_h, y_face = _dims(P)
    # fixed part: modelled directly in world coordinates, so place is the identity
    block = prim.box(-BLOCK_W / 2, BLOCK_W / 2, y_face - BLOCK_T, y_face, -axis_h, BLOCK_TOP)
    block = block - prim.place(bearings.seat_608(depth=bearings.B608_W), _seat_tf(y_face))
    # print on the -Y face, seat mouth up: Rx(90) turns world +Y into +Z, then drop the -Y face to z = 0
    block_print = tf.T(0, 0, -(y_face - BLOCK_T)) @ tf.Rx(90)

    # moving part: modelled in its joint frame (origin on the axis, see joints()), y as in the world
    r_j = shafts.JOURNAL_608_D / 2
    arm = prim.union([
        prim.cylY(r_j, y_face - bearings.B608_W, y_face),                       # journal in the inner race
        prim.cylY(5.5, y_face, y_face + 1.0),                                    # collar on the inner race face
        prim.cylY(7.0, y_face + 1.0, y_face + 5.0),                              # hub, 1 mm off the block face
        prim.box(0, LEVER_L, y_face + 1.0, y_face + 5.0, -4.0, 4.0),             # lever along +X at rest
    ])
    # print on the lever's outer (+Y) face, journal up: Rx(-90) turns world +Y into -Z
    arm_print = tf.T(0, 0, y_face + 5.0) @ tf.Rx(-90)

    return [
        Part(_n("block"), block, "PLA", joint=None, place=tf.I4, print_tf=block_print, color="#9aa7b4",
             notes="Bearing block on the deck, 608 seat on the main shaft axis"),
        Part(_n("brg"), bearings.bearing_608(), "bought", joint=None, place=_seat_tf(y_face), color="#8a8f96",
             bought_ref="608ZZ 8x22x7", notes="608 bearing pressed into the block"),
        Part(_n("arm"), arm, "PLA", joint=_n("arm"), place=tf.I4, print_tf=arm_print, color="#d97a00",
             notes="Example arm turning in the 608"),
    ]


def joints(P):
    """Joints: name, parent (None = world, or any joint of any subsystem), kind "rev" / "slide" / "fixed",
    origin in the parent frame, axis, limits (checked by the audit), gear=(other, ratio, offset) for followers."""
    return [Joint(_n("arm"), None, "rev", origin=(0, 0, 0), axis=(0, 1, 0), limits=(-90.0, 90.0))]


def kin(P, S):
    """Joint values for the master state S (base/cad/README.md section 4). Accept any S: always S.get(key, default).
    Real subsystems map wiper_deg / drum_deg / line_out_m here; the template uses its own key <prefix>_deg."""
    return {_n("arm"): S.get(f"{PREFIX}_deg", 0.0)}


def allowed(P):
    """Overlaps that are meant to be there (press fits, journals). Wildcards allowed. Every Allow should match a real
    overlap, else the audit warns."""
    return [Allow(_n("brg"), _n("block"), "608 press fit in the crush ribs"),
            Allow(_n("arm"), _n("brg"), "journal in the inner race")]


def clearances(P):
    """Pairs that must keep a gap over every audited state."""
    return [Clearance(_n("arm"), _n("block"), 0.5, "arm clears the block face")]


def audit_states(P):
    """Extra states S for the audit: sweep your joints through their range (the mission timeline may not)."""
    return [{"phase": f"{PREFIX}_sweep", f"{PREFIX}_deg": float(a)} for a in range(-90, 91, 15)]


def grounded(P):
    """Parts (fnmatch patterns) that rest on the rover; every other part must connect to one through contacts."""
    return [_n("block")]


def steps(P):
    """Assembly steps for the assembly page and the build guide."""
    return [dict(title="Press in the bearing", text="Press the 608 into the block seat until it sits on the lip.",
                 parts=[_n("block"), _n("brg")]),
            dict(title="Fit the arm", text="Push the arm journal into the bearing from the open face.",
                 parts=[_n("arm")])]


# Optional, add when you need it:
# def checks(P, asm) -> list[str]:   # own extra checks, return failure strings (empty list = pass)
