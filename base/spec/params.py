"""GrappStar base station: every shared number (single source of truth).

Units: mm, deg, N, N m, s, kg unless a name says otherwise.
Owner: the coordinator. Workers read it and never edit it; request changes in your REPORT.md.
STATUS: STUB. Fixed bought-part data below is real. Architecture values are None until BASE_SPEC.md lands
(about 12:00 Sydney, 3 Nov 2026). Code against the names; do not hard-code numbers anywhere else.
"""

VERSION = "0.1-stub"

# ------------------------------------------------------------------------------------------------ frame
# World frame (decided in BASE_SPEC.md): right-handed, Z up, +X = launch azimuth (forward), +Y = left looking forward.
FRAME = dict(origin=None, x="forward (launch azimuth)", y="left", z="up")

# ------------------------------------------------------------------------------------------------ printer
BED = (256.0, 256.0, 260.0)            # Bambu X2D single-nozzle build volume
MATERIALS = dict(
    PLA=dict(density=1.24, note="default"),
    PPA_CF=dict(density=1.25, note="loaded gears, keyed shafts"),
)

# ------------------------------------------------------------------------------------------------ bought parts Harry has
WIPER = dict(                          # 12 V car wiper motor, self-locking worm. NOT MEASURED YET: keep interfaces parametric
    measured=False,
    n0_rpm=45.0, stall_nm=15.0, stall_a=18.0, free_a=1.5,
    shaft_d=10.0, shaft_flat=8.0, shaft_thread="M8", shaft_len=20.0,
    mount_holes=None,                  # list of (x, y, dia) relative to the output shaft axis, in the mount face plane
    body_box=None,                     # gearbox housing envelope
    wires=dict(red="+", black="-", green="park switch (tape off)"),
)
SPRAG = dict(name="CSK20PP", bore=20.0, od=47.0, w=14.0, key=6.0, torque_nm=50.0, rpm_max=6000, qty=2,
             roles={1: "spring cock and fire", 2: "capstan drive"})
B608 = dict(bore=8.0, od=22.0, w=7.0, qty_buying=10)
F693ZZ = dict(bore=3.0, od=8.0, w=4.0, flange_d=9.5, flange_w=0.6)
SPRING = dict(                         # 2 x Century Spring Corp extension spring, Bunnings I/N 0487961
    name="Century 16 x 165.1 x 1.4 mm extension", qty=2, od=16.0, free_len=165.1, wire=1.4,
    k_n_per_mm=0.125, F0_n=7.0, max_ext=250.0, measured=False,   # rate and F0 by formula; hang test pending
)
LINE = dict(name="Dyneema braid 150 lb", d=1.0, length_m=20.0, break_n=667.0)
N20_WORM = dict(                       # right-angle self-locking worm N20, 12 V, biaxial output (caliper data)
    body=(12.0, 10.0), box_w=9.94, box_h=11.93, len_no_shaft=33.04, shaft_tip_to_tip=29.97, rpm=27.0,
    stall_a_8v=0.18, free_a_8v=0.025,
)
N20_STRAIGHT = dict(                   # Bringsmart, M3 threaded shaft
    body=(12.0, 10.0), gearbox_h=11.95, face_to_back=24.27, shaft_d=2.84, shaft_len=56.03, rpm=100.0,
)
DART = dict(                           # KEDGE v30 (kedge/out30, kedge/CLAUDE.md section 5)
    hull_d=42.0, hull_len=152.0,
    folded_bbox=dict(x=(0.0, 166.7), y=(-40.3, 40.3), z=(-21.0, 75.8)),   # dart frame: nose tip x = 0, belly z = -21
    belly="plain cylinder d 42 along x: the launcher cradle surface; legs stow on top and flanks",
    mass_kg=0.279,                     # 264 g printed + bought, plus 15 g nose ballast
    tether="tail cap, two d 4.2 holes", flies="nose first",
)
BOARDS = dict(                         # base electronics (sizes to confirm in the frame worker's model)
    esp32s3_devkitc1=dict(note="ESP32-S3-DevKitC-1-N8"),
    bts7960=dict(note="BTS7960 dual half-bridge module"),
    as5600=dict(note="Adafruit AS5600 STEMMA QT (ADA6357)"),
    drv8833=dict(size=(20.31, 12.70, 7.19), note="Pololu 2130, VM max 10.8 V"),
    mp1584=dict(size=(22.26, 16.96, 3.98), note="buck, needed to feed the DRV8833 from the 3S"),
)
BATTERY = dict(cells=3, v_full=12.6, v_nom=11.1, mah=5200, fuse_a=20)

# ------------------------------------------------------------------------------------------------ architecture (BASE_SPEC.md)
LAYOUT = None        # every shaft: name -> dict(origin=(x, y, z), axis=(x, y, z))
DRUM = None          # selector drum: positions, angles, tracks, N20 drive, sensing
LAUNCH = None        # rail, carriage, rack, big gear, springs, stroke, release
AIM = None           # aim train: one-way element, self-locking element, crank-rocker, pitch range
REEL = None          # storage reel, slip clutch, capstan, wrap arm, line path
FRAMEWORK = None     # frame, deck, motor adapter, electronics bay, rover mount
SWITCHES = None      # printed contacts: name -> where, what closes it, GPIO
TIMELINE = None      # mission sequence for the audit, sim and firmware
