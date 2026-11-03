"""GrappStar base station: every shared number (single source of truth).

Units: mm, deg, N, N m, s, kg unless a name says otherwise.
Owner: the coordinator. Workers read it and never edit it; request changes in your REPORT.md or NOTES.md.
STATUS: v1.0 (3 Nov 2026, 12:05 Sydney). Binding with base/spec/BASE_SPEC.md and base/spec/DECISIONS.md.
The architecture blocks below hold the INTERFACE numbers (where two owners must agree) and the module placements.
Each owner keeps its internal geometry in its own module and maps these numbers explicitly (read with defaults).
Code against the names; never hard-code a shared number anywhere else.
"""

import math

VERSION = "1.0"

# ------------------------------------------------------------------------------------------------ frame
# World frame: right-handed, Z up, +X = launch azimuth (forward), +Y = left looking forward. Origin on the main axis
# (wiper output = main shaft = rail pivot), y = 0 at the wiper motor adapter plate face (the wiper boss face).
FRAME = dict(origin="main axis at the wiper adapter plate face", x="forward (launch azimuth)", y="left", z="up",
             main_axis="world Y through the origin")

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
    # motor frame: origin on the output shaft axis in the boss face plane, +Z along the shaft into the base (world +Y),
    # +X along the gearbox towards the can. Typical car wiper, NOT MEASURED (cad-frame defaults):
    mount_holes=[(-24.0, 38.0, 6.5), (-24.0, -38.0, 6.5), (72.0, 0.0, 6.5)],
    boss_d=16.0, boss_h=8.0,
    body_box=(-38.0, 84.0, -40.0, 40.0, 56.0),     # gearbox x0, x1, y0, y1, depth behind the boss face
    can_d=58.0, can_len=95.0, can_x0=84.0,
    orient_deg=90.0,   # LAYOUT: the motor frame +X points to world +Z (gearbox and can ABOVE the main axis), so the
                       # body stays clear of the wrap cup below it (cup closest point 44 mm from the axis)
    wires=dict(red="+", black="-", green="park switch (never wire it: it may switch +12 V)"),
)
SPRAG = dict(name="CSK20PP", bore=20.0, od=47.0, w=14.0, key=6.0, torque_nm=50.0, rpm_max=6000, qty=2,
             roles={1: "spring cock and fire", 2: "capstan drive"})
B608 = dict(bore=8.0, od=22.0, w=7.0, qty_buying=10)
F693ZZ = dict(bore=3.0, od=8.0, w=4.0, flange_d=9.5, flange_w=0.6)
SPRING = dict(                         # 2 x Century Spring Corp extension spring, Bunnings I/N 0487961
    name="Century 16 x 165.1 x 1.4 mm extension", qty=2, od=16.0, free_len=165.1, wire=1.4,
    k_n_per_mm=0.125, F0_n=7.0, measured=False,     # rate and F0 by formula; hang test pending
    max_ext=280.0,                     # D5: physics safe extension about 290 mm (hook torsion, SF 1.2)
    max_ext_fallback=250.0,            # alternate anchor hole set if the hang test shows a weaker spring
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
# Angle convention in the x-z plane: deg from +X toward +Z (0 = forward, 90 = up, 270 = straight down).
def _polar(r, deg):
    return (round(r * math.cos(math.radians(deg)), 3), round(r * math.sin(math.radians(deg)), 3))

_CAP_R, _CAP_DEG = 72.1, 255.0              # capstan axis: REEL gear m2 50T + capstan gear m2 22T, +0.1 fit
_CAP_X, _CAP_Z = _polar(_CAP_R, _CAP_DEG)   # (-18.661, -69.643)

LAYOUT = dict(
    deck_top_z=-165.0,                      # D2: main axis 165 above the deck top
    footprint=dict(x=(-160.0, 255.0), y=(-95.0, 220.0), note="target; cad-frame reports the real one"),
    # every rotating axis: owner, origin (world), axis direction, axial range (world y unless stated), notes
    axes=dict(
        main=dict(owner="cad-drum", origin=(0.0, 0.0, 0.0), axis=(0, 1, 0), y=(0.5, 180.0),
                  note="wiper output = main shaft = rail pivot; the launch sleeve (cad-launch) is coaxial on its d12 spigot"),
        drum=dict(owner="cad-drum", origin=(60.0, 0.0, -112.0), axis=(0, 1, 0), y=(1.0, 142.0),
                  note="front-below: clear of the big gear sweep (lowest z about -44) and of the reel gear zone (x <= 8)"),
        capstan=dict(owner="cad-reel", origin=(_CAP_X, 0.0, _CAP_Z), axis=(0, 1, 0),
                     note="72.1 from the main axis at 255 deg; capstan and cup extend toward -y (cad-reel frame R)"),
        aim_crank=dict(owner="cad-launch", origin=(58.0, 195.0, -100.5), axis=(0, 1, 0),
                       note="worm wheel = crank, plane about 55 outboard of the rail plane; cad-launch confirms"),
    ),
    # main axis stack (world y = s from the adapter face). Owners may move a part within its band and must report it.
    stack=[
        dict(y=(0.5, 46.0), part="drum_coupler incl. the D8 torque limiter (about 5 N m)", owner="cad-drum"),
        dict(y=(40.0, 50.0), part="TAKE-UP BUS gear m1.5 (D15), downstream of the limiter, upstream of the dog", owner="cad-drum"),
        dict(y=(50.5, 62.5), part="REEL output gear m2 50T face 12, free on the reel sleeve; dogs to y 67.5", owner="cad-drum"),
        dict(y=(72.0, 98.0), part="dog collar at NEUTRAL (stroke 7 each way); fork groove y 82..88", owner="cad-drum"),
        dict(y=(100.5, 114.0), part="LAUNCH dog ring, keyed to the launch sleeve flange (face at y 111)", owner="cad-drum"),
        dict(y=(111.0, 172.0), part="LAUNCH sleeve: flange, rail cheek journals, drive gear 26T m2.5 on sprag #1, splitter sun", owner="cad-launch"),
        dict(y=(160.0, 172.0), part="frame outboard bearing seat on the sleeve far end (thrust face for LAUNCH engage)", owner="cad-frame"),
        dict(y=(172.0, 180.0), part="AS5600 magnet cap on the d12 spigot end (edge-on disc or 6 x 2.5 diametric)", owner="cad-drum"),
        dict(y=(181.0, 186.0), part="AS5600 board bracket (chip 1 to 2 mm from the magnet)", owner="cad-frame"),
    ],
    rail_plane_y=140.0,      # drive gear and big gear mid plane; rail body spans about y 95 to 185, above w 35 near the pivot
    reel_frame_R=dict(       # cad-reel's frame R placed in the world (rigid transform)
        origin=(_CAP_X, 33.5, _CAP_Z),      # capstan axis on the FRONT face of the reel wall
        x_axis=(1, 0, 0), y_axis=(0, 0, 1), z_axis=(0, -1, 0),   # R +X = world +X, R +Y = world +Z, R +Z = world -Y
        note="capstan gear plane R Z -29..-17 = world y 50.5..62.5 (meets the REEL gear); cup toward world -y",
    ),
    keep_out=dict(
        rail_sweep="rail body and carriage between 6 (or the retrieve minimum) and 47 deg pitch, for y 95..185: "
                   "behind and below the axis down to z about -125 at x about -150..-210, and the big gear down to z -45 in front",
        wiper_body="gearbox and can above the axis (WIPER.orient_deg), y -56..0, plus the can along +z",
        line="the line from the exit eye over the 693ZZ pulleys to the rail pivot pulley and along the rail (D14)",
    ),
)

DRUM = dict(
    owner="cad-drum",
    order="LAUNCH / NEUTRAL / WRAP / REEL (D16)",
    # zone table, drum deg (forward = toward REEL). cad-drum owns the exact edges and must keep every dwell >= 12 deg.
    zones=[
        dict(name="LAUNCH", deg=(-5.0, 8.0), collar="LAUNCH", clutch="LIGHT", cup="locked home"),
        dict(name="L_TO_N", deg=(8.0, 32.0), collar="ramp", clutch="LIGHT to FIRM", cup="locked"),
        dict(name="NEUTRAL", deg=(32.0, 50.0), collar="NEUTRAL", clutch="FIRM", cup="locked"),
        dict(name="WRAP_PREP", deg=(50.0, 62.0), collar="NEUTRAL", clutch="FIRM to LIGHT", cup="lock releases at 60"),
        dict(name="WRAP", deg=(62.0, 302.0), collar="NEUTRAL", clutch="LIGHT", cup="7 turns, ends at home"),
        dict(name="N_TO_R", deg=(302.0, 340.0), collar="ramp 318..340", clutch="LIGHT to FIRM 302..316", cup="locked home"),
        dict(name="REEL", deg=(340.0, 355.0), collar="REEL", clutch="FIRM", cup="locked"),
    ],
    targets=dict(LAUNCH=0.0, NEUTRAL=41.0, WRAP_START=62.0, WRAP_END=302.0, REEL=347.0),
    end_stops=(-8.0, 358.0),
    wrap_turns=7,               # D4 / D16: exactly 7 cup turns across 62..302, ring back at home
    cup_turns_per_drum_deg=7.0 / 240.0,
    n20_ratio_min_margin=2.0,   # N20 to drum ratio: at least 2x margin on the peak drum load (cad-drum sizes it)
    code_contacts=("SW_DRUM_A", "SW_DRUM_B", "SW_DRUM_C"),   # Gray code per zone; 000 = broken wire fault
    codes=dict(LAUNCH="100", L_TO_N="110", NEUTRAL="010", WRAP_PREP="011", WRAP="001", N_TO_R="101", REEL="111"),
    # (bits A B C, 1 = contact closed); adjacent zones differ by one bit; cad-drum may re-assign but must keep Gray order
    dog_contacts=("SW_DOG_L", "SW_DOG_R"),                    # collar really engaged left (LAUNCH) / right (REEL)
    magnet_face_y=178.5,        # cad-frame reads it for the AS5600 bracket
    index_cam_bumps=6,          # D12: SW_INDEX fallback encoder on the main shaft
    torque_limiter_nm=5.0,      # D8
    bus=dict(module=1.5, plane_y=(40.0, 48.0), overdrive_total=2.5, drag_brake_nm=0.04,
             note="D15: bus gear on the main shaft (cad-drum), swing idler + compound on the capstan tube + reel gear + clutch (cad-reel)"),
    reel_output=dict(module=2.0, teeth=50, face=12.0, plane_y=(50.5, 62.5)),
)

LAUNCH = dict(
    owner="cad-launch",
    drive_gear=dict(module=2.5, teeth=26, face=12.0, note="on sprag #1 (inside), coaxial on the launch sleeve"),
    big_gear=dict(module=2.5, teeth=40, present=33, face=12.0, centre_from_pivot=82.6,
                  note="on the rail pin; last tooth relieved, the one before 30 percent taller (v10)"),
    wiper_turns_per_shot=40.0 / 26.0,
    stroke=245.0,
    spring_preload=35.0,        # D5: extension at HOME
    spring_ext_full=280.0,      # D5
    spring_preload_fallback=5.0,
    second_pair_provision=True, # D5: anchor holes and clearance for 2 more of the same springs, not fitted
    stop=dict(travel_min=15.0, peak_force_max=300.0, note="D6 compliant printed stop"),
    carriage_mass_target_kg=0.060,
    sleeve=dict(od_at_sprag=20.0, key="integral rib 6 wide", wall_min=4.0, spigot_bore=12.35, note="D13"),
    flight=dict(range_m=4.5, apex_m=1.0, pitch_deg=35.0, t_flight_s=1.0, note="for the sim; physics owns the real table"),
    never_fire_without_tether=True,   # D11
)

AIM = dict(
    owner="cad-launch",
    pitch_min=6.0, pitch_max=47.0,            # cad-launch may extend the minimum down to about -10 for retrieval (D_perform)
    launch_pitch_range=(10.0, 45.0),
    wiper_deg_per_aim_cycle=3600.0,           # 10 wiper turns per full crank cycle (cad-launch table)
    rising_wiper_deg=1596.0,                  # backward wiper deg from LOW to the top
    table=[(0, 6.01), (200, 6.93), (400, 9.83), (600, 14.93), (800, 22.22), (1000, 30.98), (1100, 35.39),
           (1200, 39.42), (1400, 45.19), (1596, 47.06)],   # (backward wiper deg from LOW, pitch deg)
    holds_by="printed self-locking worm (single start, lead about 5 deg)",
    one_way="swing-arm splitter (pawl-free), engages on BACKWARD wiper turns",
    aim_only_at_home=True,
)

REEL = dict(
    owner="cad-reel",
    capstan=dict(d=40.0, gear=dict(module=2.0, teeth=22, face=12.0), backstop="sprag #2 to the frame (wind-in only)"),
    capstan_ratio=50.0 / 22.0,                # capstan turns per main shaft turn
    wraps=7,
    clutch=dict(LIGHT_line_n=0.5, FIRM_line_n=3.0, note="D7; set by cad-drum's brake track through cad-reel's pushrod (fail-safe LIGHT)"),
    storage=dict(core_d=50.0, flange_d=66.0, width=22.0, bearings="2 x 608", shroud=True),
    tow_limit_n=50.0,                         # firmware default, adjustable 2..110
    line_speed_mps=dict(free=0.21, at_50n=0.18),
    pulleys="693ZZ printed pulleys where the line turns more than about 20 deg; one on the rail at the pivot (D14)",
    cup_gear=dict(module=1.5, teeth=24, note="driven by cad-drum's wrap idler output 48T m1.5 (7 turns per wrap)"),
)

FRAMEWORK = dict(
    owner="cad-frame",
    deck_z=-165.0,                            # cad-frame's name for the deck top
    motor_side=-1,                            # the wiper sits on the -y side of the adapter plate
    ebay=dict(x=(110.0, 240.0), y=(-70.0, 60.0), note="on the deck in front of the drum; USB to the Jetson reachable"),
    camera=dict(model="Razer Kiyo X", pos=(150.0, -50.0, 100.0), pitch_deg=-10.0, look="+X",
                note="D18: bore-sighted to the launch azimuth; cad-frame sets the final pose"),
    rc832=dict(note="holder with the antenna up and clear of metal (D18)"),
    rover_mount="flat deck with an M3 hole grid; the deck is its own drill template",
)

SWITCHES = dict(
    # printed contacts: an M3 screw head touching an M3 nut, wired to a GPIO with the internal pull-up, closed = LOW.
    # GPIO suggestions avoid strapping pins 0, 3, 45, 46, USB 19, 20 and flash/PSRAM 26..37. fw-base owns the final table.
    SW_DRUM_A=dict(owner="cad-drum", closes="drum code bit A", gpio=10),
    SW_DRUM_B=dict(owner="cad-drum", closes="drum code bit B", gpio=11),
    SW_DRUM_C=dict(owner="cad-drum", closes="drum code bit C", gpio=12),
    SW_DOG_L=dict(owner="cad-drum", closes="collar engaged with the LAUNCH dog ring", gpio=13),
    SW_DOG_R=dict(owner="cad-drum", closes="collar engaged with the REEL gear", gpio=14),
    SW_INDEX=dict(owner="cad-drum", closes="6-bump index cam on the main shaft (no-magnet fallback, D12)", gpio=21),
    SW_N20=dict(owner="cad-drum", closes="N20 engaged (open = hand lever latched out)", gpio=47),
    SW_HOME=dict(owner="cad-launch", closes="carriage on its front stop (uncocked)", gpio=38),
    SW_GAP=dict(owner="cad-launch", closes="big gear in its toothless gap (rest)", gpio=39),
    SW_RAIL_LOW=dict(owner="cad-launch", closes="rail at minimum pitch (crank low dead point)", gpio=40),
    SW_SEATED=dict(owner="cad-launch", closes="dart tail against the pusher", gpio=41),
    SW_ARM=dict(owner="cad-reel", closes="wrap cup ring at home (one pulse per cup turn)", gpio=42),
    AS5600=dict(owner="cad-drum + cad-frame", i2c=dict(sda=8, scl=9)),
    BTS7960=dict(rpwm=4, lpwm=5, r_en=6, l_en=7, is_adc=1, note="10 k pull-downs on R_EN and L_EN"),
    DRV8833=dict(ain1=15, ain2=16, slp="NOT WIRED (D9)"),
    HX711=dict(dt=17, sck=18, optional=True),
    VBAT=dict(adc=2, divider="100 k / 22 k"),
)

# Mission timeline for the audit, the sim and the firmware tests: (name, seconds, text, targets eased to at the end).
# S keys: wiper_deg (multi-turn, + = forward), drum_deg, aim_deg, cock (0..1), fired (0/1), wraps (0..7),
# line_out_m, dart_flight (0 on the sled .. 1 landed), dart_return (0 at the landing point .. 1 seated again).
TIMELINE = [
    ("load", 2.0, "Dart seated on the sled, drum in LAUNCH, rail low, line taut.",
     dict(wiper_deg=0.0, drum_deg=0.0, aim_deg=6.0, cock=0.0, fired=0, wraps=0, line_out_m=0.0, dart_flight=0.0, dart_return=0.0)),
    ("aim", 4.0, "Wiper backward: the crank-rocker lifts the rail to 35 deg; the worm holds it.",
     dict(wiper_deg=-1100.0, aim_deg=35.4)),
    ("cock", 3.0, "Wiper forward: the big gear draws the sled back 245 mm; the take-up winds the slack.",
     dict(wiper_deg=-1100.0 + 470.0, cock=1.0)),
    ("fire", 0.5, "The relieved tooth lets go: the sled throws the dart; the big gear resets into its gap.",
     dict(wiper_deg=-1100.0 + 553.8, cock=0.0, fired=1, dart_flight=0.05)),
    ("flight", 1.5, "Dart in flight; the line pays off the storage reel against the 0.5 N clutch.",
     dict(dart_flight=1.0, line_out_m=4.6)),
    ("neutral", 3.5, "Drum to NEUTRAL: wiper coupled to nothing, clutch FIRM.", dict(drum_deg=41.0)),
    ("tidy", 2.0, "Wiper forward: the take-up winds the slack until the line is straight to the anchored dart.",
     dict(wiper_deg=-546.2 + 360.0, line_out_m=4.4)),
    ("wrap", 6.0, "Drum through the wrap sector: the cup lays 7 turns on the capstan.",
     dict(drum_deg=302.0, wraps=7)),
    ("reel_engage", 2.0, "Drum into REEL while the wiper creeps forward: the collar drops into the REEL dogs.",
     dict(drum_deg=347.0, wiper_deg=-186.2 + 60.0)),
    ("tow", 5.0, "Capstan winches the rover toward the anchored dart; sprag #2 holds the load.",
     dict(wiper_deg=-126.2 + 3600.0, line_out_m=1.5)),
    ("release", 2.0, "The dart unscrews its drills and lets go.", dict()),
    ("reel_in", 3.0, "Capstan reels the dart in until about 1.5 m remain.",
     dict(wiper_deg=3473.8 + 1200.0, line_out_m=1.2, dart_return=0.3)),
    ("unload", 1.0, "Wiper jogs 30 deg back inside the dog window; the collar is free.", dict(wiper_deg=4673.8 - 30.0)),
    ("unwrap", 6.0, "Drum back through the wrap sector, collar in NEUTRAL; the take-up winds the freed line.",
     dict(drum_deg=62.0, wraps=0, wiper_deg=4643.8 + 1800.0)),
    ("approach", 4.0, "Drum to NEUTRAL; the take-up pulls the dart tail first onto the sled until SW_SEATED.",
     dict(drum_deg=41.0, line_out_m=0.0, dart_return=1.0, wiper_deg=6443.8 + 1440.0)),
    ("to_launch", 3.5, "Drum to LAUNCH: clutch LIGHT, collar into the LAUNCH dogs.",
     dict(drum_deg=0.0, dart_flight=0.0, dart_return=0.0)),
    ("lower", 4.0, "Wiper backward through the rest of the aim cycle: the rail comes down to its low stop.",
     dict(aim_deg=6.0, wiper_deg=7883.8 - 2500.0)),
]
