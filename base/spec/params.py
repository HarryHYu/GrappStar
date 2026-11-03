"""GrappStar base station: every shared number (single source of truth).

Units: mm, deg, N, N m, s, kg unless a name says otherwise.
Owner: the coordinator. Workers read it and never edit it; request changes in your REPORT.md or NOTES.md.
STATUS: v1.2 (3 Nov 2026, 12:30 Sydney). Binding with base/spec/BASE_SPEC.md and base/spec/DECISIONS.md.
The architecture blocks below hold the INTERFACE numbers (where two owners must agree) and the module placements.
Each owner keeps its internal geometry in its own module and maps these numbers explicitly (read with defaults).
Code against the names; never hard-code a shared number anywhere else.
"""

import math

VERSION = "1.2.2"

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
    name="Century C-243 (5/8 x 6-1/2 in x 0.054 in wire)", qty=2, od=16.0, free_len=165.1, wire=1.37,
    k_n_per_mm=0.125, F0_n=7.0, measured=False,     # rate and F0 by formula; hang test pending
    catalogue=dict(max_safe_load_n=38.4, max_deflection_mm=302.0, note="Century rating, about 100k cycles"),
    max_ext=265.0,                     # D30: force at full cock about 40 N (catalogue 38.4 N, low cycle count); was 280
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
                       note="worm wheel = crank in cad-launch's aim housing (planes about y 186..225), confirmed by cad-launch"),
    ),
    # main axis stack (world y = s from the adapter face). Owners may move a part within its band and must report it.
    stack=[
        dict(y=(0.5, 46.0), part="drum_coupler: coupler hub on the wiper shaft, printed PLA shear pin d4 at r 13 (D8, about 5 N m, 6 spares), outer coupler", owner="cad-drum"),
        dict(y=(40.0, 48.0), part="TAKE-UP BUS gear m1.5 50T face 8 on the outer coupler (D15), downstream of the shear pin, upstream of the dog", owner="cad-drum"),
        dict(y=(50.5, 62.5), part="REEL output gear m2 50T face 12, free on the reel sleeve; dogs to y 67.5", owner="cad-drum"),
        dict(y=(72.0, 98.0), part="dog collar at NEUTRAL (stroke 7 each way); fork groove y 82..88", owner="cad-drum"),
        dict(y=(100.5, 111.0), part="LAUNCH dog ring, bolted to the launch sleeve flange (face at y 111); its joint follows the sleeve", owner="cad-drum"),
        dict(y=(111.0, 193.0), part="LAUNCH sleeve: flange, rail cheek journals, drive gear 26T m2.5 on sprag #1 (y 142.5..158.5), splitter sun (y 174..182)", owner="cad-launch"),
        dict(y=(187.0, 193.0), part="sleeve outboard bushing with the thrust face, in cad-launch's AIM HOUSING (stands on the deck; no frame seat on the sleeve)", owner="cad-launch"),
        dict(y=(96.0, 200.0), part="main shaft d12 spigot through the launch sleeve bore (12.35)", owner="cad-drum"),
        dict(y=(195.0, 203.0), part="AS5600 magnet cap on the spigot end (edge-on disc or 6 x 2.5 diametric)", owner="cad-drum"),
        dict(y=(204.0, 210.0), part="AS5600 board bracket on the aim housing outer side (chip 1 to 2 mm from the magnet)", owner="cad-launch"),
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
    order="LAUNCH / NEUTRAL / WRAP / (park WRAPPED in N_TO_R) / REEL (D16, D22)",
    # v1.0 zone table with cad-drum's forced deviations (drum_NOTES v4 section 0); drum deg, forward = toward REEL
    zones=[
        dict(name="LAUNCH", deg=(-4.0, 8.0), collar="LAUNCH", clutch="LIGHT", cup="locked home"),
        dict(name="L_TO_N", deg=(8.0, 32.0), collar="ramp", clutch="LIGHT (clutch ramps to FIRM over 30..40, after the collar shift)", cup="locked"),
        dict(name="NEUTRAL", deg=(32.0, 50.0), collar="NEUTRAL", clutch="FIRM from 40", cup="locked"),
        dict(name="WRAP_PREP", deg=(50.0, 62.0), collar="NEUTRAL", clutch="FIRM to LIGHT", cup="lock releases at 60"),
        dict(name="WRAP", deg=(62.0, 302.0), collar="NEUTRAL", clutch="LIGHT", cup="7 turns (4 pinion turns x 42/24), ends at home"),
        dict(name="N_TO_R", deg=(302.0, 340.0), collar="NEUTRAL until 318, then ramp to REEL", clutch="LIGHT to FIRM 302..312", cup="locked home"),
        dict(name="REEL", deg=(340.0, 352.0), collar="REEL", clutch="FIRM", cup="locked"),
    ],
    targets=dict(LAUNCH=0.0, NEUTRAL=41.0, WRAP_PARK_START=56.0, WRAP_WINDOW=(62.0, 302.0), WRAPPED=314.0, REEL=347.0),
    # park angles avoid code edges: 56 is mid WRAP_PREP; 314 is in N_TO_R with the clutch FIRM, the cup locked and the
    # collar still NEUTRAL (ramp starts 318): a 30 percent rate error lands 310.4..317.6
    end_stops=(-4.0, 352.0),   # 356 deg of travel; the 4 deg unreachable gap keeps the code ramps from aliasing
    code_contacts=("SW_DRUM_A", "SW_DRUM_B", "SW_DRUM_C"),
    codes=dict(LAUNCH="100", L_TO_N="110", NEUTRAL="010", WRAP_PREP="011", WRAP="001", N_TO_R="101", REEL="111"),
    # bits A B C, 1 = contact closed; Gray order; 000 = broken common wire (fault DRUM_SENSOR)
    dog_contacts=("SW_DOG_L", "SW_DOG_R"),   # collar engaged LAUNCH / REEL; checked during every move: the wrap window is
                                            # never entered with either closed
    unload_jog_deg=8.0,        # before any shift out of LAUNCH or REEL
    fault_rules=dict(from_REEL_or_N_TO_R="go to WRAPPED (314), never NEUTRAL (that would run back through the wrap window and unwrap under tow)",
                     boot_in_WRAP="stop and report", REEL_jog="forward only (sprag #2)"),
    wrap_turns=7,
    wrap_train=dict(segment_pinion_turns=4, idler_out=dict(module=1.5, teeth=42, face=8.0), cup_gear=dict(module=1.5, teeth=24),
                    centre_idler_cup=49.6, plane_y=(68.5, 76.5), idler_axis_xz=(16.24, -104.88)),
    n20=dict(axis_xz=(99.9, -78.5), note="motor pointing up; cad-drum sizes the ratio with at least 2x margin (FIRM 5 N, D24)"),
    magnet_face_y=201.5,        # D23: magnet cap y 195..201.5 on the spigot end (spigot to y 198.5)
    index_cam_bumps=6,          # D12
    sw_index_mount="2 x M3 in the wiper adapter plate front face at (x 43.1, z +6) and (x 43.1, z -36)",
    torque_limiter_nm=5.0,      # D8 / D26
    bus=dict(module=1.5, teeth=50, face=8.0, plane_y=(40.0, 48.0), meshes="cad-reel c1 46T m1.5 at 72.1",
             overdrive_total=2.5, note="D15/D25: no drag brake (the swing idler stays engaged under payout)"),
    reel_output=dict(module=2.0, teeth=50, face=12.0, plane_y=(50.5, 62.5)),
)

LAUNCH = dict(
    owner="cad-launch",
    drive_gear=dict(module=2.5, teeth=26, face=16.0, y=(142.5, 158.5), note="on sprag #1 (inside), coaxial on the launch sleeve"),
    big_gear=dict(module=2.5, teeth=40, present=27, centre_from_pivot=82.6, layers=dict(sector=(134.0, 146.0), full=(146.0, 154.0)),
                  note="two layers: a sector layer meshes the rack (last tooth relieved, the one before 30 percent taller), a full "
                       "40T layer meshes the drive gear so every cock can start (D23)"),
    wiper_turns_per_shot=40.0 / 26.0,
    stroke=241.2,
    spring_preload=23.8,        # D30: extension at HOME = SPRING.max_ext 265 - stroke 241.2
    spring_ext_full=265.0,
    spring_preload_fallback=8.8,   # anchor hole set for SPRING.max_ext_fallback 250
    second_pair_provision=True, # D5
    spring_guard=True,          # D33: printed guard over both springs (hooks near their rating)
    stop=dict(travel_min=19.0, peak_force_max=300.0,
              note="D6/D26: about 2.8 to 3.3 J per shot; a linear printed PLA flexure needs about 19 mm; TPU pads only if Harry gets TPU"),
    carriage_mass_target_kg=0.060,
    sleeve=dict(od_at_sprag=20.0, key="integral rib 6 wide", wall_min=3.8, spigot_bore=12.35, y=(111.0, 193.0), note="D13, D23"),
    rail_walls_y=(112.0, 168.0), springs_y=(94.0, 186.0), cheeks_y=((123.0, 131.0), (159.5, 167.5)),
    flight=dict(range_m=4.8, apex_m=1.1, pitch_deg=36.0, t_flight_s=1.1, note="for the sim; physics v1.2: 12.1 J, 7.41 m/s, 4.8 m at 0.5 N"),
    never_fire_without_tether=True,   # D11
)

AIM = dict(
    owner="cad-launch",
    pitch_min=6.0, pitch_max=47.0,
    wiper_deg_per_aim_cycle=3600.0,           # 10 backward wiper turns per full crank cycle
    rising_wiper_deg=1600.0,
    # full cycle: (backward wiper deg from the LOW dead point, rail pitch deg); rising to 1600, falling back to 3600
    table=[(0, 6.01), (100, 6.24), (200, 6.93), (300, 8.12), (400, 9.83), (500, 12.10), (600, 14.93), (700, 18.32),
           (800, 22.22), (900, 26.50), (1000, 30.98), (1100, 35.39), (1200, 39.42), (1300, 42.76), (1400, 45.19),
           (1500, 46.62), (1600, 47.06), (1700, 46.59), (1800, 45.37), (1900, 43.52), (2000, 41.20), (2100, 38.54),
           (2200, 35.64), (2300, 32.59), (2400, 29.49), (2500, 26.39), (2600, 23.36), (2700, 20.46), (2800, 17.72),
           (2900, 15.19), (3000, 12.91), (3100, 10.91), (3200, 9.21), (3300, 7.85), (3400, 6.85), (3500, 6.23),
           (3600, 6.01)],
    solver="use the whole table range 6..47; min range about 2.9 m (at 6 deg); prefer the higher pitch when two solutions exist",
    holds_by="printed self-locking worm (single start, lead about 5 deg)",
    one_way="swing-arm splitter (pawl-free), engages on BACKWARD wiper turns",
    aim_only_at_home=True,
)

REEL = dict(
    owner="cad-reel",
    capstan=dict(d=40.0, gear=dict(module=2.0, teeth=22, face=12.0), backstop="sprag #2 to the frame (wind-in only)"),
    capstan_ratio=50.0 / 22.0,                # capstan turns per main shaft turn
    wraps=7,
    clutch=dict(LIGHT_line_n=0.5, FIRM_line_n=5.0, LIGHT_nmm=15.0, FIRM_nmm=153.0,
                convention="line tension with the storage reel FULL (r 30.7 mm); about 23 percent higher at the core",
                note="D7, D24; set by cad-drum's brake track through cad-reel's pushrod (fail-safe LIGHT)"),
    storage=dict(core_d=50.0, flange_d=72.0, width=22.0, bearings="2 x 608", shroud=True, two_piece=True,
                 axis_R=(-88.0, 0.0), note="D25: flange 72 for 20 m with 4.6 mm freeboard"),
    take_up=dict(c1=dict(module=1.5, teeth=46, face=6.0), swing_idler=dict(module=1.5, teeth=30),
                 reel_gear=dict(module=1.5, teeth=20), disk_ratio=2.5, drag_brake=False,
                 note="D25: compound c1 free on the capstan tube meshes the bus 50T; the swing idler rides c1's hub, engages on forward turns"),
    tow_limit_n=50.0,                         # firmware default, adjustable 2..110
    tow_tension_model=dict(n_per_a=17.0, offset_n=-10.4, note="line N = 17.0 x (I - I_free) - 10.4 (physics v1.2, D32/D33)"),
    current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, blank_ms=50, hold_ms=30, note="D8/D33"),
    line_speed_mps=dict(free=0.21, at_50n=0.14),
    pulleys="693ZZ printed pulleys where the line turns more than about 20 deg; one on the rail at the pivot (D14)",
    cup_gear=dict(module=1.5, teeth=24, note="driven by cad-drum's wrap idler output 42T m1.5 (4 pinion turns = 7 cup turns)"),
    line_path_hook="reel.line_path(P, S) starts the tether polyline at the storage reel; launch.line_path continues to the dart",
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
    # fw-base owns the final table (firmware/base/grappstar_base/pins.h); these match it.
    SW_DRUM_A=dict(owner="cad-drum", closes="drum code bit A", gpio=10),
    SW_DRUM_B=dict(owner="cad-drum", closes="drum code bit B", gpio=11),
    SW_DRUM_C=dict(owner="cad-drum", closes="drum code bit C", gpio=12),
    SW_DOG_L=dict(owner="cad-drum", closes="collar engaged with the LAUNCH dog ring", gpio=13),
    SW_DOG_R=dict(owner="cad-drum", closes="collar engaged with the REEL gear", gpio=14),
    SW_INDEX=dict(owner="cad-drum", closes="6-bump index cam on the main shaft (no-magnet fallback, D12)", gpio=35),
    SW_N20=dict(owner="cad-drum", closes="N20 engaged (open = hand lever latched out)", gpio=47),
    SW_HOME=dict(owner="cad-launch", closes="carriage on its front stop (rail lever pressed by a foot under the spring arm)", gpio=21),
    SW_GAP=dict(owner="cad-launch", closes="big gear in its toothless gap (cam arc on the big gear, switch on the right cheek)", gpio=39),
    SW_RAIL_LOW=dict(owner="cad-launch", closes="rail at minimum pitch (cam on the aim crank)", gpio=40),
    SW_SEATED=dict(owner="cad-launch", closes="dart tail against the pusher (rail lever via a carriage plunger, no wires on the carriage)", gpio=41),
    SW_ARM=dict(owner="cad-reel", closes="wrap cup at home (cam bump on the cup gear, closed +-12 deg)", gpio=42),
    AS5600=dict(owner="cad-drum (magnet cap) + cad-launch (bracket)", i2c=dict(sda=8, scl=9)),
    BTS7960=dict(rpwm=4, lpwm=5, r_en=6, l_en=7, is_adc=1, note="10 k pull-downs on R_EN and L_EN"),
    DRV8833=dict(ain1=15, ain2=16, slp="NOT WIRED (D9)"),
    HX711=dict(dt=17, sck=18, optional=True),
    VBAT=dict(adc=2, divider="100 k / 22 k"),
    free_gpio_note="GPIO38 drives the RGB LED on DevKitC-1 v1.1: not used for contacts",
)

# Mission timeline for the audit, the sim and the firmware tests: (name, seconds, text, targets eased to at the end).
# S keys: wiper_deg (multi-turn, + = forward), drum_deg, aim_deg, cock (0..1), fired (0/1), wraps (0..7),
# line_out_m, dart_flight (0 on the sled .. 1 landed), dart_return (0 at the landing point .. 1 seated again).
TIMELINE = [
    ("load", 2.0, "Dart seated on the sled, drum in LAUNCH, rail low, line taut.",
     dict(wiper_deg=0.0, drum_deg=0.0, aim_deg=6.0, cock=0.0, fired=0, wraps=0, line_out_m=0.0, dart_flight=0.0, dart_return=0.0, anchored=0)),
    ("aim", 4.0, "Wiper backward: the crank-rocker lifts the rail to 36 deg; the worm holds it.",
     dict(wiper_deg=-1120.0, aim_deg=36.0)),
    ("cock", 3.0, "Wiper forward: the big gear draws the sled back 241 mm; the take-up winds the slack.",
     dict(wiper_deg=-1120.0 + 470.0, cock=1.0)),
    ("fire", 0.5, "The relieved tooth lets go: the sled throws the dart; the big gear resets into its gap.",
     dict(wiper_deg=-1120.0 + 553.8, cock=0.0, fired=1, dart_flight=0.05)),
    ("flight", 1.5, "Dart in flight; the line pays off the storage reel against the 0.5 N clutch.",
     dict(dart_flight=1.0, line_out_m=5.2)),
    ("neutral", 4.0, "Unload jog, then the drum goes to NEUTRAL: wiper coupled to nothing.", dict(drum_deg=41.0)),
    ("wrap", 30.0, "Right after landing, while the line still lies slack: the drum turns the wrap cup 7 times round the capstan.",
     dict(drum_deg=302.0, wraps=7, line_out_m=4.3)),
    ("wrapped", 2.0, "Drum to the WRAPPED park (314): cup locked, clutch FIRM, collar still NEUTRAL.", dict(drum_deg=314.0)),
    ("anchor", 4.0, "The dart plants its foot and screws its two drills into the ground.", dict(anchored=1)),
    ("tidy", 3.0, "Wiper forward: the take-up pulls the slack through the free capstan until the line is straight.",
     dict(wiper_deg=-566.2 + 400.0, line_out_m=4.1)),
    ("reel_engage", 2.0, "Drum into REEL while the wiper creeps forward: the collar drops into the REEL dogs.",
     dict(drum_deg=347.0, wiper_deg=-166.2 + 60.0)),
    ("tow", 5.0, "Capstan winches the rover toward the anchored dart; sprag #2 holds the load.",
     dict(wiper_deg=-106.2 + 3600.0, line_out_m=1.6)),
    ("release", 2.0, "The dart unscrews its drills and lets go.", dict(anchored=0)),
    ("reel_in", 3.0, "Capstan reels the dart in until about 1.5 m remain.",
     dict(wiper_deg=3493.8 + 600.0, line_out_m=1.3, dart_return=0.3)),
    ("unload", 1.0, "Wiper jogs 8 deg back inside the dog window; the collar is free.", dict(wiper_deg=4093.8 - 8.0)),
    ("unwrap", 30.0, "Drum back to WRAP_END and through the wrap sector, collar in NEUTRAL; the take-up winds the freed line.",
     dict(drum_deg=62.0, wraps=0, wiper_deg=4085.8 + 1800.0)),
    ("approach", 4.0, "Drum to NEUTRAL; the FIRM take-up pulls the dart tail first onto the sled until SW_SEATED.",
     dict(drum_deg=41.0, line_out_m=0.0, dart_return=1.0, wiper_deg=5885.8 + 1440.0)),
    ("to_launch", 4.0, "Drum to LAUNCH: clutch LIGHT, collar into the LAUNCH dogs.",
     dict(drum_deg=0.0, dart_flight=0.0, dart_return=0.0)),
    ("lower", 4.0, "Wiper backward through the rest of the aim cycle: the rail comes down to its low stop.",
     dict(aim_deg=6.0, wiper_deg=7325.8 - 2480.0)),
]
