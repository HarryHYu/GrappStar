"""GrappStar base station: every shared number (single source of truth).

Units: mm, deg, N, N m, s, kg unless a name says otherwise.
Owner: the coordinator. Workers read it and never edit it; request changes in your REPORT.md or NOTES.md.
STATUS: v1.3.8 (3 Nov 2026, 13:00 Sydney; 1.2.4 adds LAUNCH axis_z 0, sleeve_y0 111, rail_y 140; 1.2.5 adds DRUM n20 train 12:1, 8.1 deg/s; 1.2.6 TIMELINE carries mode (collar) and drum_pos; 1.2.7 D36 line guide, D39 tow cap, TIMELINE dog phase; 1.2.8 D40 aim trip 3 A, SW_ARM open at home, launcher as built; 1.2.9 D41 fly-in, D42 tension offset -3.0, carriage 85.8 g as built; 1.3.0 D43 pulleys + dart-side tow law + cap 60 N, D44 shear pin print, cock 425; 1.3.1 drive gear face 11, carriage 80.5 g; 1.3.2 path gain 1.30, dart law 13.1/-2.3, cap 55 N, drum as built, TIMELINE at real drum speed; 1.3.3 unload jog 6 deg; 1.3.4 ASSEMBLY_STAGES; 1.3.5 TIMELINE rail_low after the flight (D45), no lower at the end; 1.3.6 drum stage bands per cad-drum; 1.3.7 ASSEMBLY_STAGE_OVERRIDES for unstaged steps, launcher band 190-204; 1.3.8 final cross-check: WIPER boss_face_y -10 (D48), DRUM code_edges as built, big gear full layer plane face 11, splitter sun plane 170..182, spring fallback hole set F 256.5, SW_ARM window as built; 1.5.0 D50 reel unit on a deck extension behind the launcher: capstan along X in the rail plane y 140, one 693ZZ pulley at the rail rear, free spool d40/d62 x 20 with LIGHT 0.25 N, REEL_UNIT block). Binding with base/spec/BASE_SPEC.md and base/spec/DECISIONS.md.
The architecture blocks below hold the INTERFACE numbers (where two owners must agree) and the module placements.
Each owner keeps its internal geometry in its own module and maps these numbers explicitly (read with defaults).
Code against the names; never hard-code a shared number anywhere else.
"""

import math

VERSION = "1.5.0"

# ------------------------------------------------------------------------------------------------ frame
# World frame: right-handed, Z up, +X = launch azimuth (forward), +Y = left looking forward. Origin on the main axis
# (wiper output = main shaft = rail pivot), y = 0 at the front face of the wiper motor adapter plate; the wiper boss
# face is at WIPER boss_face_y (-10, the plate's rear face, D48).
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
    boss_face_y=-10.0,                 # D48: world y of the boss face (rear face of the 10 mm motor plate); frame and drum coupler both read it
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
    max_ext_fallback=256.5,            # anchor hole set F, 8.5 mm rearward (cad-launch as built), if the hang test shows a weaker spring
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
    footprint=dict(x=(-160.0, 255.0), y=(-95.0, 220.0), note="target for the main deck; cad-frame reports the real one. D50: a bolt-on extension tile carries the reel unit behind x -164 (REEL_UNIT deck_extension)"),
    # every rotating axis: owner, origin (world), axis direction, axial range (world y unless stated), notes
    axes=dict(
        main=dict(owner="cad-drum", origin=(0.0, 0.0, 0.0), axis=(0, 1, 0), y=(0.5, 201.5),
                  note="wiper output = main shaft = rail pivot; the launch sleeve (cad-launch) is coaxial on its d12 spigot"),
        drum=dict(owner="cad-drum", origin=(60.0, 0.0, -112.0), axis=(0, 1, 0), y=(1.0, 142.0),
                  note="front-below: clear of the big gear sweep (lowest z about -44) and of the reel gear zone (x <= 8)"),
        capstan=dict(owner="cad-reel", origin=(_CAP_X, 0.0, _CAP_Z), axis=(0, 1, 0),
                     note="D50: the old capstan axis, 72.1 from the main axis at 255 deg, now carries the cup gear (meshing "
                          "cad-drum's wrap idler), the bus gear c1 and the cup shaft K (cad-reel frame R); the capstan "
                          "itself is in the reel unit (reel_unit_capstan)"),
        reel_unit_capstan=dict(owner="cad-reel", origin=(-294.6, 140.0, 85.7), axis=(1, 0, 0),
                               note="D50: capstan in the reel unit on the deck extension behind the launcher, axis along +X "
                                    "in the rail plane y 140, cup and axial eyelet in front (REEL_UNIT)"),
        aim_crank=dict(owner="cad-launch", origin=(58.0, 195.0, -100.5), axis=(0, 1, 0),
                       note="worm wheel = crank in cad-launch's aim housing (planes about y 186..225), confirmed by cad-launch"),
    ),
    # main axis stack (world y = s from the adapter face). Owners may move a part within its band and must report it.
    stack=[
        dict(y=(0.5, 46.0), part="drum_coupler: coupler hub on the wiper shaft (the hub reaches back to y -5 inside the puck bore, D48), printed PLA shear pin d4 at r 13 (D8, about 5 N m, 6 spares), outer coupler", owner="cad-drum"),
        dict(y=(40.0, 48.0), part="TAKE-UP BUS gear m1.5 50T face 8 on the outer coupler (D15), downstream of the shear pin, upstream of the dog", owner="cad-drum"),
        dict(y=(50.5, 62.5), part="REEL output gear m2 50T face 12, on a journal on the outer coupler; dogs to y 69.5 (as built)", owner="cad-drum"),
        dict(y=(72.0, 98.0), part="dog collar at NEUTRAL (stroke 7 each way); fork groove y 82..88", owner="cad-drum"),
        dict(y=(100.5, 111.0), part="LAUNCH dog ring, bolted to the launch sleeve flange (face at y 111); its joint follows the sleeve", owner="cad-drum"),
        dict(y=(111.0, 193.0), part="LAUNCH sleeve: flange, rail cheek journals, drive gear 26T m2.5 on sprag #1 (y 142.5..158.5), splitter sun (y 170..182)", owner="cad-launch"),
        dict(y=(187.0, 193.0), part="sleeve outboard bushing with the thrust face, in cad-launch's AIM HOUSING (stands on the deck; no frame seat on the sleeve)", owner="cad-launch"),
        dict(y=(106.0, 198.5), part="main shaft d12 spigot through the launch sleeve bore (12.35), as built", owner="cad-drum"),
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
        line="D50: the line from the spool through the cup eyelet, straight to the one 693ZZ pulley at the rail rear end, then along "
             "the rail to the dart tail, all in the rail plane y 140 (REEL_UNIT)",
        reel_unit="D50: the reel unit sweep is fixed (nothing of it moves with the rail); the rail rear reaches world x -228 "
                  "(pitch 22), the cup eyelet sits at x -240.6",
    ),
)

DRUM = dict(
    owner="cad-drum",
    order="LAUNCH / NEUTRAL / WRAP / (park WRAPPED in N_TO_R) / REEL (D16, D22)",
    code_edges=(6.0, 34.0, 50.0, 62.0, 302.0, 342.0),   # drum deg where the code bumps switch, as built (_drum_kin arcs, drum_table.json); firmware edgeDeg
    # v1.0 zone table with cad-drum's forced deviations (drum_NOTES v4 section 0); drum deg, forward = toward REEL
    zones=[
        dict(name="LAUNCH", deg=(-4.0, 8.0), collar="LAUNCH", clutch="LIGHT", cup="locked home"),
        dict(name="L_TO_N", deg=(8.0, 32.0), collar="ramp", clutch="LIGHT (clutch ramps to FIRM over 30..40, after the collar shift)", cup="locked"),
        dict(name="NEUTRAL", deg=(32.0, 50.0), collar="NEUTRAL", clutch="FIRM from 40", cup="locked"),
        dict(name="WRAP_PREP", deg=(50.0, 62.0), collar="NEUTRAL", clutch="FIRM to LIGHT", cup="lock releases at 62 (as built)"),
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
    unload_jog_deg=6.0,        # before any shift out of LAUNCH or REEL (dog play +-8; stopped early on a 0.8 A current rise)
    fault_rules=dict(from_REEL_or_N_TO_R="the drum HOLDS where it is (D27, MODE err DRUM_HELD), never NEUTRAL (that would run back through the wrap window and unwrap under tow); the operator unwraps when safe",
                     boot_in_WRAP="stop and report", REEL_jog="forward only (sprag #2)"),
    wrap_turns=7,
    wrap_train=dict(segment_pinion_turns=4, idler_out=dict(module=1.5, teeth=42, face=8.0), cup_gear=dict(module=1.5, teeth=24),
                    centre_idler_cup=49.6, plane_y=(68.5, 75.8), idler_face_as_built=7.3, idler_axis_xz=(16.24, -104.88)),
    n20=dict(axis_xz=(99.9, -78.5), note="motor pointing up; cad-drum sizes the ratio with at least 2x margin (FIRM 5 N, D24)",
             pinion=dict(module=0.8, teeth=10), drum_gear=dict(module=0.8, teeth=120), ratio=12.0,
             stall_nm_est=0.22, stall_nm_design=0.20, drum_deg_per_s=8.1, supply_v=9.0,
             peak_drum_load_nm=0.92, stall_nm_min_needed=0.171, note_ratio="cad-drum as built (drum_NOTES sections 3 and 9); physics and firmware read these"),
    magnet_face_y=201.5,        # D23: magnet cap y 195..201.5 on the spigot end (spigot to y 198.5)
    index_cam_bumps=6,          # D12
    sw_index_mount="on drum idler post 2 (block on the +x side, screwed to a strut); index cam at s 33.5..38.5, base r 26 (cad-drum 13:45)",
    torque_limiter_nm=5.0,      # D8 / D26
    shear_pin_print="lying down (pin axis flat on the bed) so the shear plane cuts across layers; a layer line on the shear plane slips at 2.5 to 3.3 N m, the same as the cock and tow torques (D44)",
    bus=dict(module=1.5, teeth=50, face=8.0, plane_y=(40.0, 48.0), meshes="cad-reel c1 46T m1.5 at 72.1",
             overdrive_total=2.5, note="D15/D25: no drag brake (the swing idler stays engaged under payout)"),
    reel_output=dict(module=2.0, teeth=50, face=12.0, plane_y=(50.5, 62.5)),
)

LAUNCH = dict(
    owner="cad-launch",
    axis_z=0.0,          # world z of the main axis = LAYOUT axes main origin z (the world origin is ON the main axis)
    sleeve_y0=111.0,     # world y of the launch sleeve flange face (meets the dog ring 100.5..111, D23)
    rail_y=140.0,        # world y of the rail / dart centre plane (= LAYOUT rail_plane_y)
    line_guide_u=-200.0,  # D36: last line guide BEHIND the rail rear end on the dart axis (rail coords mm: u along the rail, 0 at the pivot)
    line_guide_w=85.0,    # D36: so the pulley-to-tail path only gets longer during the stroke (no slack, no snatch)
    carriage_mass_target_g=70.0,  # D36: printed carriage at most 70 g (physics: 100 g costs about 0.4 m and overloads the stop)
    carriage_mass_g=80.5,         # as built (cad-launch 14:28, printed set); physics uses this over the target
    drive_gear=dict(module=2.5, teeth=26, face=11.0, y=(147.5, 158.5), sprag_centre_y=152.0, note="on sprag #1 (inside, keyed), coaxial on the launch sleeve; face 11 for SF 1.5 at the 5 N m shear pin (physics, cad-launch 14:28)"),
    big_gear=dict(module=2.5, teeth=40, present=27, centre_from_pivot=82.6, as_built="sector layer 27 teeth (rack) + full 40T layer face 11 (drive gear), cad-launch 14:28", layers=dict(sector=(134.0, 146.0), full=(147.5, 158.5)),
                  note="two layers: a sector layer meshes the rack (last tooth relieved, the one before 30 percent taller), a full "
                       "40T layer meshes the drive gear so every cock can start (D23)"),
    wiper_turns_per_shot=40.0 / 26.0,
    stroke=240.9,          # as built (cad-launch 13:30); was 241.2
    spring_preload=23.8,        # D30: extension at HOME = SPRING.max_ext 265 - stroke 241.2
    spring_ext_full=265.0,
    spring_preload_fallback=15.6,  # anchor hole set F for SPRING.max_ext_fallback 256.5 (256.5 - stroke 240.9)
    second_pair_provision=True, # D5
    spring_guard=True,          # D33: printed guard over both springs (hooks near their rating)
    stop=dict(travel_min=19.0, peak_force_max=300.0,
              note="D6/D26: about 2.8 to 3.3 J per shot; a linear printed PLA flexure needs about 19 mm; TPU pads only if Harry gets TPU"),
    carriage_mass_target_kg=0.060,
    sleeve=dict(od_at_sprag=20.0, key="integral rib 6 wide", wall_min=3.8, spigot_bore=12.35, y=(111.0, 193.0), note="D13, D23"),
    rail_walls_y=(112.0, 168.0), springs_y=(94.0, 186.0), cheeks_y=((123.0, 131.0), (159.5, 167.5)),
    flight=dict(range_m=4.7, apex_m=1.2, pitch_deg=34.0, t_flight_s=0.85, note="for the sim; physics on params 1.5.0 (D50): 4.70 m at 34 deg with the fixed LIGHT 0.25 N brake and the D50 payout path (gain 1.32, take-up chain inertia)"),
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
    capstan=dict(d=40.0, gear=dict(module=2.0, teeth=22, face=12.0),
                 backstop="sprag #2 on the capstan tube in the reel unit (wind-in only, D50)",
                 note="D50: the 22T m2 gear sits on countershaft C (frame R), driven by the REEL gear through the 41T idler; "
                      "bevel 20:20 into lane C, rear train 20T / 58T idler / 20T onto the capstan tube: ratio still 50/22"),
    capstan_ratio=50.0 / 22.0,                # capstan turns per main shaft turn (D50 train keeps it)
    wraps=7,
    clutch=dict(LIGHT_line_n=0.25, FIRM_line_n=5.0, LIGHT_nmm=6.75, FIRM_nmm=135.0,
                convention="line tension at the design pack radius r 27.0 mm (about 2 m out) of the d40 / d62 spool; the clutch "
                           "hub drives the spool 1:1 (hub 28T / T2 28T, two 1:1 bevels, 16T / 34T idler / 16T)",
                note="D7, D24, D49, D50: LIGHT is a FIXED 0.25 N anti-overrun drag (payout, cock take-up), FIRM 5 N for tidy and the "
                     "final approach; set by cad-drum's brake track through cad-reel's bar, crank and rocker at the clutch hub "
                     "(front, frame R), fail-safe LIGHT"),
    storage=dict(core_d=40.0, flange_d=62.0, width=20.0, bearings="2 x 608 (spool shaft in the reel unit cheeks)", shroud=False,
                 two_piece=False,
                 note="D50: free spool keyed on the spool shaft in the reel unit, axis along world Y, pack y 130..150 in the rail "
                      "plane; the line leaves the pack top on the low-aim line through the cup eyelet"),
    take_up=dict(c1=dict(module=1.5, teeth=46, face=6.0), swing_idler=dict(module=1.5, teeth=30),
                 reel_gear=dict(module=1.5, teeth=20), disk_ratio=2.5, drag_brake=False,
                 note="D25 / D50: compound c1 free on the stationary A0 sleeve meshes the bus 50T; the swing idler rides c1's hub "
                      "and drives the reel gear 20T (clutch disk 2.5 x wiper) on forward turns; the clutch hub drives the spool "
                      "1:1 through T2, lane T and T3 (REEL_UNIT rear_trains t)"),
    tow_limit_n=50.0,                         # firmware default
    tow_limit_max_n=55.0,                     # D39/D46/D50: tension AT THE DART; path gain 1.18 keeps 0.8 x the pin above 60 N: cap stays 55
    path_gain=1.18,                           # D50: capstan/dart at the 6 deg tow pitch: ring-to-groove bend (plain eye) and the lip roller; the rail-rear pulley is straight at 6 deg
    payout_gain=1.32,                         # D50: spool/dart in flight at 20..47 deg: eyelet exit bend about 72 to 80 deg (plain mouth), rail-rear pulley (693ZZ); 1.06 at 6 deg
    tow_tension_model=dict(n_per_a=14.4, offset_n=-2.5, at="dart", capstan_n_per_a=17.0, capstan_offset_n=-3.0,
                           note="D50: dart-side line N = 14.4 x (I - I_free) - 2.5 (capstan side 17.0 / -3.0 through the unchanged 50/22 ratio, D42, divided by path_gain 1.18); tow_limit_n means the DART side; recalibrate with the luggage scale at the line end"),
    current_trips_a=dict(aim=3.0, backward_launch=3.0, reel_in=5.0, cock=6.5, tow=6.5, blank_ms=50, hold_ms=30, note="D8/D33/D40: every backward wiper run in LAUNCH (aim, decock, backward jog) drives the aim train: 3 A"),
    line_speed_mps=dict(free=0.21, at_50n=0.14),
    pulleys="D50: one 693ZZ printed pulley at the rail rear end (the old top pulley); no other pulley on the line",
    cup_gear=dict(module=1.5, teeth=24, note="driven by cad-drum's wrap idler output 42T m1.5 (4 pinion turns = 7 cup turns); D50: the K riser, bevel and rear train are all 1:1"),
    eyelet=dict(r=26.0, bore_d=2.8, mouth_d=4.0, note="D49/D50: the v7 ring hole turned axial at radius 26, lug in front of the cup face; ring at the cup BOTTOM at home"),
    line_path_hook="reel.line_path(P, S): spool pack top -> cup eyelet -> floor segment to the rail-rear pulley; launch.line_path continues over the pulley to the dart",
)

# ------------------------------------------------------------------------------------------------ D50 reel unit
# World frame. The low-aim line (rail at 6.01 deg, the crank LOW dead point) is the rail-frame line w 84.6 (dart axis,
# top of the rail-rear pulley) extended behind the rail: the capstan exit (axial cup eyelet) sits ON it, so the floor
# segment is straight at the tow pitch. Rail frame: x = u cos p - w sin p, z = u sin p + w cos p.
REEL_UNIT = dict(
    owner="cad-reel (reel unit, lanes, front gears), cad-frame (deck extension tile), cad-launch (rail-rear pulley)",
    low_pitch=6.01, eyelet_u=-233.0, line_w=84.6, line_y=140.0,
    eyelet=(-240.6, 140.0, 59.7),            # world, front mouth of the axial eyelet (cup ring at the bottom, cup home)
    capstan_axis=(140.0, 85.7),              # world (y, z); axis +X; cup in front, drum, P1 with sprag #2, gear planes, P2
    lanes=dict(c=(7.0, 7.0), k=(5.0, 36.0), t=(0.0, -55.24),
               note="world (y, z) of the three shafts along X from the front gears to the reel unit: C (capstan, d12), K "
                    "(cup, d8), T (take-up, d8)"),
    front=dict(c_axis=(-150.0, 7.0), c_idler_teeth=41, k2_axis=(-100.0, 36.0), t2_deg=160.0, riser_plane_y=(23.5, 28.5),
               riser=dict(module=1.5, teeth=(30, 34, 30, 30), note="K riser: cup shaft gear 30T, idlers 34T and 30T, K2 30T (1:1)"),
               note="frame R gears in front of / behind the reel wall: C (22T m2 in the REEL plane, bevel m2 20:20 in front), "
                    "K2 (riser gear and bevel m1.5 18:18), T2 (28T meshing the 28T clutch hub, bevel m1 20:20)"),
    rear_trains=dict(c=dict(module=2.0, teeth=(20, 58, 20), face=8.0), k=dict(module=1.5, teeth=(20, 76, 20), face=5.0),
                     t=dict(bevel=(1.0, 20, 20), spur=(1.5, 16, 34, 16), note="T3 16T, idler 34T, spool shaft 16T: light, the whole take-up chain turns with the spool in payout"),
                     note="C plane in front of the K plane; the I_c axle sits in P1, the I_k axle in P2"),
    spool=dict(axis_xz=(-369.5, 18.51), pack_y=(130.0, 150.0), drive_inertia_gmm2=9000.0,
               note="clutch hub, friction disc, T2, bevels, lane T, T3 and the rear gears turn with the spool in payout (about 9000 g mm2 at spool speed, the spool itself about 19000 full)"),
    deck_extension=dict(x=(-412.0, -164.0), note="bolt-on deck tile behind the existing deck (dovetails, heat-set inserts, M3 to "
                                                 "the rover like the other tiles); the launcher stays where it is"),
    bends=dict(pulley_deg=dict(p6=0.0, p20=92.0, p34=114.0, p47=125.0), eyelet_exit_deg=dict(p6=0.0, p20=72.0, p34=80.0, p47=78.0),
               note="D50 as built: the reel unit is close behind the rail, so the rail-rear pulley moves 150 mm up and down "
                    "with the aim and the floor segment steepens; at the 6 deg tow pitch everything is straight"),
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
    SW_ARM=dict(owner="cad-reel", closes="CLOSED while the wrap cup is away from home; OPENS for about -6 to +8 deg at home (lever drops into a notch on the cup gear track); firmware home = open", gpio=42),
    AS5600=dict(owner="cad-drum (magnet cap) + cad-launch (bracket)", i2c=dict(sda=8, scl=9)),
    BTS7960=dict(rpwm=4, lpwm=5, r_en=6, l_en=7, is_adc=1, note="10 k pull-downs on R_EN and L_EN"),
    DRV8833=dict(ain1=15, ain2=16, slp="NOT WIRED (D9)"),
    HX711=dict(dt=17, sck=18, optional=True),
    VBAT=dict(adc=2, divider="100 k / 22 k"),
    free_gpio_note="GPIO38 drives the RGB LED on DevKitC-1 v1.1: not used for contacts",
)

# Mission timeline for the audit, the sim and the firmware tests: (name, seconds, text, targets eased to at the end).
# S keys: wiper_deg (multi-turn, + = forward: main shaft, coupler, bus gear and collar ALWAYS follow it),
# sleeve_deg (launch sleeve + LAUNCH dog ring: changes only while the collar is in LAUNCH, frozen otherwise),
# reel_deg (REEL output gear: changes only while the collar is in REEL; capstan = reel_deg x 50/22),
# drum_deg, aim_deg, cock (0..1), fired (0/1), wraps (0..7),
# line_out_m, dart_flight (0 on the sled .. 1 landed), dart_return (0 at the landing point .. 1 seated again).
# ------------------------------------------------------------------------------------------------ assembly order
# One global order for the generated assembly steps (tools.steps sorts by step["stage"]). Each owner gives every step
# an int stage inside its band; cross-owner constraints are built into the band order. Test-stage-only steps use 900+.
ASSEMBLY_STAGES = dict(
    deck=(100, 109, "cad-frame", "drill the rover, deck tiles, rover mount, heat-set inserts"),
    motor=(110, 119, "cad-frame", "wiper motor on its adapter (bench), motor-side cheek, motor onto the cheek"),
    drum_core=(130, 139, "cad-drum", "bracket A, axle, drum A"),
    drum_idler=(140, 144, "cad-drum", "wrap idler group (the idler posts drop in before the shift rail and the coupler bus gear block them)"),
    main_shaft=(145, 149, "cad-drum", "shift rail, coupler hub on the wiper shaft, shear pin, outer coupler with the REEL gear and washer, index contact (before the reel bus gear c1 and the dog post)"),
    reel_capstan=(150, 159, "cad-reel", "D50 front gears: reel wall, sleeve, bus gear c1, swing idler, cup shaft, cup gear, riser to K2, countershaft C and REEL idler, clutch hub, brake crank, rocker and bar, reel shaft, T2, SW_ARM"),
    drum_shift=(160, 169, "cad-drum", "fork and dog contacts, drum B (keyed), hand lever, bracket B, N20, code switches"),
    main_stack=(170, 179, "cad-drum", "LAUNCH dog ring, magnet cap, dog post"),
    reel_storage=(180, 189, "cad-reel", "D50 reel unit on the deck extension: lane tabs and posts, P1 with sprag #2, capstan tube, lanes C, K, T, cheeks, T bracket, spool, drum, cup, rear trains, P2, line"),
    launcher=(190, 204, "cad-launch", "rail bench sub-assembly (sleeve first), big gear (timed without springs), drive gear and sprag, carriage, stops, contacts, rail-rear pulley (D50), lip roller"),
    aim=(205, 209, "cad-launch", "aim housing, worm, wheel, crank, splitter, lug, conrod"),
    sensors=(210, 219, "cad-sensors", "SW_RAIL_LOW, SW_GAP, AS5600 board"),
    electronics=(220, 229, "cad-frame", "electronics bay, set the buck before wiring the DRV8833, wiring (battery unplugged), camera, receiver, cable anchors"),
    guards=(230, 239, "cad-frame / cad-launch", "gear guards; spring guards go on with the springs at test stage 5"),
    test_stage_only=(900, 999, "all", "steps done only at a test stage (springs: 999)"),
)

# Stage for authored steps whose owner had not set step["stage"] when the build day ended (the owners stopped at the
# usage limit). tools.steps uses these when present, so the generated order follows ASSEMBLY_STAGES.
ASSEMBLY_STAGE_OVERRIDES = dict(
    frame={"Heat-set inserts": 100, "Drill the rover": 101, "Deck on the rover": 102, "Motor plate": 110,
           "Wiper motor on the puck (bench)": 111, "Puck into the plate": 112, "Motor leg": 113, "Can stay": 114,
           "Shafts and trains (other subsystems)": 119, "Electronics bay": 220,
           "Set the buck BEFORE wiring the DRV8833": 221, "Camera, receiver and cable anchors": 222,
           "Wiring and lid": 223, "Gear guards": 230},
    launch={"Launcher 1: print the parts": 190, "Launcher 2: join the rail": 191,
            "Launcher 2b: splice straps and aim lug": 192, "Launcher 3: big gear": 193,
            "Launcher 4: drive gear, sprag #1 and the launch sleeve": 194, "Launcher 5: carriage": 195,
            "Launcher 5b: lip roller": 196, "Launcher 6: carriage stop frames": 197, "Launcher 6b: spring guards": 198,
            "Launcher 6c: stop springs": 199, "Launcher 7: contacts": 200,
            "Launcher 8: time the big gear (fingers clear)": 201, "Launcher 9: rail-rear pulley": 202,
            "Aim 1: aim housing, worm, wheel, crank": 205, "Aim 2: splitter": 206,
            "Aim 3: lug, conrod, rail onto the base": 207, "Launcher 10: wiring (battery unplugged)": 208},
    reel={"Screw the reel wall to the deck": 150, "Sleeve, bus gear, swing arm and swing idler": 151,
          "Cup shaft, cup gear and riser gear": 152, "Riser idlers and K2": 153,
          "Countershaft C and the REEL idler": 154, "Clutch hub": 155, "Brake crank, rocker and bar": 156,
          "Reel shaft into the wall": 157, "T2 take-off": 158, "SW_ARM contact": 159,
          "Lane tabs and the lane T front post": 180, "Reel unit: P1, sprag #2, capstan tube": 181,
          "Lanes along the deck": 182, "Cheeks, T bracket, spool and spool shaft": 183,
          "Drum, cup and the C plane": 184, "K plane, P2 and the line": 185},
)

TIMELINE = [
    ("load", 2.0, "Dart seated on the sled, drum in LAUNCH, rail low, line taut.",
     dict(wiper_deg=0.0, sleeve_deg=0.0, reel_deg=0.0, drum_deg=0.0, aim_deg=6.0, cock=0.0, fired=0, wraps=0, line_out_m=0.0, dart_flight=0.0, dart_return=0.0, anchored=0, mode="LAUNCH", drum_pos="LAUNCH")),
    ("aim", 4.0, "Wiper backward: the crank-rocker lifts the rail to 36 deg; the worm holds it.",
     dict(wiper_deg=-1120.0, sleeve_deg=-1120.0, aim_deg=36.0)),
    ("cock", 3.0, "Wiper forward: the big gear draws the sled back 241 mm; the take-up winds the slack.",
     dict(wiper_deg=-1120.0 + 425.0, sleeve_deg=-1120.0 + 425.0, cock=1.0)),
    ("fire", 0.5, "The relieved tooth lets go: the sled throws the dart; the big gear resets into its gap.",
     dict(wiper_deg=-1120.0 + 553.8, sleeve_deg=-1120.0 + 553.8, cock=0.0, fired=1, dart_flight=0.05)),
    ("flight", 1.5, "Dart in flight; the line pays off the storage spool against the fixed 0.25 N LIGHT clutch.",
     dict(dart_flight=1.0, line_out_m=5.2)),
    ("rail_low", 4.0, "D45: wiper backward through the rest of the aim cycle: the rail comes down to 6 deg before the wrap, so tow tension cannot push the carriage back.",
     dict(aim_deg=6.0, wiper_deg=-566.2 - 2480.0, sleeve_deg=-566.2 - 2480.0)),
    ("neutral", 8.0, "Unload jog, then the drum goes to NEUTRAL: wiper coupled to nothing.", dict(drum_deg=41.0, mode="NEUTRAL", drum_pos="NEUTRAL")),
    ("wrap", 49.0, "Right after landing, while the line still lies slack: the drum turns the wrap cup 7 times round the capstan.",
     dict(drum_deg=302.0, wraps=7, line_out_m=4.3, drum_pos="BETWEEN")),
    ("wrapped", 2.5, "Drum to the WRAPPED park (314): cup locked, clutch FIRM, collar still NEUTRAL.", dict(drum_deg=314.0, drum_pos="WRAPPED")),
    ("anchor", 4.0, "The dart plants its foot and screws its two drills into the ground.", dict(anchored=1)),
    ("tidy", 3.0, "Wiper forward: the take-up pulls the slack through the free capstan until the line is straight.",
     dict(wiper_deg=-3046.2 + 430.2, line_out_m=4.1)),
    ("reel_engage", 6.5, "Drum into REEL while the wiper creeps forward: the collar drops into the REEL dogs.",
     dict(drum_deg=347.0, wiper_deg=-2610.0, reel_deg=0.0, mode="REEL", drum_pos="REEL")),
    ("tow", 5.0, "Capstan winches the rover toward the anchored dart; sprag #2 holds the load.",
     dict(wiper_deg=-2610.0 + 3600.0, reel_deg=0.0 + 3600.0, line_out_m=1.6)),
    ("release", 2.0, "The base stops (the worm holds the line), then the dart unscrews its drills and lets go.", dict(anchored=0)),
    ("reel_in", 3.0, "Capstan reels the dart in until about 1.5 m remain.",
     dict(wiper_deg=990.0 + 600.0, reel_deg=3600.0 + 600.0, line_out_m=1.3, dart_return=0.3)),
    ("unload", 6.5, "Wiper jogs 6 deg back inside the dog window, then the drum goes back to the WRAPPED park (314): the collar leaves the REEL dogs.",
     dict(wiper_deg=1590.0 - 6.0, drum_deg=314.0, mode="NEUTRAL", drum_pos="WRAPPED")),
    ("unwrap", 47.0, "Drum from the park back through the wrap sector to 62, collar in NEUTRAL; the wiper turns forward slowly and the take-up winds the freed line.",
     dict(drum_deg=62.0, wraps=0, wiper_deg=1584.0 + 1800.0, mode="NEUTRAL", drum_pos="BETWEEN")),
    ("approach", 4.0, "Drum to NEUTRAL; the FIRM take-up pulls the dart tail first onto the sled until SW_SEATED.",
     dict(drum_deg=41.0, line_out_m=0.0, dart_return=1.0, wiper_deg=3384.0 + 1487.8, drum_pos="NEUTRAL")),
    ("to_launch", 8.0, "Drum to LAUNCH: clutch LIGHT, collar into the LAUNCH dogs (a 2 deg alignment creep). Rail already low: ready to aim again.",
     dict(drum_deg=0.0, dart_flight=0.0, dart_return=0.0, wiper_deg=4873.8, mode="LAUNCH", drum_pos="LAUNCH")),
]
