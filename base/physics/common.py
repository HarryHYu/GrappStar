"""Shared parameter blocks and part models for the base station physics.

Every assumption is a named value in one place. When base/spec/params.py exists, `load_params()` overlays the spec
numbers onto these defaults (see spec_bridge.py), so every model runs on the agreed geometry.

Units: SI everywhere inside the code (m, kg, s, N, N m, rad). Names ending in _mm, _g, _deg, _rpm are the exception and
say so.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict, replace

G = 9.81
RHO_AIR = 1.20           # kg/m3, 20 C sea level
NU_AIR = 1.5e-5          # m2/s


# ----------------------------------------------------------------------------------------------- battery + wiring
@dataclass
class Battery:
    """3S LiPo with a simple series resistance sag model. MEASURE: open-circuit voltage at the bench, and the drop at a
    known current (e.g. the wiper stalled for 1 s) to get R_int."""
    v_full: float = 12.6       # V, 4.20 V/cell
    v_nom: float = 11.1        # V, 3.70 V/cell
    v_empty: float = 10.5      # V, 3.50 V/cell under light load (stop using below this)
    r_int: float = 0.040       # ohm, whole pack (typical 2200 to 5200 mAh 3S, 25C+)
    r_wiring: float = 0.020    # ohm, XT60 leads, 14 AWG runs, fuse holder, both ways
    r_driver: float = 0.016    # ohm, BTS7960 high + low side (8 mOhm each, datasheet typ at 25 C)
    fuse_a: float = 20.0       # A, blade fuse on lead #2

    @property
    def r_source(self) -> float:
        return self.r_int + self.r_wiring + self.r_driver


# ----------------------------------------------------------------------------------------------- wiper motor
@dataclass
class Wiper:
    """12 V car wiper motor, worm gearbox (self-locking), output shaft values, low-speed winding.
    MODEL UNKNOWN: these are typical numbers. MEASURE: free speed (count turns for 30 s at 12.0 V), free current, stall
    torque (arm + luggage scale, 1 s only) and stall current. Then edit this block only.
    Linear torque-speed at v_rated; at another terminal voltage the free speed and stall torque scale with the back-EMF
    and the winding resistance (see `at_voltage`)."""
    v_rated: float = 12.0
    n0_rpm: float = 45.0       # free speed at v_rated
    t_stall: float = 15.0      # N m at the output shaft at v_rated
    i_stall: float = 18.0      # A at v_rated
    i_free: float = 1.5        # A at v_rated
    backdrivable: bool = False # worm: holds any load unpowered
    limiter_nm: float | None = 5.0   # printed torque limiter in the wiper coupler (spec D8); None = no limiter
    inertia: float = 0.04      # kg m2 reflected to the output: armature ~2e-5 kg m2 x (ratio ~50)^2 x 0.8. Rough.
    # note: kt / ke = 0.39 here, i.e. the gearbox (worm) efficiency implied by these numbers is about 39 %

    @property
    def r_winding(self) -> float:
        return self.v_rated / self.i_stall

    @property
    def ke(self) -> float:
        """back-EMF constant at the output shaft, V per rad/s"""
        w0 = self.n0_rpm * 2 * math.pi / 60
        return (self.v_rated - self.i_free * self.r_winding) / w0

    @property
    def kt(self) -> float:
        """effective torque constant at the output (includes gearbox losses), N m per A above free current"""
        return self.t_stall / (self.i_stall - self.i_free)

    def current_at(self, torque: float, v_terminal: float | None = None) -> float:
        return self.i_free + max(torque, 0.0) / self.kt

    def speed_at(self, torque: float, v_terminal: float | None = None) -> float:
        """output speed in rad/s at a given output torque and terminal voltage (no source resistance here)."""
        v = self.v_rated if v_terminal is None else v_terminal
        i = self.current_at(torque)
        return max(0.0, (v - i * self.r_winding) / self.ke)

    def stall_torque_at(self, v_terminal: float) -> float:
        i_s = v_terminal / self.r_winding
        return max(0.0, self.kt * (i_s - self.i_free))

    def operate(self, torque: float, battery: "Battery", v_oc: float, duty: float = 1.0):
        """Steady operating point with source sag. PWM duty scales the average applied voltage (BTS7960 at 20 kHz:
        current is continuous, so the average model holds). Returns dict(speed rad/s, rpm, current A, v_motor V,
        v_batt V, power_in W, stalled bool)."""
        i = self.current_at(torque)
        # battery current = duty * motor current (buck-like averaging through the H bridge)
        v_batt = v_oc - duty * i * battery.r_source
        v_mot = duty * v_batt
        w = (v_mot - i * self.r_winding) / self.ke
        stalled = w <= 0
        if stalled:
            # stalled: current is set by the voltage, not the load
            i = duty * v_oc / (self.r_winding + duty * duty * battery.r_source)
            v_batt = v_oc - duty * i * battery.r_source
            v_mot = duty * v_batt
            w = 0.0
        return dict(speed=w, rpm=w * 60 / (2 * math.pi), current=i, i_batt=duty * i, v_motor=v_mot, v_batt=v_batt,
                    power_in=v_batt * duty * i, stalled=stalled)


# ----------------------------------------------------------------------------------------------- materials
@dataclass
class Printed:
    """Printed plastics. Allowables are deliberately low: across-layer strength, notch at the tooth root, creep, heat
    in the sun. Lewis allowable = bending stress the tooth root may see in service."""
    name: str
    rho: float               # kg/m3 solid
    e_mod: float             # Pa
    sigma_t: float           # Pa, tensile along layers (datasheet-ish, printed)
    sigma_lewis: float       # Pa, allowable root bending for Lewis (conservative)
    tau_allow: float         # Pa, allowable shear / torsion for shafts and keys
    mu_dyneema: tuple = (0.15, 0.25)   # friction with Dyneema braid (low, high)


PLA = Printed("PLA", 1240.0, 3.0e9, 45e6, 12e6, 8e6)
PPA_CF = Printed("PPA-CF", 1240.0, 7.0e9, 90e6, 25e6, 12e6)   # Bambu PPA-CF: ~1.2 g/cm3 printed, XY tensile ~ 90 MPa


@dataclass
class MusicWire:
    """ASTM A228 music wire. Shigley 10th ed. Table 10-4 (A, m) and Table 10-7 (extension spring static limits)."""
    G: float = 81.7e9          # Pa (Shigley 81.7 GPa for A228; some catalogues use 79.3)
    E: float = 196.5e9
    rho: float = 7850.0
    A_mpa: float = 2211.0      # Sut = A / d^m, d in mm, MPa
    m: float = 0.145
    frac_body: float = 0.45    # body torsion, static, of Sut (Shigley 10-7: 45 to 50 percent)
    frac_hook_bend: float = 0.75   # end bending (section A)
    frac_hook_tors: float = 0.40   # end torsion (section B)

    def sut(self, d_mm: float) -> float:
        return self.A_mpa / d_mm ** self.m * 1e6


# ----------------------------------------------------------------------------------------------- extension spring
@dataclass
class ExtSpring:
    """Century Spring style extension spring, machine hooks. Lengths in mm for readability (converted in methods).
    MEASURE: hang test. Hang 1, 2, 3 kg from one spring, measure length L each time. rate = slope, initial tension =
    force where the line through the points meets the free length."""
    od_mm: float = 16.0
    wire_mm: float = 1.4
    length_mm: float = 165.1          # free length over the hooks (outside to outside), as Century lists it
    rate_n_mm: float | None = 0.125   # catalogue / estimate; None -> computed from the coil count
    f_init: float = 7.5               # N, initial tension (5 to 10 N band)
    hook_r2_over_d: float = 2.5       # bend radius at the hook root / wire diameter (Shigley r2, typical 2 to 3)
    wire: MusicWire = field(default_factory=MusicWire)
    safety: float = 1.2               # extra factor on the static allowables (shot count in the hundreds)

    @property
    def D_mm(self) -> float:          # mean coil diameter
        return self.od_mm - self.wire_mm

    @property
    def C(self) -> float:
        return self.D_mm / self.wire_mm

    @property
    def hook_len_mm(self) -> float:   # one machine hook, body end to the outside of the loop: about OD - d
        return self.od_mm - self.wire_mm

    @property
    def n_body(self) -> float:        # close-wound body: (Nb + 1) d = body length
        body = self.length_mm - 2 * self.hook_len_mm
        return body / self.wire_mm - 1

    @property
    def n_active(self) -> float:      # Shigley: Na = Nb + G/E
        return self.n_body + self.wire.G / self.wire.E

    @property
    def rate_formula(self) -> float:  # N/mm
        d, D = self.wire_mm, self.D_mm
        return self.wire.G * 1e-6 * d ** 4 / (8 * D ** 3 * self.n_active)

    @property
    def k(self) -> float:             # N/m
        return (self.rate_n_mm if self.rate_n_mm is not None else self.rate_formula) * 1000

    @property
    def mass(self) -> float:          # kg, body + 2 hooks
        wire_len = math.pi * self.D_mm * (self.n_body + 2 * 0.75) * 1e-3
        return self.wire.rho * math.pi / 4 * (self.wire_mm * 1e-3) ** 2 * wire_len

    def force(self, ext_m: float) -> float:
        return self.f_init + self.k * max(ext_m, 0.0) if ext_m > 0 else 0.0

    def energy(self, ext_m: float) -> float:
        return self.f_init * ext_m + 0.5 * self.k * ext_m ** 2

    # stresses per newton (Pa/N), Shigley 10-2, 10-34, 10-35
    def tau_body_per_n(self) -> float:
        C = self.C
        Kb = (4 * C + 2) / (4 * C - 3)
        return Kb * 8 * (self.D_mm * 1e-3) / (math.pi * (self.wire_mm * 1e-3) ** 3)

    def sigma_hook_per_n(self) -> float:
        C1 = self.C                     # hook mean radius = coil mean radius
        Ka = (4 * C1 ** 2 - C1 - 1) / (4 * C1 * (C1 - 1))
        d = self.wire_mm * 1e-3
        D = self.D_mm * 1e-3
        return Ka * 16 * D / (math.pi * d ** 3) + 4 / (math.pi * d ** 2)

    def tau_hook_per_n(self) -> float:
        C2 = 2 * self.hook_r2_over_d
        Kb = (4 * C2 - 1) / (4 * C2 - 4)
        return Kb * 8 * (self.D_mm * 1e-3) / (math.pi * (self.wire_mm * 1e-3) ** 3)

    def limits(self) -> dict:
        sut = self.wire.sut(self.wire_mm)
        f_body = self.wire.frac_body * sut / self.tau_body_per_n() / self.safety
        f_hb = self.wire.frac_hook_bend * sut / self.sigma_hook_per_n() / self.safety
        f_ht = self.wire.frac_hook_tors * sut / self.tau_hook_per_n() / self.safety
        f_max = min(f_body, f_hb, f_ht)
        gov = {f_body: "body torsion", f_hb: "hook bending", f_ht: "hook torsion"}[f_max]
        ext = (f_max - self.f_init) / self.k
        # initial tension sanity: Shigley 10-28 preferred range of uncorrected tau_i for this index
        C = self.C
        tau_i_hi = (33500 / math.exp(0.105 * C) + 1000 * (4 - (C - 3) / 6.5)) * 6.895e3
        tau_i_lo = (33500 / math.exp(0.105 * C) - 1000 * (4 - (C - 3) / 6.5)) * 6.895e3
        tau_i = self.f_init * 8 * self.D_mm * 1e-3 / (math.pi * (self.wire_mm * 1e-3) ** 3)
        return dict(sut_mpa=sut / 1e6, f_body_n=f_body, f_hook_bend_n=f_hb, f_hook_tors_n=f_ht, f_safe_n=f_max,
                    governed_by=gov, ext_safe_mm=ext * 1000, safety=self.safety,
                    tau_init_mpa=tau_i / 1e6, tau_init_pref_mpa=(tau_i_lo / 1e6, tau_i_hi / 1e6))

    def surge(self) -> dict:
        """Surge (longitudinal wave) frequencies of the coil body. Fixed-fixed: f = d/(2 pi D^2 Na) sqrt(G/(2 rho)).
        One end released (fixed-free) is half of that."""
        d, D = self.wire_mm * 1e-3, self.D_mm * 1e-3
        f_ff = d / (2 * math.pi * D ** 2 * self.n_active) * math.sqrt(self.wire.G / (2 * self.wire.rho))
        return dict(f_fixed_fixed_hz=f_ff, f_fixed_free_hz=f_ff / 2, t_wave_one_way_ms=1000 / (2 * f_ff))


# ----------------------------------------------------------------------------------------------- tether
@dataclass
class Line:
    """1 mm Dyneema (UHMWPE) braid, 150 lb class."""
    d: float = 1.0e-3
    lam: float = 0.60e-3       # kg/m (1 mm 12-strand braid ~0.55 to 0.70 g/m)
    breaking: float = 667.0    # N (150 lb)
    length: float = 20.0
    ea: float = 2.5e4          # N, axial stiffness (E ~ 60 GPa effective on the braid area ~0.42 mm2): ~2 to 4 percent at break
    cd_normal: float = 1.2     # cross-flow on a rough braid at Re ~ 100 to 1000
    cf_axial: float = 0.010    # skin friction coefficient on the wetted perimeter (axial flow)


# ----------------------------------------------------------------------------------------------- dart
@dataclass
class Dart:
    """KEDGE v30 folded, with 15 g nose ballast. kedge/out30/physics.json: 264.3 g, CG x 83.0 (before ballast);
    the workplan says about 292 g flying (ballast + tether knot + spare margin): 292 g is used.
    Drag area: kedge/gs/physics_gs.py uses Cd 1.0 x 2864 mm2 (folded silhouette, barrel.json); kedge/flight_sim.py uses
    0.9 x 1.9 x pi 21^2 = 2370 mm2. The mean (2.6e-3 m2) is the default; both ends are in the sensitivity run."""
    mass: float = 0.292
    cda: float = 2.6e-3         # m2
    cda_range: tuple = (2.37e-3, 2.86e-3)
    length: float = 0.152
    diameter: float = 0.042


# ----------------------------------------------------------------------------------------------- 608 bearing
@dataclass
class Bearing608:
    C_dyn: float = 3300.0       # N
    C0: float = 1370.0          # N
    n_max_rpm_zz: float = 30000.0
    n_max_rpm_2rs: float = 18000.0
    drag_torque: float = 2.0e-4  # N m per bearing, greased shielded, light load (0.1 to 0.5 mN m typical)
    mass: float = 0.012


@dataclass
class Sprag:
    """CSK20PP one-way bearing, 20 x 47 x 14, keyways inside and out."""
    t_rated: float = 50.0       # N m
    n_max_rpm: float = 6000.0
    overrun_drag: float = 0.02  # N m (sprag drag when freewheeling, rough)


@dataclass
class Shaft:
    """20 mm printed PPA-CF keyed shaft with a 6 mm key: about 12 N m (8 MPa torsion on the 20 mm section, keyway
    stress concentration and across-layer printing included in the allowable)."""
    d: float = 0.020
    key_w: float = 0.006
    t_allow: float = 12.0


# ----------------------------------------------------------------------------------------------- everything
@dataclass
class Params:
    battery: Battery = field(default_factory=Battery)
    wiper: Wiper = field(default_factory=Wiper)
    spring: ExtSpring = field(default_factory=ExtSpring)
    n_springs: int = 2
    line: Line = field(default_factory=Line)
    dart: Dart = field(default_factory=Dart)
    bearing: Bearing608 = field(default_factory=Bearing608)
    sprag: Sprag = field(default_factory=Sprag)
    shaft: Shaft = field(default_factory=Shaft)
    # launcher (placeholders until base/spec/params.py lands)
    launch: dict = field(default_factory=lambda: dict(
        stroke=0.245,          # m carriage travel under spring force (spec D5)
        pulley_ratio=1.0,      # carriage travel / spring extension (1 = springs pull the carriage directly)
        preload_ext=0.035,     # m spring extension at HOME (spec D5: 35 mm, 280 mm at full cock; alternate hole 5 mm)
        carriage_mass=0.080,   # kg carriage sled + rack + cradle (cad-launch NOTES estimate, CAD value to come)
        mu_rail=0.30,          # PLA on PLA sliding (dart + carriage on the rail)
        eta_pulley=0.97,       # per pulley turn on a 608 or printed bushing
        n_pulley_turns=0,      # cord turns between spring and carriage (0 when direct)
        pitch_deg=35.0,
        stop_pad_travel=0.015, # m travel of the compliant carriage stop (spec D6: at least 15 mm)
        muzzle_height=0.35,    # m above the ground at the muzzle (rover deck + rail)
    ))
    # flight
    flight: dict = field(default_factory=lambda: dict(
        brake_n=0.50,          # N payout brake at the reel exit, full reel (spec D7: 0.5 N in LAUNCH)
        eye_gain=1.15,         # tension gain through the guide eye(s): exp(mu theta), ceramic or smooth PLA
        reel_height=0.25,      # m above ground
        pitch_min=6.0, pitch_max=47.0,  # cad-launch crank-rocker range (6.0 to 47.1)
    ))
    # storage reel
    reel: dict = field(default_factory=lambda: dict(
        core_d=0.040, flange_d=0.064, width=0.020, wall=0.002, flange_t=0.002,
        infill=0.40, line_pitch=1.05e-3, hub_mass=0.010,   # hub/clutch plate that spins with the reel
        line_stored=20.0,
    ))
    # capstan
    capstan: dict = field(default_factory=lambda: dict(
        d=0.040, ratio=0.6, eta=0.85, wraps=5, takeup_n=3.0, tow_limit_n=50.0,   # ratio = wiper turns per capstan turn (REEL gear 40 T -> capstan 24 T); take-up 3 N in REEL (D7)
    ))
    source: str = "defaults (base/spec/params.py not found)"

    def asdict(self) -> dict:
        return asdict(self)


def load_params(spec_path: str | None = None) -> Params:
    """Defaults, overlaid with base/spec/params.py when present."""
    p = Params()
    try:
        from . import spec_bridge  # type: ignore
    except ImportError:
        try:
            import spec_bridge  # type: ignore
        except ImportError:
            return p
    return spec_bridge.apply(p, spec_path)


def fmt(x, n=2):
    return f"{x:.{n}f}"
