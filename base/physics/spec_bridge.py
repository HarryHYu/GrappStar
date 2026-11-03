"""Overlay base/spec/params.py onto the physics defaults.

Only this file knows the spec names. Spec lengths are mm and masses kg; the physics is SI, so lengths are converted here.
Blocks that are still None in the spec leave the defaults alone (and the run says so in p.source).
"""
from __future__ import annotations

import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = os.path.join(HERE, "..", "spec", "params.py")

# spec key -> (physics dict, physics key, scale). Filled in as the spec v1.0 names appear; unknown keys are ignored.
LAUNCH_MAP = {
    "stroke": ("launch", "stroke", 1e-3),
    "STROKE": ("launch", "stroke", 1e-3),
    "pulley_ratio": ("launch", "pulley_ratio", 1.0),
    "preload_ext": ("launch", "preload_ext", 1e-3),
    "carriage_mass_kg": ("launch", "carriage_mass", 1.0),
    "carriage_mass": ("launch", "carriage_mass", 1.0),
    "mu_rail": ("launch", "mu_rail", 1.0),
    "stop_pad_travel": ("launch", "stop_pad_travel", 1e-3),
    "spring_preload": ("launch", "preload_ext", 1e-3),
    "line_guide_u": ("launch", "line_guide_u", 1e-3),
    "line_guide_w": ("launch", "line_guide_w", 1e-3),
    "tail_home_u": ("launch", "tail_home_u", 1e-3),
    "carriage_mass_target_kg": ("launch", "carriage_mass", 1.0),
    "muzzle_height": ("launch", "muzzle_height", 1e-3),
}
AIM_MAP = {
    "pitch_min": ("flight", "pitch_min", 1.0),
    "pitch_max": ("flight", "pitch_max", 1.0),
}
REEL_MAP = {
    "core_d": ("reel", "core_d", 1e-3),
    "flange_d": ("reel", "flange_d", 1e-3),
    "width": ("reel", "width", 1e-3),
    "wall": ("reel", "wall", 1e-3),
    "flange_t": ("reel", "flange_t", 1e-3),
    "hub_mass": ("reel", "hub_mass", 1.0),
    "capstan_d": ("capstan", "d", 1e-3),
    "takeup_n": ("capstan", "takeup_n", 1.0),
    "tow_limit_n": ("capstan", "tow_limit_n", 1.0),
    "brake_n": ("flight", "brake_n", 1.0),
    "reel_height": ("flight", "reel_height", 1e-3),
}


def load_spec_module(path=None):
    path = path or SPEC
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("base_spec_params", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _overlay(p, block, mapping, used):
    if not isinstance(block, dict):
        return
    for k, v in block.items():
        if k in mapping and isinstance(v, (int, float)) and not isinstance(v, bool):
            dname, key, scale = mapping[k]
            getattr(p, dname)[key] = v * scale
            used.append(k)


def apply(p, path=None):
    mod = load_spec_module(path)
    if mod is None:
        return p
    used = []
    w = getattr(mod, "WIPER", None) or {}
    for sk, pk in (("n0_rpm", "n0_rpm"), ("stall_nm", "t_stall"), ("stall_a", "i_stall"), ("free_a", "i_free")):
        if w.get(sk) is not None:
            setattr(p.wiper, pk, float(w[sk]))
            used.append("WIPER." + sk)
    s = getattr(mod, "SPRING", None) or {}
    for sk, pk in (("od", "od_mm"), ("free_len", "length_mm"), ("wire", "wire_mm"), ("k_n_per_mm", "rate_n_mm"),
                   ("F0_n", "f_init")):
        if s.get(sk) is not None:
            setattr(p.spring, pk, float(s[sk]))
            used.append("SPRING." + sk)
    if s.get("qty"):
        p.n_springs = int(s["qty"])
    if s.get("max_ext") is not None:
        p.launch["ext_cap"] = float(s["max_ext"]) * 1e-3
        used.append("SPRING.max_ext")
    ln = getattr(mod, "LINE", None) or {}
    if ln.get("d") is not None:
        p.line.d = float(ln["d"]) * 1e-3
    if ln.get("length_m") is not None:
        p.line.length = float(ln["length_m"])
        p.reel["line_stored"] = float(ln["length_m"])
    if ln.get("break_n") is not None:
        p.line.breaking = float(ln["break_n"])
    d = getattr(mod, "DART", None) or {}
    if d.get("mass_kg") is not None:
        p.dart.mass = float(d["mass_kg"])
        used.append("DART.mass_kg")
    b = getattr(mod, "BATTERY", None) or {}
    for sk, pk in (("v_full", "v_full"), ("v_nom", "v_nom"), ("fuse_a", "fuse_a")):
        if b.get(sk) is not None:
            setattr(p.battery, pk, float(b[sk]))
    _overlay(p, getattr(mod, "LAUNCH", None), LAUNCH_MAP, used)
    lt = getattr(mod, "LAUNCH", None) or {}
    if lt.get("carriage_mass_target_g") is not None:          # D36 (wins over the older _kg key)
        p.launch["carriage_mass"] = float(lt["carriage_mass_target_g"]) * 1e-3
        used.append("LAUNCH.carriage_mass_target_g")
    if lt.get("carriage_mass_g") is not None:                 # as built / slicer mass: wins over the target
        p.launch["carriage_mass"] = float(lt["carriage_mass_g"]) * 1e-3
        used.append("LAUNCH.carriage_mass_g")
    if isinstance(lt.get("stop"), dict) and lt["stop"].get("travel_min") is not None:
        p.launch["stop_pad_travel"] = float(lt["stop"]["travel_min"]) * 1e-3
        used.append("LAUNCH.stop.travel_min")
    lay = getattr(mod, "LAYOUT", None) or {}
    if lay.get("deck_top_z") is not None:
        # main axis (rail pivot) height above the ground = deck height on the rover (ASSUMED) + spec axis height
        p.launch["pivot_h"] = p.launch.get("deck_h", 0.15) - float(lay["deck_top_z"]) * 1e-3
        used.append("LAYOUT.deck_top_z")
    rl = getattr(mod, "REEL", None) or {}
    if rl:
        st = rl.get("storage") or {}
        for sk, pk in (("core_d", "core_d"), ("flange_d", "flange_d"), ("width", "width")):
            if st.get(sk) is not None:
                p.reel[pk] = float(st[sk]) * 1e-3
                used.append("REEL.storage." + sk)
        cp = rl.get("capstan") or {}
        if cp.get("d") is not None:
            p.capstan["d"] = float(cp["d"]) * 1e-3
            used.append("REEL.capstan.d")
        if rl.get("capstan_ratio"):
            p.capstan["ratio"] = 1.0 / float(rl["capstan_ratio"])      # spec: capstan turns per main shaft turn
            used.append("REEL.capstan_ratio")
        if rl.get("wraps") is not None:
            p.capstan["wraps"] = int(rl["wraps"])
        cl = rl.get("clutch") or {}
        if cl.get("LIGHT_line_n") is not None:
            p.flight["brake_n"] = float(cl["LIGHT_line_n"])
            used.append("REEL.clutch.LIGHT_line_n")
        if cl.get("FIRM_line_n") is not None:
            p.capstan["takeup_n"] = float(cl["FIRM_line_n"])
            used.append("REEL.clutch.FIRM_line_n")
        if isinstance(rl.get("current_trips_a"), dict):
            p.capstan["trips"] = {k: float(v) for k, v in rl["current_trips_a"].items() if isinstance(v, (int, float))}
            used.append("REEL.current_trips_a")
        if rl.get("path_gain") is not None:
            p.capstan["path_gain"] = float(rl["path_gain"])
            used.append("REEL.path_gain")
        st_ = rl.get("storage") or {}
        if st_.get("lay_width") is not None:
            p.reel["lay_width"] = float(st_["lay_width"]) * 1e-3
            used.append("REEL.storage.lay_width")
        if rl.get("tow_limit_max_n") is not None:
            p.capstan["tow_limit_max_n"] = float(rl["tow_limit_max_n"])
        if rl.get("tow_limit_n") is not None:
            p.capstan["tow_limit_n"] = float(rl["tow_limit_n"])
    dr = getattr(mod, "DRUM", None) or {}
    if isinstance(dr, dict) and isinstance(dr.get("bus"), dict) and dr["bus"].get("overdrive_total") is not None:
        p.capstan["overspeed"] = float(dr["bus"]["overdrive_total"])
        used.append("DRUM.bus.overdrive_total")
    if isinstance(dr, dict) and dr.get("torque_limiter_nm") is not None:
        p.wiper.limiter_nm = float(dr["torque_limiter_nm"])
        used.append("DRUM.torque_limiter_nm")
    # SPRING.max_ext only caps (launcher.py flags over_safe); the stroke comes from LAUNCH or the D5 defaults
    _overlay(p, getattr(mod, "AIM", None), AIM_MAP, used)
    _overlay(p, getattr(mod, "REEL", None), REEL_MAP, used)
    pending = [n for n in ("LAUNCH", "AIM", "REEL", "LAYOUT") if getattr(mod, n, None) is None]
    p.source = f"base/spec/params.py v{getattr(mod, 'VERSION', '?')}" + (f" (still defaults for {', '.join(pending)})" if pending else "")
    p.spec_used = used
    return p
