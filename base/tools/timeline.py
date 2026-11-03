"""Mission timeline: a list of master states S (dicts) with phase labels.

The mission sequence follows knowledge/SYSTEM_PLAN_2026-11-03.md. Numbers come from P where present (BASE_SPEC.md
names) and otherwise from the placeholders below. Subsystems read S with defaults, so extra keys are harmless.

    from tools.timeline import timeline, phases
    states = timeline(P, dt=0.1)        # list of S dicts, each with S["phase"]
"""
from __future__ import annotations

import math

# phase: (label, duration s, text)
PHASES = [
    ("load", 2.0, "Dart seated on the rail, drum in LAUNCH, rail at low."),
    ("aim", 3.0, "Wiper oscillates the rail to the aim angle."),
    ("cock", 4.0, "Wiper reverses: cocks the spring launcher, sear catches."),
    ("fire", 1.5, "Sear trips, dart flies, line pays off the storage reel against the slip clutch."),
    ("flight", 1.5, "Dart in flight, line paying out."),
    ("neutral", 1.0, "Drum to NEUTRAL: wiper coupled to nothing."),
    ("reel_mode", 1.0, "Drum to REEL: dog clutch couples the wiper to the capstan."),
    ("wrap", 3.0, "Drum past REEL: the wrap arm lays the wraps round the capstan."),
    ("winch", 4.0, "Capstan winches the rover towards the anchored dart."),
    ("retrieve", 4.0, "Dart released, reeled home."),
    ("unwrap", 3.0, "Drum back: wrap arm unwraps the capstan."),
    ("reset", 1.5, "Drum to LAUNCH, rail back to low."),
]


def _p(P, name, default):
    return getattr(P, name, default) if P is not None else default


def _ease(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


KNOWN = {n for n, _, _ in PHASES}


def _custom(P):
    """P.TIMELINE when the spec defines one. Entries: (name, duration s, text, targets) where targets is a dict of S
    values reached (eased) at the end of the phase; S keys not named hold their value. Entries without targets must
    use a built-in phase name."""
    tl = _p(P, "TIMELINE", None)
    if not (isinstance(tl, (list, tuple)) and tl):
        return None
    out = []
    for x in tl:
        if isinstance(x, dict):
            x = (x["name"], x["dur"], x.get("text", ""), x.get("targets"))
        name, dur = x[0], float(x[1])
        text = x[2] if len(x) > 2 else ""
        targets = x[3] if len(x) > 3 else None
        if targets is None and name not in KNOWN:
            raise ValueError(f"P.TIMELINE phase {name!r} has no targets and is not a built-in phase {sorted(KNOWN)}")
        out.append((name, dur, text, targets))
    return out


def _phases(P):
    c = _custom(P)
    return [(n, d, t) for n, d, t, _ in c] if c else PHASES


def _drum(P):
    d = _p(P, "DRUM", None)
    base = dict(_p(P, "DRUM_POS", None) or {"LAUNCH": 0.0, "NEUTRAL": 60.0, "REEL": 120.0, "WRAPPED": 300.0})
    if isinstance(d, dict) and isinstance(d.get("angles"), dict):
        base.update(d["angles"])
    missing = {"LAUNCH", "NEUTRAL", "REEL", "WRAPPED"} - set(base)
    if missing:
        raise ValueError(f"drum angles missing {sorted(missing)}")
    return base


def _num(P, name, key, default):
    """P.name[key] (dict) or P.name (number), else default."""
    v = _p(P, name, None)
    if isinstance(v, dict):
        return float(v.get(key, default))
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v)
    return default


def state_at(P, t: float) -> dict:
    """Master state at mission time t (s). Placeholder kinematics; the subsystem kin() turns these into joints.
    World frame: BASE_SPEC.md section 2 (origin on the wiper / main shaft axis)."""
    drum = _drum(P)
    aim = _p(P, "AIM", None) if isinstance(_p(P, "AIM", None), dict) else {}
    aim_low = float(aim.get("pitch_min", aim.get("pitch_load", 5.0)))
    aim_deg = float(aim.get("pitch_demo", min(30.0, aim.get("pitch_max", 30.0))))
    wiper_rpm = _num(P, "WIPER", "n0_rpm", _p(P, "WIPER_RPM", 45.0))
    line_total = _num(P, "LINE", "launch_m", 0.5 * _num(P, "LINE", "length_m", _p(P, "LINE_M", 20.0)))
    wraps_n = float((_p(P, "REEL", None) or {}).get("wraps", 3.0)) if isinstance(_p(P, "REEL", None), dict) else 3.0
    t0 = 0.0
    custom = _custom(P)
    S = dict(t=t, wiper_deg=0.0, drum_deg=drum["LAUNCH"], line_out_m=0.0, mode="LAUNCH", aim_deg=aim_low, cock=0.0,
             fired=False, wraps=0.0, phase=_phases(P)[-1][0])
    wiper = 0.0
    if custom and any(c[3] is not None for c in custom):
        return _state_custom(S, custom, t)
    for name, dur, _ in _phases(P):
        u = (t - t0) / dur if dur else 1.0
        if u < 0:
            break
        uu = min(1.0, u)
        S["phase"] = name
        if name == "aim":
            S["aim_deg"] = aim_low + (aim_deg - aim_low) * _ease(uu)
            wiper += 360 * wiper_rpm / 60 * dur * uu * 0.25
        elif name == "cock":
            S["cock"] = _ease(uu)
            wiper -= 360 * wiper_rpm / 60 * dur * uu
        elif name == "fire":
            S["fired"] = uu > 0.2
            S["cock"] = 1.0 if uu <= 0.2 else max(0.0, 1 - (uu - 0.2) / 0.1)
            S["line_out_m"] = line_total * 0.3 * max(0.0, (uu - 0.2) / 0.8)
        elif name == "flight":
            S["line_out_m"] = line_total * (0.3 + 0.7 * uu)
        elif name == "neutral":
            S["drum_deg"] = drum["LAUNCH"] + (drum["NEUTRAL"] - drum["LAUNCH"]) * _ease(uu)
            S["mode"] = "NEUTRAL" if uu > 0.5 else "LAUNCH"
            S["aim_deg"] = aim_deg
        elif name == "reel_mode":
            S["drum_deg"] = drum["NEUTRAL"] + (drum["REEL"] - drum["NEUTRAL"]) * _ease(uu)
            S["mode"] = "REEL" if uu > 0.5 else "NEUTRAL"
        elif name == "wrap":
            S["drum_deg"] = drum["REEL"] + (drum["WRAPPED"] - drum["REEL"]) * _ease(uu)
            S["wraps"] = wraps_n * _ease(uu)
            S["mode"] = "WRAPPED"
        elif name == "winch":
            wiper += 360 * wiper_rpm / 60 * dur * uu
        elif name == "retrieve":
            S["line_out_m"] = line_total * (1 - _ease(uu))
            wiper += 360 * wiper_rpm / 60 * dur * uu
        elif name == "unwrap":
            S["drum_deg"] = drum["WRAPPED"] + (drum["REEL"] - drum["WRAPPED"]) * _ease(uu)
            S["wraps"] = wraps_n * (1 - _ease(uu))
            S["mode"] = "REEL"
        elif name == "reset":
            S["drum_deg"] = drum["REEL"] + (drum["LAUNCH"] - drum["REEL"]) * _ease(uu)
            S["aim_deg"] = aim_deg + (aim_low - aim_deg) * _ease(uu)
            S["mode"] = "LAUNCH" if uu > 0.5 else "REEL"
            S["fired"] = False
        if u < 1:
            break
        t0 += dur
    S["wiper_deg"] = wiper
    return S


def _state_custom(S, custom, t):
    """Generic timeline: each phase eases the S keys in its targets from their previous values. Every key named in
    any phase starts at its first target value, so no key is ever missing (the first phase holds its own targets)."""
    cur = dict(S)
    seen = set()
    for _, _, _, targets in custom:
        for k, v in (targets or {}).items():
            if k not in seen:
                seen.add(k)
                cur[k] = v
    t0 = 0.0
    for name, dur, _, targets in custom:
        u = (t - t0) / dur if dur else 1.0
        if u < 0:
            break
        cur["phase"] = name
        start = dict(cur)
        e = _ease(min(1.0, u))
        for k, v in (targets or {}).items():
            a = start.get(k)
            flag = isinstance(v, bool) or (isinstance(v, int) and v in (0, 1) and a in (0, 1))
            if isinstance(v, (int, float)) and not flag and isinstance(a, (int, float)) and not isinstance(a, bool):
                cur[k] = a + (v - a) * e          # numbers ease (wraps 0 -> 7 winds evenly); 0/1 flags and bools step
            else:
                cur[k] = v if u >= 0.5 else a
        if u < 1:
            break
        t0 += dur
    cur["t"] = t
    return cur


def duration(P=None) -> float:
    return sum(d for _, d, _ in _phases(P))


def phases(P=None) -> list[dict]:
    out, t0 = [], 0.0
    for n, d, txt in _phases(P):
        out.append(dict(name=n, t0=t0, t1=t0 + d, text=txt))
        t0 += d
    return out


def timeline(P=None, dt: float = 0.1) -> list[dict]:
    n = int(math.ceil(duration(P) / dt))
    return [state_at(P, i * dt) for i in range(n + 1)]
