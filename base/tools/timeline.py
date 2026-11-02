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


def state_at(P, t: float) -> dict:
    """Master state at mission time t (s). Placeholder kinematics; the subsystem kin() turns these into joints."""
    drum = _p(P, "DRUM_POS", {"LAUNCH": 0.0, "NEUTRAL": 60.0, "REEL": 120.0, "WRAPPED": 300.0})
    aim_deg = _p(P, "AIM_DEMO_DEG", 30.0)
    aim_low = _p(P, "AIM_LOW_DEG", 5.0)
    wiper_rpm = _p(P, "WIPER_RPM", 45.0)
    line_total = _p(P, "LAUNCH_LINE_M", 10.0)
    t0 = 0.0
    S = dict(t=t, wiper_deg=0.0, drum_deg=drum["LAUNCH"], line_out_m=0.0, mode="LAUNCH", aim_deg=aim_low, cock=0.0,
             fired=False, wraps=0.0, phase=PHASES[-1][0])
    wiper = 0.0
    for name, dur, _ in PHASES:
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
            S["wraps"] = _p(P, "WRAPS", 3.0) * _ease(uu)
            S["mode"] = "WRAPPED"
        elif name == "winch":
            wiper += 360 * wiper_rpm / 60 * dur * uu
        elif name == "retrieve":
            S["line_out_m"] = line_total * (1 - _ease(uu))
            wiper += 360 * wiper_rpm / 60 * dur * uu
        elif name == "unwrap":
            S["drum_deg"] = drum["WRAPPED"] + (drum["REEL"] - drum["WRAPPED"]) * _ease(uu)
            S["wraps"] = _p(P, "WRAPS", 3.0) * (1 - _ease(uu))
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


def duration(P=None) -> float:
    return sum(d for _, d, _ in PHASES)


def phases(P=None) -> list[dict]:
    out, t0 = [], 0.0
    for n, d, txt in PHASES:
        out.append(dict(name=n, t0=t0, t1=t0 + d, text=txt))
        t0 += d
    return out


def timeline(P=None, dt: float = 0.1) -> list[dict]:
    n = int(math.ceil(duration(P) / dt))
    return [state_at(P, i * dt) for i in range(n + 1)]
