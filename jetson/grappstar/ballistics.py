"""Launch solver (contract C3): pick the rail pitch for a target range.

Reads base/out/ballistics.json (written by the base physics model):
  {"version": 1, "pitch_min": .., "pitch_max": .., "drag_n": [0.5, 0.75, 1.0],
   "table": [{"pitch_deg": .., "v_muzzle": .., "range_m": [r05, r075, r10], "t_flight_s": [...], "apex_m": [...]}]}
Until that file exists it uses grappstar/data/ballistics_placeholder.json (made by make_placeholder() below from a
simple point-mass model: NOT the real launcher, marked "placeholder": true).

Rules:
- Interpolate linearly in pitch and in drag (drag outside the table is clamped).
- Range rises with pitch up to a peak (about 36 deg) and then falls. When two pitches reach the range, prefer the
  higher one (coordinator decision D29); lob=False gives the flatter, faster low branch.
- range_m is measured from the ground point under the rail pivot (D29): callers add the camera's offset.
- Line payout = path length of the flight approximated by the straight distance plus sag, times a margin.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from typing import List, Optional

PREFER_HIGH = True   # D29 (3 Nov): prefer the higher pitch when two solutions exist
HERE = os.path.dirname(os.path.abspath(__file__))
PLACEHOLDER = os.path.join(HERE, "data", "ballistics_placeholder.json")
REPO_TABLE = os.path.normpath(os.path.join(HERE, "..", "..", "base", "out", "ballistics.json"))


@dataclass
class Solution:
    reachable: bool
    pitch_deg: Optional[float]
    t_flight_s: Optional[float]
    line_out_m: Optional[float]
    apex_m: Optional[float]
    v_muzzle: Optional[float]
    range_m: float
    drag_n: float
    range_min_m: float
    range_max_m: float
    note: str = ""
    source: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


def _lerp(a: float, b: float, f: float) -> float:
    return a + (b - a) * f


class Ballistics:
    def __init__(self, data: dict, source: str = ""):
        self.data = data
        self.source = source
        self.placeholder = bool(data.get("placeholder"))
        self.drag = [float(x) for x in data["drag_n"]]
        rows = sorted(data["table"], key=lambda r: r["pitch_deg"])
        if len(rows) < 2:
            raise ValueError("ballistics table needs at least 2 rows")
        self.rows = rows
        self.pitch_min = float(data.get("pitch_min", rows[0]["pitch_deg"]))
        self.pitch_max = float(data.get("pitch_max", rows[-1]["pitch_deg"]))
        for r in rows:
            for k in ("range_m", "t_flight_s", "apex_m"):
                if len(r[k]) != len(self.drag):
                    raise ValueError(f"row {r['pitch_deg']}: {k} must have one value per drag setting")

    @classmethod
    def load(cls, path: Optional[str] = None) -> "Ballistics":
        for p in ([path] if path else [os.environ.get("GRAPPSTAR_BALLISTICS"), REPO_TABLE, PLACEHOLDER]):
            if p and os.path.exists(p):
                with open(p) as f:
                    return cls(json.load(f), source=p)
        raise FileNotFoundError("no ballistics table found")

    # column for one drag value (interpolated)
    def _col(self, key: str, drag_n: float) -> List[float]:
        d = self.drag
        x = min(max(drag_n, d[0]), d[-1])
        for j in range(len(d) - 1):
            if d[j] <= x <= d[j + 1]:
                f = 0.0 if d[j + 1] == d[j] else (x - d[j]) / (d[j + 1] - d[j])
                return [_lerp(r[key][j], r[key][j + 1], f) for r in self.rows]
        return [r[key][0] for r in self.rows]

    def at_pitch(self, pitch_deg: float, drag_n: float = 0.75) -> dict:
        """range, flight time and apex for one pitch."""
        ps = [r["pitch_deg"] for r in self.rows]
        p = min(max(pitch_deg, ps[0]), ps[-1])
        out = {"pitch_deg": p}
        for key in ("range_m", "t_flight_s", "apex_m"):
            col = self._col(key, drag_n)
            for i in range(len(ps) - 1):
                if ps[i] <= p <= ps[i + 1]:
                    f = (p - ps[i]) / (ps[i + 1] - ps[i])
                    out[key] = _lerp(col[i], col[i + 1], f)
                    break
        vs = [r.get("v_muzzle", 0.0) for r in self.rows]
        for i in range(len(ps) - 1):
            if ps[i] <= p <= ps[i + 1]:
                out["v_muzzle"] = _lerp(vs[i], vs[i + 1], (p - ps[i]) / (ps[i + 1] - ps[i]))
        return out

    def solve(self, range_m: float, drag_n: float = 0.75, lob: Optional[bool] = None,
              line_margin: float = 1.10) -> Solution:
        """Pitch for a range. lob=None uses PREFER_HIGH (coordinator D29: prefer the higher pitch when two solutions
        exist; the steeper shot lands nose-down for the augers and clears low obstacles). Ranges are from the ground
        point under the rail pivot (D29)."""
        if lob is None:
            lob = PREFER_HIGH
        ps = [r["pitch_deg"] for r in self.rows]
        rng = self._col("range_m", drag_n)
        tf = self._col("t_flight_s", drag_n)
        ap = self._col("apex_m", drag_n)
        vs = [r.get("v_muzzle", 0.0) for r in self.rows]
        ok_idx = [i for i, p in enumerate(ps) if self.pitch_min - 1e-9 <= p <= self.pitch_max + 1e-9]
        rmin = min(rng[i] for i in ok_idx)
        rmax = max(rng[i] for i in ok_idx)
        base = dict(range_m=float(range_m), drag_n=float(drag_n), range_min_m=round(rmin, 2), range_max_m=round(rmax, 2),
                    source=os.path.basename(self.source) + (" (placeholder)" if self.placeholder else ""))
        if not (range_m > 0):
            return Solution(False, None, None, None, None, None, note="range must be positive", **base)
        # candidate crossings, in the allowed pitch window
        cands = []
        for a, b in zip(ok_idx, ok_idx[1:]):
            r0, r1 = rng[a], rng[b]
            if (r0 - range_m) * (r1 - range_m) <= 0 and r0 != r1:
                f = (range_m - r0) / (r1 - r0)
                cands.append((_lerp(ps[a], ps[b], f), _lerp(tf[a], tf[b], f), _lerp(ap[a], ap[b], f),
                              _lerp(vs[a], vs[b], f)))
        if not cands:
            why = "too far" if range_m > rmax else "too close"
            return Solution(False, None, None, None, None, None,
                            note=f"{why}: reachable {rmin:.1f} to {rmax:.1f} m at drag {drag_n:g} N", **base)
        cands.sort(key=lambda c: c[0])
        p, t, apex, v = cands[-1] if lob else cands[0]
        sag = 8.0 * (apex * 0.25) ** 2 / (3.0 * max(range_m, 0.1))   # small extra for the line's arc
        line = (math.hypot(range_m, 0.0) + sag) * line_margin
        note = ("high branch" if lob else "low branch") if len(cands) > 1 else ""
        if self.placeholder:
            note = (note + "; " if note else "") + "PLACEHOLDER table: not the real launcher"
        return Solution(True, round(p, 2), round(t, 3), round(line, 2), round(apex, 2), round(v, 2), note=note, **base)


def make_placeholder(path: str = PLACEHOLDER, v0: float = 10.7, mass: float = 0.279, cd_a: float = 0.5 * 1.2 * 0.6 * 0.0014,
                     h0: float = 0.25) -> dict:
    """Point-mass flight with a constant line drag force (N) along -velocity plus quadratic air drag."""
    g = 9.81
    drags = [0.5, 0.75, 1.0]
    table = []
    for pitch in range(15, 61, 5):
        rs, ts, aps = [], [], []
        for dn in drags:
            th = math.radians(pitch)
            x, z, vx, vz, t, apex = 0.0, h0, v0 * math.cos(th), v0 * math.sin(th), 0.0, h0
            dt = 0.001
            while z > 0.0 and t < 10:
                v = math.hypot(vx, vz) or 1e-9
                f = dn + cd_a * v * v
                ax = -f * vx / v / mass
                az = -g - f * vz / v / mass
                vx += ax * dt; vz += az * dt
                x += vx * dt; z += vz * dt; t += dt
                apex = max(apex, z)
            rs.append(round(x, 3)); ts.append(round(t, 3)); aps.append(round(apex, 3))
        table.append({"pitch_deg": float(pitch), "v_muzzle": v0, "range_m": rs, "t_flight_s": ts, "apex_m": aps})
    data = {"version": 1, "placeholder": True,
            "note": "Placeholder from grappstar.ballistics.make_placeholder (point mass, v0 10.7 m/s, 0.279 kg). "
                    "Replace with base/out/ballistics.json from the base physics model.",
            "pitch_min": 15.0, "pitch_max": 60.0, "drag_n": drags, "table": table}
    if path:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f, indent=1)
    return data


if __name__ == "__main__":
    d = make_placeholder()
    print("wrote", PLACEHOLDER, "rows", len(d["table"]))
