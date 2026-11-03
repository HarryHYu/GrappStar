"""Turn Harry's bench measurements into model parameters.

Spring hang test (one spring, hooks included in the length):
  python calib.py spring 165.1 1.0:187 2.0:265.5 3.0:344
  (free length mm, then mass_kg:length_mm pairs) -> rate N/mm and initial tension N (least squares).
Test shots (after the first real launches, level ground, range from the ground point under the rail pivot):
  python calib.py range 36:0.5:4.2 20:0.5:3.6   (pitch_deg:brake_n:measured_range_m)
  -> muzzle speed factor; then: python ballistics.py --v-scale <factor>
Wiper:
  python calib.py wiper --v 12.0 --rpm 44 --i-free 1.4 --stall-kg 3.1 --arm-mm 100 --i-stall 17
  -> the Wiper block values (stall torque = luggage scale kg x 9.81 x arm length).
"""
from __future__ import annotations

import argparse

G = 9.81


def fit_spring(free_mm: float, pairs: list[tuple[float, float]]) -> dict:
    """Least-squares line F = F0 + k x through (x, F), x = length - free length. Points with x <= 0 are dropped
    (initial tension not yet overcome: the spring did not open)."""
    pts = [(L - free_mm, m * G) for m, L in pairs if L - free_mm > 0.5]
    if len(pts) < 2:
        raise ValueError("need at least two loads that open the coils")
    n = len(pts)
    sx = sum(x for x, _ in pts)
    sy = sum(y for _, y in pts)
    sxx = sum(x * x for x, _ in pts)
    sxy = sum(x * y for x, y in pts)
    k = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    f0 = (sy - k * sx) / n
    res = [y - (f0 + k * x) for x, y in pts]
    return dict(rate_n_mm=k, f_init_n=f0, residual_n=max(abs(r) for r in res), points=pts)


def fit_v_scale(shots, p=None):
    """Least-squares muzzle speed factor so the model ranges match measured ones. shots: [(pitch, brake, range_m)]."""
    from common import load_params
    from flight import simulate
    from launcher import launch
    p = p or load_params()
    base = {}
    for th, b, _ in shots:
        base[th] = launch(p, th)["v_muzzle"]

    def err(k):
        return sum((simulate(p, base[th] * k, th, brake_n=b, reel_model=True)["range_m"] - r) ** 2 for th, b, r in shots)
    lo, hi = 0.6, 1.4
    for _ in range(40):                    # golden-section search
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        if err(m1) < err(m2):
            hi = m2
        else:
            lo = m1
        if hi - lo < 1e-3:
            break
    k = 0.5 * (lo + hi)
    rows = [(th, b, r, simulate(p, base[th] * k, th, brake_n=b, reel_model=True)["range_m"]) for th, b, r in shots]
    return dict(v_scale=k, rows=rows, rms_m=(err(k) / len(shots)) ** 0.5)


def wiper_block(v: float, rpm: float, i_free: float, stall_kg: float, arm_mm: float, i_stall: float) -> dict:
    return dict(v_rated=v, n0_rpm=rpm, t_stall=stall_kg * G * arm_mm / 1000, i_stall=i_stall, i_free=i_free)


def main(argv=None):
    try:
        return _main(argv)
    except (ValueError, ZeroDivisionError) as e:
        raise SystemExit(f"calib: {e} (need at least two different loads that open the coils, as kg:mm)")


def _main(argv=None):
    ap = argparse.ArgumentParser(description="bench measurements -> parameters")
    sub = ap.add_subparsers(dest="what", required=True)
    s = sub.add_parser("spring")
    s.add_argument("free_mm", type=float)
    s.add_argument("pairs", nargs="+", help="mass_kg:length_mm")
    rg = sub.add_parser("range")
    rg.add_argument("shots", nargs="+", help="pitch_deg:brake_n:measured_range_m")
    w = sub.add_parser("wiper")
    w.add_argument("--v", type=float, required=True)
    w.add_argument("--rpm", type=float, required=True)
    w.add_argument("--i-free", type=float, required=True)
    w.add_argument("--stall-kg", type=float, required=True)
    w.add_argument("--arm-mm", type=float, required=True)
    w.add_argument("--i-stall", type=float, required=True)
    a = ap.parse_args(argv)
    if a.what == "spring":
        pairs = [tuple(float(x) for x in p.split(":")) for p in a.pairs]
        r = fit_spring(a.free_mm, pairs)
        print(f"rate {r['rate_n_mm']:.4f} N/mm, initial tension {r['f_init_n']:.2f} N, worst residual {r['residual_n']:.2f} N")
        print(f"spec: SPRING k_n_per_mm={r['rate_n_mm']:.4f}, F0_n={r['f_init_n']:.2f}")
        return r
    if a.what == "range":
        shots = [tuple(float(x) for x in s.split(":")) for s in a.shots]
        if any(len(s) != 3 for s in shots):
            raise ValueError("each shot is pitch:brake:range")
        r = fit_v_scale(shots)
        print(f"muzzle speed factor {r['v_scale']:.3f} (rms miss {r['rms_m']:.2f} m)")
        for th, b, meas, mod in r["rows"]:
            print(f"  pitch {th:4.1f}, brake {b} N: measured {meas:.2f} m, model {mod:.2f} m")
        print(f"next: python ballistics.py --v-scale {r['v_scale']:.3f}")
        return r
    r = wiper_block(a.v, a.rpm, a.i_free, a.stall_kg, a.arm_mm, a.i_stall)
    print("common.Wiper / spec WIPER:", ", ".join(f"{k}={v:.3g}" for k, v in r.items()))
    return r


if __name__ == "__main__":
    main()
