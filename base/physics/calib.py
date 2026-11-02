"""Turn Harry's bench measurements into model parameters.

Spring hang test (one spring, hooks included in the length):
  python calib.py spring 165.1 1.0:236 2.0:315 3.0:393
  (free length mm, then mass_kg:length_mm pairs) -> rate N/mm and initial tension N (least squares).
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


def wiper_block(v: float, rpm: float, i_free: float, stall_kg: float, arm_mm: float, i_stall: float) -> dict:
    return dict(v_rated=v, n0_rpm=rpm, t_stall=stall_kg * G * arm_mm / 1000, i_stall=i_stall, i_free=i_free)


def main(argv=None):
    ap = argparse.ArgumentParser(description="bench measurements -> parameters")
    sub = ap.add_subparsers(dest="what", required=True)
    s = sub.add_parser("spring")
    s.add_argument("free_mm", type=float)
    s.add_argument("pairs", nargs="+", help="mass_kg:length_mm")
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
    r = wiper_block(a.v, a.rpm, a.i_free, a.stall_kg, a.arm_mm, a.i_stall)
    print("common.Wiper / spec WIPER:", ", ".join(f"{k}={v:.3g}" for k, v in r.items()))
    return r


if __name__ == "__main__":
    main()
