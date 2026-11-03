"""Turn Harry's bench measurements into model parameters.

Spring hang test (one spring, hooks included in the length):
  python calib.py spring 165.1 1.0:187 2.0:265.5 3.0:344
  (free length mm, then mass_kg:length_mm pairs) -> rate N/mm and initial tension N (least squares).
Test shots (after the first real launches, level ground, range from the ground point under the rail pivot):
  python calib.py range 36:0.5:4.2 20:0.5:3.6   (pitch_deg:brake_n:measured_range_m)
  -> muzzle speed factor; put LAUNCH v_scale = <factor> in base/spec/params.py, then ./run_all.sh
Wiper:
  python calib.py wiper --v 12.0 --rpm 44 --i-free 1.4 --stall-kg 3.1 --arm-mm 100 --i-stall 17
  -> the spec WIPER values n0_rpm, stall_nm, stall_a, free_a (stall torque = luggage scale kg x 9.81 x arm length).
"""
from __future__ import annotations

import argparse

G = 9.81
CATALOGUE_N = 38.4          # Century C-243 max safe load (spec SPRING.catalogue)
SOFT_LIMIT_N = 40.0         # accepted for a low cycle count (D30)
MAX_EXT_MM = 265.0          # spec SPRING.max_ext default
FALLBACK_EXT_MM = 250.0     # spec SPRING.max_ext_fallback default


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
    from launcher import effective_muzzle
    p = p or load_params()
    base = {}
    for th, b, _ in shots:
        base[th] = effective_muzzle(p, th)["v"]          # the same speed ballistics.py scales

    def err(k):
        return sum((simulate(p, base[th] * k, th, brake_n=b, reel_model=True)["range_m"] - r) ** 2 for th, b, r in shots)
    lo, hi = 0.6, 1.4
    g = 0.6180339887
    m1, m2 = hi - g * (hi - lo), lo + g * (hi - lo)
    e1, e2 = err(m1), err(m2)
    while hi - lo > 2e-3:                  # golden-section search, one new evaluation per step
        if e1 < e2:
            hi, m2, e2 = m2, m1, e1
            m1 = hi - g * (hi - lo)
            e1 = err(m1)
        else:
            lo, m1, e1 = m1, m2, e2
            m2 = lo + g * (hi - lo)
            e2 = err(m2)
    k = 0.5 * (lo + hi)
    rows = [(th, b, r, simulate(p, base[th] * k, th, brake_n=b, reel_model=True)["range_m"]) for th, b, r in shots]
    return dict(v_scale=k, rows=rows, rms_m=(err(k) / len(shots)) ** 0.5)


def wiper_block(v: float, rpm: float, i_free: float, stall_kg: float, arm_mm: float, i_stall: float) -> dict:
    return dict(v_rated=v, n0_rpm=rpm, t_stall=stall_kg * G * arm_mm / 1000, i_stall=i_stall, i_free=i_free)


def wiper_spec(r: dict) -> dict:
    """wiper_block -> the spec WIPER key names."""
    return dict(n0_rpm=r["n0_rpm"], stall_nm=r["t_stall"], stall_a=r["i_stall"], free_a=r["i_free"])


def spec_spring():
    """(max_ext, fallback_ext, catalogue N) from base/spec/params.py, defaults if the spec is missing."""
    try:
        import spec_bridge
        mod = spec_bridge.load_spec_module()
    except Exception:
        mod = None
    s = (getattr(mod, "SPRING", None) or {}) if mod is not None else {}
    cat = (s.get("catalogue") or {}).get("max_safe_load_n", CATALOGUE_N)
    return float(s.get("max_ext") or MAX_EXT_MM), float(s.get("max_ext_fallback") or FALLBACK_EXT_MM), float(cat)


def spring_verdict(f_n: float, catalogue_n: float = CATALOGUE_N, soft_n: float = SOFT_LIMIT_N) -> str:
    if f_n <= catalogue_n:
        return "OK"
    if f_n <= soft_n:
        return "over catalogue, accept for low cycles"
    return "too high: use the 250 mm holes"


HINTS = dict(spring="need at least two different loads that open the coils, as kg:mm",
             range="each shot is pitch_deg:brake_n:measured_range_m, e.g. 36:0.5:4.2",
             wiper="all of --v --rpm --i-free --stall-kg --arm-mm --i-stall are numbers")


def main(argv=None):
    import sys
    args = list(sys.argv[1:] if argv is None else argv)
    what = next((x for x in args if x in HINTS), None)
    try:
        return _main(args)
    except (ValueError, ZeroDivisionError) as e:
        hint = f" ({HINTS[what]})" if what else ""
        raise SystemExit(f"calib {what or ''}: {e}{hint}".replace("calib : ", "calib: "))


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
        ext, fb, cat = spec_spring()
        for L in (ext, fb):
            x = L - a.free_mm
            f = r["f_init_n"] + r["rate_n_mm"] * x
            r[f"force_at_{L:g}mm_n"] = f
            print(f"force per spring at {L:g} mm ({x:.1f} mm extension): {f:.1f} N "
                  f"(catalogue {cat:g} N, soft limit {SOFT_LIMIT_N:g} N): {spring_verdict(f, cat)}")
        r["force_full_cock_n"] = r[f"force_at_{ext:g}mm_n"]
        r["verdict"] = spring_verdict(r["force_full_cock_n"], cat)
        print("then edit base/spec/params.py and run ./run_all.sh")
        return r
    if a.what == "range":
        shots = [tuple(float(x) for x in s.split(":")) for s in a.shots]
        if any(len(s) != 3 for s in shots):
            raise ValueError("a shot does not have three numbers")
        r = fit_v_scale(shots)
        print(f"muzzle speed factor {r['v_scale']:.3f} (rms miss {r['rms_m']:.2f} m)")
        for th, b, meas, mod in r["rows"]:
            print(f"  pitch {th:4.1f}, brake {b} N: measured {meas:.2f} m, model {mod:.2f} m")
        print(f"put LAUNCH v_scale = {r['v_scale']:.3f} in base/spec/params.py, then ./run_all.sh")
        return r
    r = wiper_block(a.v, a.rpm, a.i_free, a.stall_kg, a.arm_mm, a.i_stall)
    print("spec WIPER:", ", ".join(f"{k}={v:.3g}" for k, v in wiper_spec(r).items()))
    print("put these in base/spec/params.py WIPER (and measured=True), then ./run_all.sh")
    return r


if __name__ == "__main__":
    main()
