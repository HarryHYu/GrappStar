#!/usr/bin/env python3
"""KEDGE soil pull test and bench torque log (procedure: kedge/DART_PULLTEST.md).

Pull test: the anchored dart is pulled along the tether in force steps; at every step the script logs the force (HX711
load cell on a USB serial board, or a luggage scale typed in), the tail movement (typed from a tape against a peg) and,
when the dart is reachable on WiFi, its pitch and roll from http://192.168.4.1/status (movement onset). Then it plots
force against movement with the predicted holding (out31/phys31/anchor.json, else out30) for that soil and line angle.

  python3 dart_pulltest.py pull --soil "moist loam" --rover 3 --serial /dev/ttyUSB0     # HX711 on serial
  python3 dart_pulltest.py pull --soil "dry sand" --rover 2 --manual                     # luggage scale
  python3 dart_pulltest.py plot pull_moist_loam_20261108_1012.csv
  python3 dart_pulltest.py torque --what worm_backdrive                                   # bench tests B1 to B4
  python3 dart_pulltest.py torque --what drill_stall

Bench torque results go into kedge/bench31.json (read by dart_model.py: motor stall torques and the worm holding
strength replace the assumptions) so `python dart_phys31.py` reruns the physics on measured numbers.
Force sign: tension in the line, newtons. Movement: tail displacement toward the puller, mm.
Safety: stand beside the line, never in line with it; gloves; stop at the test cap (default 50 N).
"""
import argparse, csv, json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.join(HERE, "bench31.json")
G = 9.81


def predicted(soil, rover):
    for d in ("out31", "out30"):
        f = os.path.join(HERE, d, "phys31", "anchor.json")
        if os.path.exists(f):
            R = json.load(open(f))
            rows = R["holding_limit"].get(soil)
            if rows:
                best = min(rows, key=lambda r: abs(r["rover_m"] - rover))
                return dict(design=best["F_design_N"], upper=best["F_upper_translation_N"], no_foot=best["F_without_foot_N"],
                            worms_never_slip=best["F_if_worms_never_slip_N"], src=f, rover_m=best["rover_m"])
    return None


class Dart:
    """Optional: the dart's own status page for pitch and roll (movement onset)."""
    def __init__(self, host="192.168.4.1"):
        self.url = f"http://{host}/status"

    def attitude(self):
        try:
            import urllib.request
            j = json.loads(urllib.request.urlopen(self.url, timeout=0.6).read())
            return float(j.get("pitch", "nan")), float(j.get("roll", "nan"))
        except Exception:
            return float("nan"), float("nan")


class Scale:
    """HX711 over USB serial: the board prints one number per line (newtons, or grams with --grams)."""
    def __init__(self, port, baud=115200, grams=False):
        import serial                                  # pip install pyserial
        self.s = serial.Serial(port, baud, timeout=0.3); self.grams = grams

    def read(self, n=10):
        v = []
        t0 = time.time()
        while len(v) < n and time.time() - t0 < 3:
            line = self.s.readline().decode(errors="ignore").strip()
            try:
                v.append(float(line.split()[-1]))
            except (ValueError, IndexError):
                pass
        if not v:
            return float("nan")
        v.sort(); x = v[len(v) // 2]
        return x / 1000 * G if self.grams else x


def ask(prompt, default=None):
    s = input(prompt).strip()
    if not s and default is not None:
        return default
    try:
        return float(s)
    except ValueError:
        return float("nan")


def pull(a):
    pred = predicted(a.soil, a.rover)
    if pred:
        print(f"Predicted for {a.soil}, rover {pred['rover_m']} m: design {pred['design']} N, upper {pred['upper']} N, "
              f"without the foot {pred['no_foot']} N, if the worms never slipped {pred['worms_never_slip']} N  ({pred['src']})")
    scale = None if a.manual else Scale(a.serial, grams=a.grams)
    dart = Dart(a.dart) if not a.no_dart else None
    stamp = time.strftime("%Y%m%d_%H%M")
    fn = a.out or f"pull_{a.soil.replace(' ', '_')}_{stamp}.csv"
    meta = dict(soil=a.soil, rover_m=a.rover, line_angle_deg=round(math.degrees(math.atan2(a.exit_h - 0.021, a.rover)), 1),
                moisture=a.moisture, auger_depth_note=a.note, predicted=pred, start=stamp)
    f = open(fn, "w", newline=""); w = csv.writer(f)
    f.write("# " + json.dumps(meta) + "\n")
    w.writerow(["t_s", "target_N", "force_N", "move_mm", "pitch_deg", "roll_deg", "note"])
    p0, r0 = dart.attitude() if dart else (float("nan"), float("nan"))
    t0 = time.time(); F = a.start; rows = []
    print("At each step: pull until the reading holds at the target, keep it there for the hold time, then type the tape reading.")
    try:
        while F <= a.cap + 1e-9:
            input(f"\nStep {F:.0f} N: bring the force to {F:.0f} N and hold. Press Enter when steady.")
            t1 = time.time(); fs = []
            while time.time() - t1 < a.hold:
                fs.append(scale.read(5) if scale else float("nan"))
                time.sleep(0.2)
            fmeas = (sorted(x for x in fs if x == x)[len(fs) // 2] if any(x == x for x in fs) else float("nan")) if scale else ask("  luggage scale reading (kg): ") * G
            mv = ask("  tail movement from the peg (mm, Enter = 0): ", 0.0)
            p, r = dart.attitude() if dart else (float("nan"), float("nan"))
            note = input("  note (Enter = none, 'x' = it let go): ").strip()
            row = [round(time.time() - t0, 1), F, round(fmeas, 2), mv, round(p, 1), round(r, 1), note]
            w.writerow(row); f.flush(); rows.append(row)
            dp = abs(p - p0) if p == p and p0 == p0 else float("nan")
            print(f"  logged {fmeas:.1f} N, {mv:.1f} mm, pitch change {dp:.1f} deg")
            if note.lower().startswith("x") or mv > a.max_move or (dp == dp and dp > a.max_pitch):
                print("  STOP: the anchor is moving (let go, movement or pitch limit). Ease off the line.")
                break
            F += a.step
    except KeyboardInterrupt:
        print("\nstopped by the operator")
    f.close()
    print(f"saved {fn}")
    plot(fn)


def load(fn):
    meta = json.loads(open(fn).readline()[2:])
    rows = list(csv.DictReader(l for l in open(fn) if not l.startswith("#")))
    return meta, rows


def plot(fn):
    meta, rows = load(fn)
    F = [float(r["force_N"]) for r in rows]; X = [float(r["move_mm"]) for r in rows]
    ok = [(x, f) for x, f in zip(X, F) if f == f]
    peak = max((f for _, f in ok), default=float("nan"))
    f2 = next((f for x, f in ok if x >= 2), float("nan")); f5 = next((f for x, f in ok if x >= 5), float("nan"))
    summ = dict(file=fn, soil=meta["soil"], rover_m=meta["rover_m"], peak_N=peak, force_at_2mm_N=f2, force_at_5mm_N=f5, predicted=meta.get("predicted"))
    json.dump(summ, open(fn.replace(".csv", "_summary.json"), "w"), indent=1)
    print(json.dumps(summ, indent=1))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        plt.rcParams.update({"font.family": "Liberation Sans", "font.size": 8})
        fig, ax = plt.subplots(figsize=(6, 3.6))
        ax.plot([x for x, _ in ok], [f for _, f in ok], "k-o", ms=3, label="measured")
        pr = meta.get("predicted")
        if pr:
            for k, ls in (("design", "--"), ("upper", ":"), ("no_foot", "-.")):
                ax.axhline(pr[k], color="k", ls=ls, lw=0.7, label=f"predicted {k.replace('_', ' ')} {pr[k]} N")
        ax.set_xlabel("tail movement (mm)"); ax.set_ylabel("line force (N)"); ax.grid(lw=0.3); ax.legend(fontsize=7)
        ax.set_title(f"Pull test, {meta['soil']}, rover {meta['rover_m']} m, line {meta['line_angle_deg']} deg")
        fig.tight_layout(); fig.savefig(fn.replace(".csv", ".png"), dpi=150)
        print("plot", fn.replace(".csv", ".png"))
    except Exception as e:
        print("plot skipped:", e)


TESTS = {
    "worm_stall": ("B1 worm N20 stall torque (kick or deploy), powered at the pack voltage", "lever_mm", "kg at stall", "worm_stall_Nm"),
    "worm_backdrive": ("B2 worm gearbox holding strength, UNPOWERED: load the lever until it slips or clicks", "lever_mm", "kg when it slipped", "worm_hold_Nm"),
    "drill_stall": ("B3 drill motor stall torque, powered", "lever_mm", "kg at stall", "drill_stall_Nm"),
    "foot_stall": ("B4 foot motor stall torque, powered", "lever_mm", "kg at stall", "foot_stall_Nm"),
}


def torque(a):
    title, _, unit, key = TESTS[a.what]
    print(title)
    L = ask("lever length, shaft centre to the scale hook (mm): ")
    V = ask("pack voltage at the test (V, Enter = 7.4): ", 7.4)
    vals = []
    for i in range(3):
        vals.append(ask(f"  try {i + 1}: {unit}: ") * G * L / 1000)
    vals = [v for v in vals if v == v]
    T = sorted(vals)[len(vals) // 2]
    B = json.load(open(BENCH)) if os.path.exists(BENCH) else {}
    B[key] = dict(Nm=round(T, 3), V=V, tries_Nm=[round(v, 3) for v in vals], lever_mm=L, date=time.strftime("%Y-%m-%d %H:%M"))
    json.dump(B, open(BENCH, "w"), indent=1)
    print(f"{key} = {T:.3f} N m (median of {len(vals)}), saved to {BENCH}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("pull")
    p.add_argument("--soil", required=True, choices=["dry sand", "moist loam", "firm clay", "other"])
    p.add_argument("--rover", type=float, default=3.0, help="horizontal distance from the dart tail to where the line is pulled from (m)")
    p.add_argument("--exit-h", type=float, default=0.41, help="height of the pull point above the ground (m); 0.41 = the rover pusher")
    p.add_argument("--serial", default="/dev/ttyUSB0"); p.add_argument("--grams", action="store_true")
    p.add_argument("--manual", action="store_true", help="luggage scale, type the readings")
    p.add_argument("--dart", default="192.168.4.1"); p.add_argument("--no-dart", action="store_true")
    p.add_argument("--start", type=float, default=5.0); p.add_argument("--step", type=float, default=5.0)
    p.add_argument("--cap", type=float, default=50.0); p.add_argument("--hold", type=float, default=10.0)
    p.add_argument("--max-move", type=float, default=15.0); p.add_argument("--max-pitch", type=float, default=5.0)
    p.add_argument("--moisture", default="", help="squeeze test: dry crumbles / moist holds / wet smears")
    p.add_argument("--note", default=""); p.add_argument("--out", default=None)
    q = sp.add_parser("plot"); q.add_argument("csv")
    t = sp.add_parser("torque"); t.add_argument("--what", required=True, choices=list(TESTS))
    a = ap.parse_args()
    {"pull": pull, "plot": lambda a: plot(a.csv), "torque": torque}[a.cmd](a)


if __name__ == "__main__":
    main()
