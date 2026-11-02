#!/usr/bin/env python3
"""Soil pull test: how hard does the anchored KEDGE hold? Uses the ground station's own tow (no extra hardware).
The tension limit steps up slowly; the script logs tension and line wound in at 10 Hz and stops when the dart lets go
(line keeps coming in while tension falls) or at the top limit. Then it plots force against displacement and puts
the Broms predictions from out30/physics.json on the same chart.

  python3 pulltest.py --soil "moist loam" --max 50            # ground station on 192.168.5.1, dart anchored (50 N is the firmware cap)
  python3 pulltest.py --mock                                    # dry run against mock_gs.py (start it first)
  python3 pulltest.py --plot pull_moist_loam_*.csv              # re-plot a saved run

Procedure (one person, about 10 minutes per test):
 1. Fire or place the dart on the test ground, at least 3 m from the rover. Let it anchor (auto, or 'a' on the dart).
 2. Measure the line from the exit eye to the dart tail with a tape, type it in when asked (sets the line estimate).
 3. Chock the rover wheels or tie it to something solid: the test must pull the dart, not the rover.
 4. Run this script. Watch the dart; stop with Ctrl-C (sends STOP) if anything looks wrong.
 5. After it ends, measure how far the dart moved (tape from a peg you put next to the tail before the test).
 6. Repeat 3 times per soil. Note soil type, moisture (squeeze test), and the auger depth the dart reported.
The firmware caps the tow at 50 N (reel gears and grub screws), so in firm clay (predicted about 124 N) the dart will not let go:
the test then shows 'held at least 50 N'. For more, pull by hand with a luggage scale on the tether.
Safety: stand to the side of the line, never in line with it. Dyneema stores little energy but the dart can jump.
"""
import argparse, csv, json, os, sys, time
sys.path.insert(0, os.path.dirname(__file__))
from gs_client import GroundStation, Refused

HERE = os.path.dirname(os.path.abspath(__file__))
PHYS = os.path.join(HERE, "..", "..", "out30", "physics.json")


def broms():
    try:
        d = json.load(open(PHYS))["anchoring"]["by_soil"]
        return {k: v["total_N"] for k, v in d.items()}
    except Exception:
        return {"dry sand": 7.7, "moist loam": 48.5, "firm clay": 124.4}


def run(g, soil, f_start, f_step, f_max, hold_s, out):
    rows = []; t0 = time.time()
    line0 = float(input("Line from the exit eye to the dart tail (m): ") or 0)
    if line0 > 0:
        g.set_line(line0)
    st = g.status(); base = st.get("line_m", line0)
    peak = 0.0; limit = f_start
    try:
        while limit <= f_max + 1e-6:
            g.limit(limit)
            if g.status().get("state") != "tow":
                try: g.tow(0)
                except Refused as e: print("\nrefused:", e); raise StopIteration
            t_l = time.time()
            while time.time() - t_l < hold_s:
                s = g.status(); t = time.time() - t0
                disp = (base - s.get("line_m", base)) * 1000.0
                F = s.get("tension_N", 0.0)
                rows.append((round(t, 2), limit, round(F, 2), round(disp, 1), s.get("state"), s.get("tension_src", "")))
                peak = max(peak, F)
                print("\r t %6.1f s  limit %5.1f N  tension %6.1f N  wound in %7.1f mm  %s   " % (t, limit, F, disp, s.get("state")), end="", flush=True)
                # let go: tension drops below half the peak while the line keeps coming in
                if peak > 5 and F < 0.5 * peak and disp > 30:
                    print("\nanchor let go at %.1f N" % peak); raise StopIteration
                time.sleep(0.1)
            limit += f_step
    except (KeyboardInterrupt, StopIteration):
        pass
    finally:
        try: g.stop()
        except Exception: pass
    with open(out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", "limit_N", "tension_N", "wound_in_mm", "state", "source"]); w.writerows(rows)
    print("\npeak %.1f N, saved %s" % (peak, out))
    return out


def plot(files, soil):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5))
    for fn in files:
        r = list(csv.DictReader(open(fn)))
        ax.plot([float(x["wound_in_mm"]) for x in r], [float(x["tension_N"]) for x in r], lw=1.4, label=os.path.basename(fn))
    for k, v in broms().items():
        ax.axhline(v, ls="--", lw=0.8, color="k"); ax.text(1, v + 1, "Broms prediction, %s: %.0f N" % (k, v), fontsize=8)
    ax.set_xlabel("line wound in (mm): dart movement plus line stretch"); ax.set_ylabel("line tension (N)")
    ax.set_title("KEDGE pull test" + (", " + soil if soil else "")); ax.grid(True, lw=0.3); ax.legend(fontsize=8)
    out = os.path.splitext(files[0])[0] + ".png"; fig.savefig(out, dpi=140, bbox_inches="tight"); print("plot", out)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--host", default="192.168.5.1"); a.add_argument("--mock", action="store_true")
    a.add_argument("--soil", default=""); a.add_argument("--start", type=float, default=5); a.add_argument("--step", type=float, default=5)
    a.add_argument("--max", type=float, default=50); a.add_argument("--hold", type=float, default=4.0)
    a.add_argument("--plot", nargs="*")
    o = a.parse_args()
    if o.plot:
        plot(o.plot, o.soil); return
    g = GroundStation("127.0.0.1:8781" if o.mock else o.host)
    print("ground station:", g.status().get("state"))
    out = "pull_%s_%s.csv" % ((o.soil or "soil").replace(" ", "_"), time.strftime("%Y%m%d_%H%M%S"))
    run(g, o.soil, o.start, o.step, o.max, o.hold, out)
    plot([out], o.soil)


if __name__ == "__main__":
    main()
