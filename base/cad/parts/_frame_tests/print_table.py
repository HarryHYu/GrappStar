"""Markdown table of every printed frame part in its print orientation: material, bed footprint and height, est.
mass (core.Part.mass_g), notes. python cad/parts/_frame_tests/print_table.py   (from base/)"""
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, BASE)


def main():
    from tools.params import load_params
    from cad.parts import frame
    from cad.lib import prim
    P = load_params()
    rows, tot = [], 0.0
    for p in frame.parts(P):
        if not p.printed:
            continue
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        m = p.mass_g() or 0.0
        tot += m
        rows.append(f"| {p.name} | {p.material} | {hi[0] - lo[0]:.0f} x {hi[1] - lo[1]:.0f} x {hi[2] - lo[2]:.0f} | "
                    f"{m:.0f} | {p.infill:.0%} | {p.notes} |")
    print("| part | material | print bbox x y z (mm) | mass g | infill | notes |")
    print("|---|---|---|---|---|---|")
    print("\n".join(rows))
    print(f"\nTotal printed frame mass about {tot / 1000:.2f} kg.")
    bought = [p for p in frame.parts(P) if not p.printed]
    print("\nBought / modelled envelopes: " + ", ".join(f"{p.name} ({p.bought_ref})" for p in bought))


if __name__ == "__main__":
    main()
