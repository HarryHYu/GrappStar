"""Rover drill template: 1:1 PDF of the deck outline and every rover hole, plus a CSV table.

python cad/parts/_frame_tests/drill_template.py OUT    (from base/) -> OUT/rover_drill_template.pdf, .csv
The PDF tiles the deck over A4 pages at 1:1 with a 100 mm check bar on every page. Plain black Liberation Sans.
"""
import csv
import math
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, BASE)


def main(out):
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from tools.params import load_params
    from cad.parts import frame
    font = "Helvetica"
    for path in ("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
                 "/usr/share/fonts/liberation/LiberationSans-Regular.ttf"):
        if os.path.exists(path):
            pdfmetrics.registerFont(TTFont("LiberationSans", path))
            font = "LiberationSans"
            break
    P = load_params()
    info = frame.build(P)["info"]
    x0, x1, y0, y1 = info["deck_rect"]
    holes = info["rover_holes"]
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "rover_drill_template.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["hole", "x_mm_from_deck_rear_edge", "y_mm_from_deck_right_edge", "drill_mm"])
        for i, (hx, hy) in enumerate(holes, 1):
            w.writerow([i, round(hx - x0, 1), round(hy - y0, 1), 3.4])
    pw, ph = A4
    m = 12 * mm
    cw, ch = pw - 2 * m, ph - 2 * m - 18 * mm
    nx = max(1, math.ceil((x1 - x0) * mm / cw))
    ny = max(1, math.ceil((y1 - y0) * mm / ch))
    c = canvas.Canvas(os.path.join(out, "rover_drill_template.pdf"), pagesize=A4)
    for j in range(ny):
        for i in range(nx):
            ox, oy = x0 + i * cw / mm, y0 + j * ch / mm          # deck mm at the page cell's lower-left
            c.setFont(font, 9)
            c.drawString(m, ph - m, f"GrappStar base: rover drill template, 1:1. Page {j * nx + i + 1} of {nx * ny} "
                                    f"(column {i + 1}, row {j + 1}). Print at 100 %, check the 100 mm bar.")
            c.drawString(m, ph - m - 11, "Drill 3.4 mm at every cross. Front of the base (launch direction) = right of the page.")
            c.setLineWidth(0.6)
            c.line(m, m + 4 * mm, m + 100 * mm, m + 4 * mm)
            c.drawString(m, m + 6 * mm, "100 mm")

            def P2(x, y):
                return m + (x - ox) * mm, m + 10 * mm + (y - oy) * mm
            c.saveState()
            p = c.beginPath()
            p.rect(m, m + 10 * mm, cw, ch)
            c.clipPath(p, stroke=0)
            c.rect(*P2(x0, y0), (x1 - x0) * mm, (y1 - y0) * mm)
            for k, (hx, hy) in enumerate(holes, 1):
                X, Y = P2(hx, hy)
                c.line(X - 4 * mm, Y, X + 4 * mm, Y)
                c.line(X, Y - 4 * mm, X, Y + 4 * mm)
                c.circle(X, Y, 1.7 * mm)
                c.drawString(X + 2.5 * mm, Y + 2.5 * mm, str(k))
            c.restoreState()
            c.showPage()
    c.save()
    print(f"{len(holes)} holes, deck {x1 - x0:.0f} x {y1 - y0:.0f} mm, {nx * ny} pages -> {out}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out_frame")
