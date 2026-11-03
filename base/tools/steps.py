"""Assembly steps page: python -m tools.steps OUT [--only demo] [--shots]

Writes OUT/assembly.html (standalone, offline) and OUT/assembly_body.html (body fragment for publishing). Steps come
from asm.steps ({title, text, parts, subsystem}); a part with Part.step = k joins step k (1-based) of that list;
parts in no step get one auto step per subsystem. Each step shows the parts fitted so far, the new ones highlighted
and flying in from an exploded offset, with a bill of parts. --shots writes a few headless screenshots to OUT/shots/.
"""
from __future__ import annotations

import argparse
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.assembly import Assembly  # noqa: E402
from tools.params import load_params  # noqa: E402
from tools import timeline as TL  # noqa: E402
from tools.render import build_data, write_pages, run_shots  # noqa: E402


def plan_steps(asm: Assembly) -> tuple[list[dict], list[str]]:
    """Ordered steps covering every part exactly once. Returns (steps, warnings)."""
    warn = []
    steps, seen = [], set()
    for st in asm.steps:
        parts = []
        for n in st.get("parts", []):
            if n not in asm.parts:
                warn.append(f"step '{st.get('title', '')}': unknown part {n}")
            elif n in seen:
                warn.append(f"step '{st.get('title', '')}': part {n} already in an earlier step")
            else:
                parts.append(n)
                seen.add(n)
        steps.append(dict(title=st.get("title", "Step"), text=st.get("text", ""), parts=parts,
                          subsystem=st.get("subsystem", ""), auto=False))
    extra: dict[int, list[str]] = {}
    for n, p in asm.parts.items():
        if n in seen or p.step is None:
            continue
        k = int(p.step)
        if 1 <= k <= len(steps):
            steps[k - 1]["parts"].append(n)
        else:
            extra.setdefault(k, []).append(n)
        seen.add(n)
    for k in sorted(extra):
        steps.append(dict(title=f"Step {k}", text="Parts numbered for this step.", parts=extra[k], subsystem="", auto=True))
    for sub in asm.modules:
        rest = [n for n, p in asm.parts.items() if n not in seen and p.subsystem == sub]
        if not rest:
            continue
        # fixed structure first, then moving parts, bought parts after the printed ones
        rest.sort(key=lambda n: (asm.parts[n].joint is not None, not asm.parts[n].printed))
        seen.update(rest)
        steps.append(dict(title=f"Fit the remaining {sub} parts", text="Parts without an explicit assembly step: "
                          + ", ".join(rest) + ".", parts=rest, subsystem=sub, auto=True))
    rest = [n for n in asm.parts if n not in seen]
    if rest:
        steps.append(dict(title="Fit the remaining parts", text="", parts=rest, subsystem="", auto=True))
    return [s for s in steps if s["parts"]], warn


def build(out: str, names=None, P=None) -> tuple[str, list[dict]]:
    P = P or load_params()
    asm = Assembly.load(P, names)
    os.makedirs(out, exist_ok=True)
    asm.export(out)
    steps, warn = plan_steps(asm)
    for w in warn:
        print("WARN", w)
    data = build_data(asm, out, [TL.state_at(P, 0.0)])
    data["steps"] = steps
    dst, _ = write_pages(out, "assembly", "assembly_template.html", data, ["viewer_core.js", "assembly_app.js"],
                         data["title"] + " assembly steps")
    return dst, steps


def shots(out: str, n: int, which=None) -> list[str]:
    """Headless screenshots of OUT/assembly.html: first, middle and last step (settled) plus one mid fly-in."""
    sd = os.path.join(out, "shots")
    os.makedirs(sd, exist_ok=True)
    url = "file://" + os.path.abspath(os.path.join(out, "assembly.html"))
    which = which or sorted({0, n // 2, n - 1})
    job, files = [], []
    # every shot is posed at a fixed animation fraction and frozen (no timing dependence on the renderer speed)
    for i in which:
        f = os.path.join(sd, f"assembly_step{i + 1:02d}.png")
        job.append(dict(file=f, ops=[["ASM_GO", i, True], ["ASM_FREEZE", 1.0]]))
        files.append(f)
    f = os.path.join(sd, f"assembly_step{n:02d}_flyin.png")
    job.append(dict(file=f, ops=[["ASM_GO", n - 1], ["ASM_FREEZE", 0.45]]))
    files.append(f)
    res = run_shots(dict(url=url, width=1400, height=880, ready="window.READY === true", shots=job))
    f = os.path.join(sd, "assembly_dark_phone.png")
    res2 = run_shots(dict(url=url, width=400, height=820, scale=2, color_scheme="dark", ready="window.READY === true",
                          shots=[dict(file=f, ops=[["ASM_GO", n - 1, True], ["ASM_FREEZE", 1.0]])]))
    files.append(f)
    for e in res.get("errors", []) + res2.get("errors", []):
        print("page:", e)
    return files


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--shots", action="store_true", help="also write headless screenshots to OUT/shots/")
    a = ap.parse_args(argv)
    names = [s for s in a.only.split(",") if s] or None
    dst, steps = build(a.out, names)
    body = os.path.join(a.out, "assembly_body.html")
    print(f"{dst} {os.path.getsize(dst) / 1e6:.2f} MB, {body} {os.path.getsize(body) / 1e6:.2f} MB")
    if a.shots:
        for f in shots(a.out, len(steps)):
            print(f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
