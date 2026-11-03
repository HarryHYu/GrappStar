#!/usr/bin/env python3
"""Build the GrappStar base station site (coordinator).

    python3 scripts/make_site.py [--out base/site] [--branch main] [--checks checks.txt]

Writes OUT/index_body.html (artifact page body: title, style, content; the publisher adds the document skeleton),
OUT/index.html (standalone copy for the repo), and copies the generated pages it finds:
  base/out/base/sim.html -> OUT/sim.html, base/out/base/assembly.html -> OUT/assembly.html,
  base/out/base/shots/*.png (first one) -> OUT/hero.png.
Numbers come from base/spec/params.py, base/out/physics/*.json and base/out/base/audit.json when present.
No em dashes anywhere in the text.
"""
from __future__ import annotations

import argparse
import base64
import glob
import html
import importlib.util
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = "https://github.com/HarryHYu/GrappStar"


def load_params():
    p = os.path.join(ROOT, "base", "spec", "params.py")
    spec = importlib.util.spec_from_file_location("params", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def jload(path, default=None):
    try:
        with open(os.path.join(ROOT, path)) as f:
            return json.load(f)
    except Exception:
        return default


def get(d, *keys, default=None):
    for k in keys:
        if isinstance(d, dict) and k in d:
            d = d[k]
        else:
            return default
    return d


def esc(s):
    return html.escape(str(s), quote=True)


def gh(path, branch):
    kind = "tree" if not os.path.splitext(path)[1] else "blob"
    return f"{REPO}/{kind}/{branch}/{path}"


def git_head():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "log", "-1", "--format=%h %cd", "--date=format:%d %b %Y %H:%M"],
                                       text=True).strip()
    except Exception:
        return ""


def measure_rows():
    """Rows for the measure list: from docs/build/MEASURE_LIST.md table rows if present, else from params."""
    rows = []
    md = os.path.join(ROOT, "docs", "build", "MEASURE_LIST.md")
    if os.path.exists(md):
        for line in open(md, encoding="utf-8"):
            line = line.strip()
            if line.startswith("|") and not set(line) <= set("|- :"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                if len(cells) >= 2 and cells[0].lower() not in ("#", "id", "item", "what"):
                    rows.append(cells[:4])
    if not rows:
        rows = [["Wiper motor", "shaft diameter, flats, end thread, mount holes, housing size, speed and stall current",
                 "params.WIPER", "motor adapter, coupler"],
                ["Springs", "hang a full 1 L bottle on one spring, length before and after", "params.SPRING",
                 "spring anchor holes"]]
    return rows[:40]


def build(out, branch, checks_file):
    P = load_params()
    phys_launch = jload("base/out/physics/launch.json", {}) or {}
    ball = jload("base/out/ballistics.json", {}) or {}
    audit = jload("base/out/base/audit.json", {}) or {}
    os.makedirs(out, exist_ok=True)

    # key numbers (datasheet)
    spring = getattr(P, "SPRING", {}) or {}
    launch = getattr(P, "LAUNCH", None) or {}
    reel = getattr(P, "REEL", None) or {}
    aim = getattr(P, "AIM", None) or {}
    v = get(phys_launch, "v_muzzle") or get(phys_launch, "baseline", "v_muzzle")
    e = get(phys_launch, "energy_stored_j") or get(phys_launch, "baseline", "energy_stored_j")
    rng = None
    try:
        rows = ball.get("table", [])
        rng = max(r["range_m"][0] for r in rows) if rows else None
    except Exception:
        rng = None
    data = [
        ("Range (2 springs, 0.5 N payout brake)", f"{rng:.1f} m" if rng else "4 to 5 m"),
        ("Muzzle speed", f"{v:.1f} m/s" if v else "about 7 m/s"),
        ("Spring energy stored", f"{e:.1f} J" if e else "about 13 J"),
        ("Springs", f"{spring.get('qty', 2)} x {spring.get('name', 'Century 16 x 165.1 x 1.4')}"),
        ("Pitch range", f"{get(aim, 'pitch_min', default='6')} to {get(aim, 'pitch_max', default='47')} deg"),
        ("Capstan wraps", str(get(reel, "wraps", default=7))),
        ("Tow limit (firmware default)", f"{get(reel, 'tow_limit_n', default=50)} N"),
        ("Dart", f"KEDGE v30, {getattr(P, 'DART', {}).get('mass_kg', 0.279) * 1000:.0f} g with ballast"),
        ("Bed", " x ".join(str(int(x)) for x in getattr(P, "BED", (256, 256, 260))) + " mm (Bambu X2D)"),
    ]

    checks = []
    if checks_file and os.path.exists(checks_file):
        for line in open(checks_file):
            line = line.strip()
            if line.startswith(("PASS", "FAIL", "SKIP")):
                st, _, rest = line.partition(" ")
                name, _, detail = rest.strip().partition(":")
                checks.append((st, name.strip(), detail.strip()))
    audit_state = audit.get("result") or audit.get("status") or ("CLEAN" if audit.get("clean") else "")

    # copy generated pages
    files = {}
    for src, dst in [("base/out/base/sim.html", "sim.html"), ("base/out/base/assembly.html", "assembly.html")]:
        s = os.path.join(ROOT, src)
        if os.path.exists(s):
            shutil.copy(s, os.path.join(out, dst))
            files[dst] = True
    shots = sorted(glob.glob(os.path.join(ROOT, "base/out/base/shots/sim_t*.png")))
    hero = ""
    if shots:
        shutil.copy(shots[len(shots) // 2], os.path.join(out, "hero.png"))
        files["hero.png"] = True
        hero = "hero.png"

    steps = [
        ("Measure", "the wiper motor and one spring (list below). Put the numbers in base/spec/params.py and rerun the "
                    "base pipeline: the motor adapter, coupler and spring anchors regenerate."),
        ("Buy", "8 to 10 bearings 608ZZ (8 x 22 x 7). Optional: a small neodymium disc magnet for the angle sensor."),
        ("Print", "the base print pack (plates for the X2D) and the dart v30 pack."),
        ("Build", "the base in the order of the assembly steps, then the dart."),
        ("Wire", "with the battery unplugged, following SYSTEM_ELECTRICAL and the two WIRING files."),
        ("Flash", "the base ESP32-S3 and the dart XIAO; install the Jetson software."),
        ("Test", "in the stages of TEST_PLAN, bench first. Never fire without the tether attached."),
    ]
    mission = ["Aim: the wiper turns backward and the crank-rocker lifts the rail; a self-locking worm holds it.",
               "Fire: the wiper turns forward one and a half turns: the big gear cocks the sled, lets go at the relieved "
               "tooth and resets. The take-up winds the slack while cocking.",
               "Flight: the line pays off the storage reel against a 0.5 N slip clutch; the dart films its flight.",
               "Landing: the Jetson scores the landing zone from the nose camera and the flight log.",
               "Anchor: the dart plants its foot and screws in its two drills.",
               "Tidy and wrap: the base winds the slack, then the drum turns the wrap cup round the capstan 7 times "
               "and engages REEL.",
               "Tow: the capstan winches the rover toward the dart; sprag #2 holds the load in any state.",
               "Release and retrieve: the dart unscrews, the capstan reels it in, the drum unwraps with take-up "
               "running, and the take-up pulls the dart onto the sled until it is seated."]
    links = [
        ("Build guide (PDF)", "docs/build/GrappStar_Build_Guide.pdf"),
        ("Measure list", "docs/build/MEASURE_LIST.md"),
        ("Test plan", "docs/build/TEST_PLAN.md"),
        ("Electrical system", "docs/build/SYSTEM_ELECTRICAL.md"),
        ("Base spec", "base/spec/BASE_SPEC.md"),
        ("Decisions log", "base/spec/DECISIONS.md"),
        ("Every shared number", "base/spec/params.py"),
        ("Base print pack", "base/pack"),
        ("Base firmware", "firmware/base"),
        ("Dart firmware", "firmware/dart"),
        ("Jetson software", "jetson"),
        ("Dart v30 print pack", "kedge/dist_v30"),
    ]

    def li(items):
        return "".join(f"<li>{esc(x)}</li>" for x in items)

    hero_html = (f'<figure class="hero"><img src="{hero}" alt="The base station in the mission sim"></figure>' if hero
                 else '<figure class="hero empty"><p>The mission sim renders here after the base pipeline runs.</p></figure>')
    nav = []
    if files.get("sim.html"):
        nav.append('<a class="go" href="sim.html">Open the mission sim</a>')
    if files.get("assembly.html"):
        nav.append('<a class="go alt" href="assembly.html">Assembly steps</a>')
    status = f"Built {esc(git_head())}" + (f" &middot; collision audit {esc(audit_state)}" if audit_state else "")

    body = f"""<title>GrappStar Base Station</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
/* Layout: one datasheet column, sim figure up top, ordered work list, then reference tables. */
:root {{
  --paper: #f5f6f4; --ink: #1d2327; --muted: #5d676d; --rule: #d3d8d6; --panel: #eaedeb;
  --accent: #d2561c; --ok: #2f7d4f; --bad: #b3261e;
  --display: "IBM Plex Sans Condensed", "Arial Narrow", Arial, sans-serif;
  --body: "IBM Plex Sans", Arial, sans-serif; --mono: "IBM Plex Mono", ui-monospace, Menlo, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --paper: #141819; --ink: #e4e8e6; --muted: #9aa5a9; --rule: #2c3437; --panel: #1c2224;
  --accent: #f0794a; --ok: #6cc28f; --bad: #f2867f; color-scheme: dark; }} }}
:root[data-theme="dark"] {{ --paper: #141819; --ink: #e4e8e6; --muted: #9aa5a9; --rule: #2c3437; --panel: #1c2224;
  --accent: #f0794a; --ok: #6cc28f; --bad: #f2867f; color-scheme: dark; }}
body {{ background: var(--paper); color: var(--ink); font: 16px/1.55 var(--body); }}
.wrap {{ max-width: 1040px; margin: 0 auto; padding-block: 28px 64px;
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px));
  display: grid; gap: 40px; }}
header h1 {{ font: 600 clamp(2rem, 5vw, 3.1rem)/1.05 var(--display); letter-spacing: 0.01em; margin: 0; text-wrap: balance; }}
header p {{ margin: 10px 0 0; max-width: 64ch; color: var(--muted); }}
.status {{ font: 0.8rem var(--mono); color: var(--muted); margin-top: 14px; }}
.hero {{ margin: 0; border: 1px solid var(--rule); background: var(--panel); }}
.hero img {{ display: block; width: 100%; max-width: 100%; height: auto; }}
.hero.empty {{ min-height: 180px; display: grid; place-items: center; color: var(--muted); }}
.actions {{ display: flex; flex-wrap: wrap; gap: 12px; }}
a.go {{ display: inline-block; padding: 10px 18px; background: var(--accent); color: #fff; text-decoration: none;
  font: 600 0.95rem var(--display); letter-spacing: 0.03em; border-radius: 2px; }}
a.go.alt {{ background: transparent; color: var(--ink); border: 1px solid var(--ink); }}
a.go:focus-visible, a:focus-visible {{ outline: 2px solid var(--accent); outline-offset: 2px; }}
h2 {{ font: 600 1.35rem/1.2 var(--display); margin: 0 0 14px; letter-spacing: 0.02em; text-wrap: balance; }}
h2 small {{ font: 0.75rem var(--mono); color: var(--muted); letter-spacing: 0.06em; text-transform: uppercase; margin-left: 8px; }}
ol.work {{ margin: 0; padding: 0; list-style: none; counter-reset: s; display: grid; gap: 10px; }}
ol.work li {{ counter-increment: s; display: grid; grid-template-columns: 2.2rem 1fr; gap: 10px; min-width: 0; }}
ol.work li::before {{ content: counter(s); font: 500 0.9rem var(--mono); color: var(--accent); padding-top: 2px; }}
ol.work b {{ font-weight: 600; }}
.scroll {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; font-size: 0.93rem; }}
th, td {{ text-align: left; vertical-align: top; padding: 8px 10px; border-bottom: 1px solid var(--rule); }}
th {{ font: 600 0.78rem var(--display); letter-spacing: 0.06em; text-transform: uppercase; color: var(--muted); }}
td.num {{ font-family: var(--mono); font-variant-numeric: tabular-nums; white-space: nowrap; }}
.pass {{ color: var(--ok); font: 600 0.85rem var(--mono); }} .fail {{ color: var(--bad); font: 600 0.85rem var(--mono); }}
ol.seq {{ margin: 0; padding-left: 1.4rem; display: grid; gap: 8px; max-width: 70ch; }}
ul.files {{ list-style: none; margin: 0; padding: 0; display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr)); gap: 8px 20px; }}
ul.files a {{ color: var(--ink); text-decoration-color: var(--accent); text-underline-offset: 3px; }}
.warn {{ border-left: 3px solid var(--accent); padding: 4px 0 4px 14px; max-width: 70ch; }}
@media (prefers-reduced-motion: reduce) {{ * {{ scroll-behavior: auto; }} }}
</style>
<div class="wrap">
<header>
  <h1>GrappStar Base Station</h1>
  <p>One 12 V wiper motor launches the KEDGE anchor dart, aims it and winches the rover out against it. A small N20
  selector drum swaps the motor between the launcher and the capstan reel, in a fixed physical order:
  LAUNCH, NEUTRAL, WRAP, REEL.</p>
  <div class="status">{status}</div>
</header>
<section>{hero_html}<div class="actions" style="margin-top:14px">{''.join(nav)}</div></section>
<section><h2>What to do next <small>in order</small></h2>
<ol class="work">{''.join(f'<li><span><b>{esc(a)}</b> {esc(b)}</span></li>' for a, b in steps)}</ol></section>
<section><h2>Measure these first <small>assumed until measured</small></h2><div class="scroll"><table>
<thead><tr><th>Item</th><th>How</th><th>Changes</th><th>Reprint</th></tr></thead><tbody>
{''.join('<tr>' + ''.join(f'<td>{esc(c)}</td>' for c in (r + ['', '', '', ''])[:4]) + '</tr>' for r in measure_rows())}
</tbody></table></div></section>
<section><h2>Datasheet</h2><div class="scroll"><table><tbody>
{''.join(f'<tr><td>{esc(k)}</td><td class="num">{esc(val)}</td></tr>' for k, val in data)}
</tbody></table></div>
<p class="warn">Never fire without the tether attached: at these speeds the tether keeps the dart nose first.</p></section>
<section><h2>Mission <small>one full cycle</small></h2><ol class="seq">{li(mission)}</ol></section>
<section><h2>Checks</h2><div class="scroll"><table><thead><tr><th>Suite</th><th>Result</th><th>Detail</th></tr></thead><tbody>
{''.join(f'<tr><td>{esc(n)}</td><td class="{"pass" if s == "PASS" else "fail"}">{esc(s)}</td><td class="num">{esc(d)}</td></tr>' for s, n, d in checks) or '<tr><td colspan="3">Run scripts/check_all.sh and pass --checks.</td></tr>'}
</tbody></table></div></section>
<section><h2>Files <small>on GitHub</small></h2><ul class="files">
{''.join(f'<li><a href="{esc(gh(p, branch))}" target="_blank" rel="noopener">{esc(t)}</a></li>' for t, p in links)}
</ul></section>
</div>
"""
    assert "—" not in body, "em dash in the page"
    with open(os.path.join(out, "index_body.html"), "w", encoding="utf-8") as f:
        f.write(body)
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write("<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">"
                "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">"
                "</head><body style=\"margin:0\">\n" + body + "</body></html>\n")
    print("site:", out, "files:", ", ".join(["index_body.html", "index.html"] + sorted(files)))
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "base", "site"))
    ap.add_argument("--branch", default="base-v1")
    ap.add_argument("--checks", default="")
    a = ap.parse_args()
    build(a.out, a.branch, a.checks)
