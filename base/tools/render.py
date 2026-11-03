"""Audit renders: 24-angle sheet, section cuts and mission state views (headless Chromium + three.js).

python -m tools.render OUT [--only demo] [--state-index k] [--states load,cock,winch | all] [--quick]

Writes OUT/audit_sheet.png (8 azimuths x 3 elevations), OUT/sections.png (4 cuts each along x, y, z),
OUT/states.png (with --states) and the single shots in OUT/views/. The scene comes from the Assembly
(exported to OUT first) through tools/web/viewer_core.js; tools/web/shots.js drives the browser.

The shared helpers here (build_data, inline_page, serve, run_shots) are used by tools.sim and tools.steps.
"""
from __future__ import annotations

import argparse
import base64
import functools
import http.server
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.assembly import Assembly  # noqa: E402
from tools.params import load_params  # noqa: E402
from tools import timeline as TL  # noqa: E402

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
FONT_PATHS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"]
FONT_BOLD = ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
             "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"]

# sheet layout (tests read these)
SHEET_AZ = [0, 45, 90, 135, 180, 225, 270, 315]
SHEET_EL = [-20, 25, 60]
TILE = (420, 320)
SEC_TILE = (480, 360)
SEC_PER_AXIS = 4
TITLE_H = 44
LABEL_H = 0


# ---------------------------------------------------------------------------------------------- data
def _b64(a: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode("ascii")


def _quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (x, y, z, w)."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def encode_mesh(v: np.ndarray, f: np.ndarray) -> dict:
    """int16 vertices on the part's own bbox grid (better than 0.01 mm below 650 mm), uint16/uint32 indices."""
    v = np.asarray(v, np.float64)
    lo, hi = v.min(0), v.max(0)
    s = max(float((hi - lo).max()), 1e-6) / 65534.0
    o = lo + 32767 * s
    q = np.clip(np.round((v - o) / s), -32767, 32767).astype(np.int16)
    wide = len(v) > 65535
    return dict(q=[round(float(x), 6) for x in o] + [s], v=_b64(q), f=_b64(np.asarray(f, np.uint32 if wide else np.uint16)),
                wide=wide, nv=int(len(v)), nf=int(len(f)))


def load_meshes(asm: Assembly, out: str) -> dict:
    meshes = {}
    for n in asm.parts:
        fn = os.path.join(out, "meshes", f"{n}.npz")
        if os.path.exists(fn):
            d = np.load(fn)
            v, f = d["v"], d["f"]
        else:
            from cad.lib import prim
            v, f = prim.mesh_of(asm.parts[n].solid)
        meshes[n] = encode_mesh(v, f)
    return meshes


def encode_tracks(asm: Assembly, states: list[dict]):
    """Per-part world matrices from asm.tracks(states), grouped by rigid body: parts on one joint share one track
    (quaternion int16 + position float32 per sample) and keep a constant place matrix. Static parts use group 0
    (identity) with their world matrix as place. Returns (tracks dict, {part: (group, place 4x4)})."""
    T = asm.tracks(states)
    n = len(states)
    groups = [dict(m=np.eye(4).reshape(-1).tolist())]
    assign = {}
    by_joint: dict = {}
    for name, lst in T["parts"].items():
        mats = np.array(lst, np.float64).reshape(-1, 4, 4)
        if len(mats) == 1:
            assign[name] = (0, mats[0])
            continue
        part = asm.parts[name]
        place = np.asarray(part.place, np.float64)
        key = part.joint
        if key in by_joint:
            gi, G = by_joint[key]
            if np.abs(G @ place - mats).max() < 2e-3:
                assign[name] = (gi, place)
                continue
            key = None
        G = mats @ np.linalg.inv(place) if key is not None else mats
        pl = place if key is not None else np.eye(4)
        qs = np.zeros((n, 4))
        prev = None
        for i in range(n):
            q = _quat(G[i, :3, :3])
            if prev is not None and np.dot(q, prev) < 0:
                q = -q
            qs[i] = prev = q
        groups.append(dict(q=_b64(np.round(qs * 32767).astype(np.int16)), p=_b64(G[:, :3, 3].astype(np.float32))))
        gi = len(groups) - 1
        if key is not None:
            by_joint[key] = (gi, G)
        assign[name] = (gi, pl)
    tracks = dict(n=n, t=[round(float(x), 4) for x in T["t"]], groups=groups)
    return tracks, assign, T


def state_columns(states: list[dict]) -> dict:
    """Scalar state keys as columns (numbers rounded, strings kept) for the readout."""
    keys = []
    for S in states:
        for k, v in S.items():
            if k not in keys and isinstance(v, (int, float, str, bool)) and k != "phase":
                keys.append(k)
    cols = {}
    for k in keys:
        col = []
        for S in states:
            v = S.get(k)
            if isinstance(v, bool):
                v = int(v)
            elif isinstance(v, float):
                v = round(v, 3)
            col.append(v)
        cols[k] = col
    return cols


def ground_z(P):
    """Ground (deck) height in world z when the params say so: P.GROUND_Z, or P.LAYOUT["ground_z"] / ["deck_z"].
    None means the viewer puts the ground at the assembly bbox min z (the world origin sits on the main shaft
    axis, so the deck is below z = 0)."""
    for v in (getattr(P, "GROUND_Z", None),):
        if isinstance(v, (int, float)):
            return float(v)
    L = getattr(P, "LAYOUT", None)
    if isinstance(L, dict):
        for k in ("ground_z", "deck_z"):
            if isinstance(L.get(k), (int, float)):
                return float(L[k])
    return None


def build_data(asm: Assembly, out: str, states: list[dict], title: str = "") -> dict:
    meshes = load_meshes(asm, out)
    tracks, assign, T = encode_tracks(asm, states)
    parts = []
    for n, p in asm.parts.items():
        gi, place = assign[n]
        parts.append(dict(name=n, color=p.color, material=p.material, notes=p.notes, subsystem=p.subsystem, joint=p.joint,
                          bought_ref=p.bought_ref, printed=bool(p.printed), qty=p.qty,
                          mass_g=None if p.mass_g() is None else round(p.mass_g(), 1), step=p.step,
                          group=gi, place=np.round(np.asarray(place, np.float64), 6).reshape(-1).tolist()))
    return dict(title=title or "GrappStar " + " + ".join(asm.modules), subsystems=list(asm.modules), parts=parts,
                meshes=meshes, tracks=tracks, phase=T["phase"], states=state_columns(states), ground_z=ground_z(asm.P))


def data_js(data: dict) -> str:
    return "window.CAD_DATA=" + json.dumps(data, separators=(",", ":")) + ";\n"


def inline_page(template: str, data: dict, scripts: list[str]) -> str:
    """Self-contained page: template with <!--LIBS--> replaced by inline three.js, controls, data and app scripts."""
    with open(os.path.join(WEB, template)) as fh:
        return inline_page_text(fh.read(), data, scripts)


THREE_CDN = "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
REPO_URL = "https://github.com/HarryHYu/GrappStar/tree/base-v1/"     # base for any file links on published pages
INLINE_LIMIT = 12_000_000          # body variant: data larger than this moves the meshes to files next to the page
FILE_LIMIT = 14_000_000            # each mesh file at most this many bytes


def _script(c: str) -> str:
    return "<script>\n%s\n</script>\n" % c.replace("</script", "<\\/script")


def write_pages(out: str, stem: str, template: str, data: dict, scripts: list[str], title: str) -> tuple[str, str]:
    """Write OUT/<stem>.html (standalone, offline: everything inline) and OUT/<stem>_body.html (a page body fragment
    for publishing: <title>, viewport meta and <style> first, three.js r128 from cdnjs, everything else inline; the
    meshes go to OUT/web/<stem>_meshes_<k>.json when the data is too large to inline). Returns both paths."""
    import re
    with open(os.path.join(WEB, template)) as fh:
        t = fh.read().replace("__TITLE__", _html_escape(title))
    full = inline_page_text(t, data, scripts)
    p_full = os.path.join(out, f"{stem}.html")
    with open(p_full, "w") as fh:
        fh.write(full)
    title_tag = re.search(r"<title>.*?</title>", t, re.S).group(0)
    style = re.search(r"<style>.*?</style>", t, re.S).group(0)
    body = re.search(r"<body[^>]*>(.*)</body>", t, re.S).group(1)
    d = dict(data)
    blob = data_js(d)
    webdir = os.path.join(out, "web")
    for fn in (os.listdir(webdir) if os.path.isdir(webdir) else []):
        if fn.startswith(f"{stem}_meshes_"):
            os.unlink(os.path.join(webdir, fn))
    if len(blob) > INLINE_LIMIT:
        os.makedirs(webdir, exist_ok=True)
        chunks, cur, size = [], {}, 0
        for n, m in data["meshes"].items():
            js = len(json.dumps(m)) + len(n) + 8
            if cur and size + js > FILE_LIMIT:
                chunks.append(cur)
                cur, size = {}, 0
            cur[n] = m
            size += js
        if cur:
            chunks.append(cur)
        files = []
        for k, c in enumerate(chunks):
            fn = f"{stem}_meshes_{k}.json"
            with open(os.path.join(webdir, fn), "w") as fh:
                json.dump(c, fh, separators=(",", ":"))
            files.append("web/" + fn)
        d = dict(data, meshes={}, mesh_files=files)
        blob = data_js(d)
    libs = f'<script src="{THREE_CDN}"></script>\n' + _script(open(os.path.join(WEB, "vendor", "OrbitControls.js")).read())
    libs += _script(blob) + "".join(_script(open(os.path.join(WEB, s_)).read()) for s_ in scripts)
    frag = (title_tag + '\n<meta name="claude:viewport-fit-sides" content="cover">\n' + style + "\n"
            + body.replace("<!--LIBS-->", libs).strip() + "\n")
    p_body = os.path.join(out, f"{stem}_body.html")
    with open(p_body, "w") as fh:
        fh.write(frag)
    return p_full, p_body


def _html_escape(s: str) -> str:
    import html
    return html.escape(s)


def inline_page_text(t: str, data: dict, scripts: list[str]) -> str:
    chunks = [open(os.path.join(WEB, "vendor", "three.min.js")).read(),
              open(os.path.join(WEB, "vendor", "OrbitControls.js")).read(), data_js(data)]
    chunks += [open(os.path.join(WEB, s)).read() for s in scripts]
    return t.replace("<!--LIBS-->", "".join(_script(c) for c in chunks))


# ---------------------------------------------------------------------------------------------- browser
class _Handler(http.server.SimpleHTTPRequestHandler):
    routes: dict = {}

    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?")[0].lstrip("/") or "index.html"
        src = self.routes.get(path)
        if src is None:
            src = os.path.join(WEB, path) if os.path.isfile(os.path.join(WEB, path)) else None
        if src is None:
            self.send_error(404)
            return
        body = src if isinstance(src, bytes) else open(src, "rb").read()
        ct = "text/html" if path.endswith(".html") else "application/javascript" if path.endswith(".js") else "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", ct + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def serve(routes: dict):
    """Tiny local http server: routes {path: bytes or file}, everything else from tools/web. Returns (server, url)."""
    H = type("H", (_Handler,), {"routes": routes})
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}/"


def _node_env():
    env = dict(os.environ)
    try:
        root = subprocess.run(["npm", "root", "-g"], capture_output=True, text=True, timeout=30).stdout.strip()
        if root:
            env["NODE_PATH"] = root + (os.pathsep + env["NODE_PATH"] if env.get("NODE_PATH") else "")
    except (OSError, subprocess.SubprocessError):
        pass
    return env


def run_shots(job: dict, timeout: int = 600) -> dict:
    """Run tools/web/shots.js on a job dict. Returns its JSON result; raises on failure."""
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as fh:
        json.dump(job, fh)
    try:
        r = subprocess.run(["node", os.path.join(WEB, "shots.js"), path], capture_output=True, text=True, timeout=timeout,
                           env=_node_env())
    finally:
        os.unlink(path)
    line = (r.stdout.strip().splitlines() or ["{}"])[-1]
    try:
        res = json.loads(line)
    except json.JSONDecodeError:
        res = dict(ok=False, errors=[r.stdout[-2000:], r.stderr[-2000:]])
    if r.returncode != 0:
        raise RuntimeError("shots.js failed: " + "\n".join(res.get("errors", [])) + r.stderr[-2000:])
    return res


# ---------------------------------------------------------------------------------------------- sheets
def font(size, bold=False):
    from PIL import ImageFont
    for p in (FONT_BOLD if bold else FONT_PATHS):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def grid_sheet(items, cols, dst, title, subtitle=""):
    """items: [(png, label)] -> one labelled sheet."""
    from PIL import Image, ImageDraw
    ims = [(Image.open(f).convert("RGB"), lab) for f, lab in items]
    w, h = ims[0][0].size
    rows = (len(ims) + cols - 1) // cols
    sh = Image.new("RGB", (w * cols, h * rows + TITLE_H), "white")
    d = ImageDraw.Draw(sh)
    d.text((10, 8), title, fill=(20, 22, 26), font=font(18, True))
    if subtitle:
        tw = d.textlength(title, font=font(18, True))
        d.text((24 + tw, 11), subtitle, fill=(90, 98, 110), font=font(14))
    fl = font(14)
    for i, (im, lab) in enumerate(ims):
        x, y = (i % cols) * w, (i // cols) * h + TITLE_H
        sh.paste(im, (x, y))
        tw = d.textlength(lab, font=fl)
        d.rectangle([x + 4, y + 4, x + 14 + tw, y + 24], fill=(255, 255, 255))
        d.text((x + 9, y + 6), lab, fill=(170, 60, 20), font=fl)
        d.rectangle([x, y, x + w - 1, y + h - 1], outline=(214, 218, 224))
    sh.save(dst)
    return sh.size


def section_planes(asm: Assembly, S: dict, lo, hi, per_axis=SEC_PER_AXIS):
    """Per axis: up to two planes through distinct moving joint origins (if inside the bbox), the rest evenly spaced."""
    JW = asm.joint_world(S)
    out = []
    for k, ax in enumerate("xyz"):
        a, b = float(lo[k]), float(hi[k])
        span = max(b - a, 1e-6)
        picks = []
        for jn, M in JW.items():
            if asm.joints[jn].kind == "fixed":
                continue
            c = float(M[k, 3])
            if a + 0.06 * span < c < b - 0.06 * span and all(abs(c - p[0]) > 0.08 * span for p in picks):
                picks.append((c, f"joint {jn}"))
            if len(picks) >= min(2, per_axis // 2):
                break
        m = per_axis - len(picks)
        even = [a + span * (i + 1) / (m + 1) for i in range(m)]
        for c in even:
            if any(abs(c - p[0]) < 0.06 * span for p in picks):
                c += 0.07 * span
            picks.append((c, ""))
        for c, why in sorted(picks):
            out.append((ax, round(c + 0.013, 3), why))
    return out


def _sec_view(ax):
    # camera on the kept-material side looking at the cut face, a little oblique so depth reads
    return {"x": (25, 18), "y": (115, 18), "z": (-60, 62)}[ax]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    ap.add_argument("--state-index", type=int, default=0, help="timeline state (dt 0.1 s) for the sheet and sections")
    ap.add_argument("--states", default="", help="comma separated mission phases (or 'all') rendered to states.png")
    ap.add_argument("--quick", action="store_true", help="fewer, smaller views (tests)")
    ap.add_argument("--no-sheet", action="store_true", help="skip audit sheet and sections")
    a = ap.parse_args(argv)
    names = [s for s in a.only.split(",") if s] or None
    P = load_params()
    asm = Assembly.load(P, names)
    os.makedirs(a.out, exist_ok=True)
    asm.export(a.out)
    for e in asm.errors:
        print("ERROR", e)

    tl = TL.timeline(P, 0.1)
    S0 = dict(tl[max(0, min(len(tl) - 1, a.state_index))])
    states = [S0]
    phase_sel = []
    if a.states:
        want = [p["name"] for p in TL.phases(P)] if a.states.strip() == "all" else [s.strip() for s in a.states.split(",") if s.strip()]
        known = {p["name"]: p for p in TL.phases(P)}
        for w in want:
            if w not in known:
                print(f"unknown phase {w}; known: {', '.join(known)}")
                continue
            ph = known[w]
            S = TL.state_at(P, (ph["t0"] + ph["t1"]) / 2)
            phase_sel.append((w, len(states), S))
            states.append(S)
    data = build_data(asm, a.out, states)
    views = os.path.join(a.out, "views")
    if os.path.isdir(views):
        shutil.rmtree(views)
    os.makedirs(views)

    srv, url = serve({"data.js": data_js(data).encode(), "index.html": os.path.join(WEB, "render.html")})
    tile = (TILE[0] * 3 // 4, TILE[1] * 3 // 4) if a.quick else TILE
    stile = (SEC_TILE[0] * 3 // 4, SEC_TILE[1] * 3 // 4) if a.quick else SEC_TILE
    azs = SHEET_AZ[::2] if a.quick else SHEET_AZ
    els = SHEET_EL[1:] if a.quick else SHEET_EL
    sub = f"state {a.state_index}: phase {S0.get('phase', '')}, t = {S0.get('t', 0):.1f} s"
    title_base = data["title"]
    try:
        if not a.no_sheet:
            shots, items = [], []
            for el in els:
                for az in azs:
                    f = os.path.join(views, f"view_az{az:03d}_el{el:+03d}.png")
                    shots.append(dict(file=f, ops=[["sample", 0], ["cut", None], ["ground", el > 0], ["view", az, el, 0.92]]))
                    items.append((f, f"az {az}  el {el}"))
            res = run_shots(dict(url=url + "index.html", width=tile[0], height=tile[1], shots=shots))
            info = res.get("info") or {}
            for e in res.get("errors", []):
                print("page:", e)
            size = grid_sheet(items, len(azs), os.path.join(a.out, "audit_sheet.png"),
                              f"{title_base} audit sheet: {len(items)} views", sub)
            print(f"audit_sheet.png {size[0]} x {size[1]}")

            box = info.get("box") or [[-50, -50, 0], [50, 50, 50]]
            planes = section_planes(asm, S0, box[0], box[1], 2 if a.quick else SEC_PER_AXIS)
            shots, items = [], []
            for i, (ax, c, why) in enumerate(planes):
                f = os.path.join(views, f"section_{i:02d}_{ax}.png")
                az, el = _sec_view(ax)
                shots.append(dict(file=f, ops=[["sample", 0], ["ground", ax == "z"], ["cut", ax, c], ["view", az, el, 0.8]]))
                items.append((f, f"{ax} = {c:g} mm" + (f"  ({why})" if why else "")))
            run_shots(dict(url=url + "index.html", width=stile[0], height=stile[1], shots=shots))
            size = grid_sheet(items, len(planes) // 3, os.path.join(a.out, "sections.png"),
                              f"{title_base} section cuts", sub + "; material on the + side removed, cut faces hatched")
            print(f"sections.png {size[0]} x {size[1]}")

        if phase_sel:
            shots, items = [], []
            for w, k, S in phase_sel:
                f = os.path.join(views, f"state_{w}.png")
                shots.append(dict(file=f, ops=[["sample", k], ["cut", None], ["ground", True], ["view", -55, 28, 0.92]]))
                items.append((f, f"{w}  t = {S.get('t', 0):.2f} s"))
            run_shots(dict(url=url + "index.html", width=tile[0], height=tile[1], shots=shots))
            cols = min(4, len(items))
            size = grid_sheet(items, cols, os.path.join(a.out, "states.png"), f"{title_base} mission states",
                              "mid point of each phase")
            print(f"states.png {size[0]} x {size[1]}")
    finally:
        srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
