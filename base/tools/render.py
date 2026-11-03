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


def hull_index(v: np.ndarray) -> np.ndarray:
    """Indices of the convex hull vertices (all vertices when the hull fails): the bbox of any rigid transform of the
    mesh is exactly the bbox of the transformed hull vertices, at a fraction of the cost."""
    v = np.asarray(v, np.float64)
    if len(v) < 5:
        return np.arange(len(v))
    try:
        from scipy.spatial import ConvexHull
        return np.sort(ConvexHull(v).vertices)
    except Exception:          # noqa: BLE001 (flat or degenerate mesh, or no scipy)
        return np.arange(len(v))


def encode_mesh(v: np.ndarray, f: np.ndarray) -> dict:
    """int16 vertices on the part's own bbox grid (better than 0.01 mm below 650 mm), uint16/uint32 indices, plus
    the hull vertex indices h (for exact bounding boxes in any pose)."""
    v = np.asarray(v, np.float64)
    lo, hi = v.min(0), v.max(0)
    s = max(float((hi - lo).max()), 1e-6) / 65534.0
    o = lo + 32767 * s
    q = np.clip(np.round((v - o) / s), -32767, 32767).astype(np.int16)
    wide = len(v) > 65535
    it = np.uint32 if wide else np.uint16
    h = hull_index(o + q.astype(np.float64) * s)
    return dict(q=[round(float(x), 6) for x in o] + [s], v=_b64(q), f=_b64(np.asarray(f, it)), h=_b64(np.asarray(h, it)),
                wide=wide, nv=int(len(v)), nf=int(len(f)), nh=int(len(h)))


def decode_mesh_points(md: dict, which: str = "h") -> np.ndarray:
    """Vertices of an encoded mesh as the page decodes them (which="h": hull vertices only, "v": all)."""
    q = np.frombuffer(base64.b64decode(md["v"]), np.int16).reshape(-1, 3).astype(np.float64)
    pts = np.asarray(md["q"][:3]) + q * md["q"][3]
    if which == "h":
        pts = pts[np.frombuffer(base64.b64decode(md["h"]), np.uint32 if md["wide"] else np.uint16)]
    return pts


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


def _joint_order(asm: Assembly) -> list[str]:
    """Joints that carry a part (directly or through a child), parents before children."""
    need = set()
    for p in asm.parts.values():
        k, guard = p.joint, 0
        while k is not None and k in asm.joints and k not in need and guard < 10_000:
            need.add(k)
            k, guard = asm.joints[k].parent, guard + 1
    order: list[str] = []
    done: set = set()

    def visit(k, depth=0):
        if k in done or depth > len(need):
            return
        par = asm.joints[k].parent
        if par in need:
            visit(par, depth + 1)
        if k not in done:
            done.add(k)
            order.append(k)

    for k in asm.joints:
        if k in need:
            visit(k)
    return order


def _motion(kind: str, axis, v: np.ndarray) -> np.ndarray:
    """Joint motion matrices (n, 4, 4) for values v (rev: degrees about axis, slide: mm along axis), as Joint.local."""
    n = len(v)
    M = np.tile(np.eye(4), (n, 1, 1))
    a = np.asarray(axis, np.float64)
    if kind == "rev":
        t = np.radians(v)
        c, s, C = np.cos(t), np.sin(t), 1 - np.cos(t)
        x, y, z = a
        M[:, 0, 0] = c + x * x * C; M[:, 0, 1] = x * y * C - z * s; M[:, 0, 2] = x * z * C + y * s
        M[:, 1, 0] = y * x * C + z * s; M[:, 1, 1] = c + y * y * C; M[:, 1, 2] = y * z * C - x * s
        M[:, 2, 0] = z * x * C - y * s; M[:, 2, 1] = z * y * C + x * s; M[:, 2, 2] = c + z * z * C
    elif kind == "slide":
        M[:, :3, 3] = a[None, :] * v[:, None]
    return M


def _pre(j) -> np.ndarray:
    M = np.eye(4)
    M[:3, 3] = j.origin
    if j.base is not None:
        M = M @ np.asarray(j.base, np.float64)
    return M


def encode_tracks(asm: Assembly, states: list[dict]):
    """Joint-space tracks. Every joint that carries a part becomes a group (index k + 1; group 0 is the world) with
    its parent group, static local transform pre = T(origin) * base and its scalar value per sample (rev: degrees
    about the axis, unwrapped as the kinematics give it; slide: mm along the axis), from Assembly.values. The page
    rebuilds group(s) = parent(s) * pre * motion(value(s)) with the value interpolated linearly between samples, so it
    matches Assembly.world at the samples and is exact for single axis motion between them (no aliasing however fast
    a joint turns, nested joints stay on their pivots). Parts keep Part.place on their joint group.

    Returns (tracks dict, {part: (group, place 4x4)}, group world matrices {group: (n or 1, 4, 4)})."""
    n = len(states)
    vals = [asm.values(S) for S in states]
    order = _joint_order(asm)
    index = {k: i for i, k in enumerate(order)}
    joints, worlds = [], {0: np.eye(4)[None]}
    for k in order:
        j = asm.joints[k]
        v = np.array([V.get(k, 0.0) for V in vals], np.float64) if j.kind != "fixed" else np.zeros(n)
        rec = dict(name=k, parent=index.get(j.parent, -1), kind=j.kind, origin=[float(x) for x in j.origin],
                   axis=[float(x) for x in j.axis],
                   base=None if j.base is None else np.asarray(j.base, np.float64).reshape(-1).tolist())
        const = n == 0 or bool(np.all(v == v[0]))
        if const:
            rec["c"] = float(v[0]) if n else 0.0
            vv = v[:1] if n else np.zeros(1)
        else:
            v32 = v.astype(np.float32)
            err = np.abs(v32.astype(np.float64) - v).max()
            # float32 when its rounding moves a point 1000 mm out by < 0.1 um, else float64
            tol = 1e-4 if j.kind == "slide" else math.degrees(1e-7)
            f64 = bool(err > tol)
            rec["v"] = _b64(v if f64 else v32)
            rec["f64"] = f64
            vv = v
        joints.append(rec)
        L = _pre(j)[None] @ _motion(j.kind, j.axis, vv)
        Wp = worlds[rec["parent"] + 1]
        W = Wp @ L if (len(Wp) == len(L) or len(Wp) == 1 or len(L) == 1) else None
        worlds[index[k] + 1] = W[:1] if np.ptp(W, axis=0).max() < 1e-12 else W
    assign = {}
    for name, p in asm.parts.items():
        g = index[p.joint] + 1 if p.joint in index else 0
        assign[name] = (g, np.asarray(p.place, np.float64))
    tracks = dict(n=n, t=[round(float(S.get("t", i)), 4) for i, S in enumerate(states)], joints=joints)
    line = encode_line(asm, states)
    if line:
        tracks["line"] = line
    return tracks, assign, worlds


def encode_line(asm: Assembly, states: list[dict]):
    """Tether polyline per sample from Assembly.line(S) (world mm; the point count may change between samples):
    {cnt: b64 uint16 per sample, p: b64 float32 xyz of all points in sample order}, or None when there is none."""
    if not hasattr(asm, "line"):
        return None
    cnt, pts = [], []
    for S in states:
        try:
            L = np.asarray(asm.line(S) or [], np.float64).reshape(-1, 3)[:65535]
        except Exception as e:      # noqa: BLE001 (a subsystem hook that does not follow line_path(P, S) -> points)
            print(f"WARNING: no tether in the sim: Assembly.line failed at t = {S.get('t', '?')}: {type(e).__name__}: {e}")
            return None
        cnt.append(len(L))
        pts.append(L)
    if max(cnt, default=0) < 2:
        return None
    return dict(cnt=_b64(np.asarray(cnt, np.uint16)), p=_b64(np.concatenate(pts).astype(np.float32)))


def track_box(meshes: dict, assign: dict, worlds: dict, chunk: int = 4_000_000):
    """World bbox [[min], [max]] of all parts over every sample (exact: hull vertices of the encoded meshes)."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for name, (g, place) in assign.items():
        md = meshes.get(name)
        if not md:
            continue
        H = decode_mesh_points(md, "h")
        if not len(H):
            continue
        Ws = worlds[g] @ place[None]
        step = max(1, chunk // max(1, len(H)))
        for a in range(0, len(Ws), step):
            Wc = Ws[a:a + step]
            P = np.einsum("nij,mj->nmi", Wc[:, :3, :3], H) + Wc[:, None, :3, 3]
            lo = np.minimum(lo, P.reshape(-1, 3).min(0))
            hi = np.maximum(hi, P.reshape(-1, 3).max(0))
    if not np.all(np.isfinite(lo)):
        lo, hi = np.array([-50.0, -50.0, 0.0]), np.array([50.0, 50.0, 50.0])
    return [[round(float(x), 4) for x in lo], [round(float(x), 4) for x in hi]]


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
    tracks, assign, worlds = encode_tracks(asm, states)
    parts = []
    for n, p in asm.parts.items():
        gi, place = assign[n]
        parts.append(dict(name=n, color=p.color, material=p.material, notes=p.notes, subsystem=p.subsystem, joint=p.joint,
                          bought_ref=p.bought_ref, printed=bool(p.printed), qty=p.qty,
                          mass_g=None if p.mass_g() is None else round(p.mass_g(), 1), step=p.step,
                          group=gi, place=np.round(np.asarray(place, np.float64), 9).reshape(-1).tolist()))
    return dict(title=title or "GrappStar " + " + ".join(asm.modules), subsystems=list(asm.modules), parts=parts,
                meshes=meshes, tracks=tracks, phase=[S.get("phase", "") for S in states], states=state_columns(states),
                ground_z=ground_z(asm.P), box_track=track_box(meshes, assign, worlds))


def data_js(data: dict) -> str:
    return "window.CAD_DATA=" + json.dumps(data, separators=(",", ":")) + ";\n"


def inline_page(template: str, data: dict, scripts: list[str]) -> str:
    """Self-contained page: template with <!--LIBS--> replaced by inline three.js, controls, data and app scripts."""
    with open(os.path.join(WEB, template)) as fh:
        return inline_page_text(fh.read(), data, scripts)


THREE_CDN = "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
REPO_URL = "https://github.com/HarryHYu/GrappStar/tree/base-v1/"     # base for any file links on published pages
PAGE_LIMIT = 16_000_000            # a published page (the body fragment) must stay below this many bytes
FILE_MAX = 15_000_000              # every supporting file next to it must stay below this many bytes
INLINE_LIMIT = 12_000_000          # body variant: data larger than this moves the big arrays to web/<stem>_ext_<k>.json
FILE_LIMIT = 14_000_000            # target size of each of those files (single arrays are split across files)


class PageTooLarge(ValueError):
    pass


def _script(c: str) -> str:
    return "<script>\n%s\n</script>\n" % c.replace("</script", "<\\/script")


def _ext_candidates(data: dict, which: str):
    """(container, key) of the large base64 strings in the data: mesh arrays or joint value tracks."""
    if which == "meshes":
        for m in data.get("meshes", {}).values():
            for k in ("v", "f", "h"):
                if isinstance(m.get(k), str) and m[k]:
                    yield m, k
    else:
        tr = data.get("tracks", {})
        for j in tr.get("joints", []):
            if isinstance(j.get("v"), str) and j["v"]:
                yield j, "v"
        if isinstance(tr.get("line"), dict):
            for k in ("p", "cnt"):
                if isinstance(tr["line"].get(k), str):
                    yield tr["line"], k


def externalize(data: dict, stem: str, webdir: str, inline_limit=None, file_limit=None) -> tuple[dict, list[str]]:
    """Move the mesh arrays (then, if that is not enough, the joint value tracks) out of the page data into
    web/<stem>_ext_<k>.json files of at most file_limit bytes each; a string longer than that is split into pieces
    across files. Each moved string becomes {"ext": id, "n": pieces} and the page joins the pieces back after
    fetching CAD_DATA.ext_files. Returns (new data, written file paths relative to the page)."""
    import copy
    inline_limit = INLINE_LIMIT if inline_limit is None else inline_limit
    file_limit = FILE_LIMIT if file_limit is None else file_limit
    d = copy.deepcopy(data)
    pieces: list[tuple[str, str]] = []
    piece = max(1024, file_limit - 4096)
    for which in ("meshes", "tracks"):
        if len(data_js(d)) <= inline_limit:
            break
        for obj, k in list(_ext_candidates(d, which)):
            s = obj[k]
            if len(s) < 256:
                continue
            eid = str(len(pieces))
            parts = [s[i:i + piece] for i in range(0, len(s), piece)]
            for i, p in enumerate(parts):
                pieces.append((f"{eid}.{i}", p))
            obj[k] = dict(ext=eid, n=len(parts))
    if not pieces:
        return d, []
    os.makedirs(webdir, exist_ok=True)
    files, cur, size = [], {}, 2

    def flush():
        fn = f"{stem}_ext_{len(files)}.json"
        with open(os.path.join(webdir, fn), "w") as fh:
            json.dump(cur, fh, separators=(",", ":"))
        files.append("web/" + fn)

    for key, p in pieces:
        need = len(p) + len(key) + 6
        if cur and size + need > file_limit:
            flush()
            cur, size = {}, 2
        cur[key] = p
        size += need
    if cur:
        flush()
    d["ext_files"] = files
    return d, files


def check_sizes(page: str, files: list[str], base_dir: str, page_limit=None, file_max=None):
    """Raise PageTooLarge when the page is over the page limit or any supporting file over the file limit."""
    page_limit = PAGE_LIMIT if page_limit is None else page_limit
    file_max = FILE_MAX if file_max is None else file_max
    ps = os.path.getsize(page)
    if ps >= page_limit:
        raise PageTooLarge(f"{page} is {ps / 1e6:.2f} MB, the limit for a published page is {page_limit / 1e6:.0f} MB; "
                           "even with the meshes and tracks moved to web/*.json the inline data is too large "
                           "(fewer parts, coarser meshes or a larger --dt)")
    for f in files:
        fs = os.path.getsize(os.path.join(base_dir, f))
        if fs >= file_max:
            raise PageTooLarge(f"{f} is {fs / 1e6:.2f} MB, every file next to a published page must stay below "
                               f"{file_max / 1e6:.0f} MB")


def write_pages(out: str, stem: str, template: str, data: dict, scripts: list[str], title: str) -> tuple[str, str]:
    """Write OUT/<stem>.html (standalone, offline: everything inline; a warning is printed above 16 MB) and
    OUT/<stem>_body.html (a page body fragment for publishing: <title>, viewport meta and <style> first, three.js r128
    from cdnjs, everything else inline; the meshes and, if needed, the joint tracks go to OUT/web/<stem>_ext_<k>.json
    when the data is too large to inline). The body fragment must stay below PAGE_LIMIT and each file below FILE_MAX,
    else PageTooLarge is raised. Returns both paths."""
    import re
    with open(os.path.join(WEB, template)) as fh:
        t = fh.read().replace("__TITLE__", _html_escape(title))
    full = inline_page_text(t, data, scripts)
    p_full = os.path.join(out, f"{stem}.html")
    with open(p_full, "w") as fh:
        fh.write(full)
    fs = os.path.getsize(p_full)
    if fs >= PAGE_LIMIT:
        print(f"WARNING: {p_full} is {fs / 1e6:.1f} MB (over {PAGE_LIMIT / 1e6:.0f} MB): fine as an offline file, but "
              f"slow to open; publish {stem}_body.html with web/ instead")
    title_tag = re.search(r"<title>.*?</title>", t, re.S).group(0)
    style = re.search(r"<style>.*?</style>", t, re.S).group(0)
    body = re.search(r"<body[^>]*>(.*)</body>", t, re.S).group(1)
    webdir = os.path.join(out, "web")
    for fn in (os.listdir(webdir) if os.path.isdir(webdir) else []):
        if fn.startswith(f"{stem}_meshes_") or fn.startswith(f"{stem}_ext_"):
            os.unlink(os.path.join(webdir, fn))
    d, files = externalize(data, stem, webdir)
    libs = f'<script src="{THREE_CDN}"></script>\n' + _script(open(os.path.join(WEB, "vendor", "OrbitControls.js")).read())
    libs += _script(data_js(d)) + "".join(_script(open(os.path.join(WEB, s_)).read()) for s_ in scripts)
    frag = (title_tag + '\n<meta name="claude:viewport-fit-sides" content="cover">\n' + style + "\n"
            + body.replace("<!--LIBS-->", libs).strip() + "\n")
    p_body = os.path.join(out, f"{stem}_body.html")
    with open(p_body, "w") as fh:
        fh.write(frag)
    check_sizes(p_body, files, out)
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
