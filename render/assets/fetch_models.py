"""Fetch and convert the third-party models and textures used by render/models.py (see CREDITS.md).

    kedge/.venv/bin/python render/assets/fetch_models.py          # download + convert everything missing

1. Jetson Orin Nano developer kit: NVIDIA's dev kit assembly as included in the CC0 Printables model 555369
   ("NVIDIA Jetson Orin Nano Developer's Kit Case", file Orin_nano_dev_kit_case.STEP, 87.6 MB, no login: the public
   Printables GraphQL download link). The STEP is tessellated with build123d / OCP (kedge/.venv has them) into
   render/assets/models/jetson_orin_nano_devkit.npz (mm, kit frame, one mesh per leaf solid with its part name);
   the printed case and the NVMe SSD (Harry has none) are left out.
2. CC0 PBR texture sets from ambientCG (1K JPG) and Poly Haven (plywood, 2K JPG) into
   render/assets/cache/tex_models/<id>/.
The STEP and the textures stay in render/assets/cache (git-ignored); the converted npz is committed.
"""
from __future__ import annotations

import io
import json
import os
import sys
import time
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, "cache")
MODELS = os.path.join(HERE, "models")
STEP_PATH = os.path.join(CACHE, "jetson_dl", "orin_nano_dev_kit_case.step")
NPZ_PATH = os.path.join(MODELS, "jetson_orin_nano_devkit.npz")
PRINTABLES = dict(model_id="555369", file_id="2359914")
TEXTURES = ["Wood090A", "Wood088", "Metal009", "Fingerprints002", "Plastic006", "Rubber004"]   # ambientCG
POLYHAVEN = {"plywood": ("diff", "rough", "nor_gl")}                                           # 2K JPG
SKIP_PATHS = ("Orin_nano_case", "NVME-M-2_ASM")


def _get(url, data=None, timeout=600):
    req = urllib.request.Request(url, data=data, headers={"User-Agent": "grappstar-render/1.0",
                                                          "Content-Type": "application/json"})
    return urllib.request.urlopen(req, timeout=timeout)


def fetch_step():
    if os.path.exists(STEP_PATH):
        return STEP_PATH
    os.makedirs(os.path.dirname(STEP_PATH), exist_ok=True)
    q = {"query": "mutation GetDownloadLink($id: ID!, $modelId: ID!, $fileType: DownloadFileTypeEnum!, "
                  "$source: DownloadSourceEnum!) { getDownloadLink(id: $id, printId: $modelId, fileType: $fileType, "
                  "source: $source) { ok output { link } } }",
         "variables": {"id": PRINTABLES["file_id"], "modelId": PRINTABLES["model_id"], "fileType": "stl",
                       "source": "model_detail"}}
    with _get("https://api.printables.com/graphql/", json.dumps(q).encode()) as r:
        link = json.load(r)["data"]["getDownloadLink"]["output"]["link"]
    print("downloading", link)
    with _get(link) as r, open(STEP_PATH + ".part", "wb") as fh:
        while True:
            b = r.read(1 << 20)
            if not b:
                break
            fh.write(b)
    os.replace(STEP_PATH + ".part", STEP_PATH)
    return STEP_PATH


def convert_step(step=STEP_PATH, out=NPZ_PATH):
    """STEP -> npz: names (leaf part names), paths, offsets into concatenated v (float32 mm) / f (uint32)"""
    import numpy as np
    from build123d import import_step, Location
    t0 = time.time()
    comp = import_step(step)
    print(f"imported in {time.time() - t0:.0f} s")
    leaves = []

    def mat(loc):
        T = loc.wrapped.Transformation()
        return np.array([[T.Value(r, c) for c in range(1, 5)] for r in range(1, 4)])

    def walk(s, path, acc):
        # a child's location is relative to its parent assembly (build123d keeps the assembly tree): leaves carry
        # their own location in their shape; the ancestors' locations are applied to the tessellated vertices
        ch = list(getattr(s, "children", []) or [])
        if ch:
            acc = acc * s.location if path[1:] else acc
            for k in ch:
                walk(k, path + [k.label or "?"], acc)
        else:
            leaves.append((path, s, mat(acc)))
    walk(comp, [comp.label or "root"], Location())
    names, paths, vs, fs, voff, foff = [], [], [], [], [0], [0]
    for path, s, A in leaves:
        p = "/".join(path)
        if any(k in p for k in SKIP_PATHS):
            continue
        bb = s.bounding_box()
        size = max(bb.size.X, bb.size.Y, bb.size.Z)
        tol = min(0.08, max(0.006, 0.004 * size))
        try:
            v, f = s.tessellate(tol, 0.35)
        except Exception as e:                       # noqa: BLE001
            print("skip", p, e)
            continue
        if not f:
            continue
        v = np.array([(q.X, q.Y, q.Z) for q in v], np.float64) @ A[:, :3].T + A[:, 3]
        v = v.astype(np.float32)
        f = np.array(f, np.uint32)
        names.append(path[-1])
        paths.append(p)
        fs.append(f + voff[-1])                      # global vertex indices
        vs.append(v)
        voff.append(voff[-1] + len(v))
        foff.append(foff[-1] + len(f))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez_compressed(out, names=np.array(names), paths=np.array(paths), v=np.vstack(vs), f=np.vstack(fs),
                        voff=np.array(voff, np.int64), foff=np.array(foff, np.int64), global_idx=np.array(1))
    print(f"{len(names)} leaves, {foff[-1]} triangles -> {out} ({os.path.getsize(out) / 1e6:.1f} MB) "
          f"in {time.time() - t0:.0f} s")
    return out


def fetch_textures():
    dst = os.path.join(CACHE, "tex_models")
    for pid, maps in POLYHAVEN.items():
        d = os.path.join(dst, pid)
        os.makedirs(d, exist_ok=True)
        for mp in maps:
            fn = os.path.join(d, f"{pid}_{mp}_2k.jpg")
            if os.path.exists(fn):
                continue
            url = f"https://dl.polyhaven.org/file/ph-assets/Textures/jpg/2k/{pid}/{pid}_{mp}_2k.jpg"
            print("downloading", url)
            with _get(url) as r, open(fn, "wb") as fh:
                fh.write(r.read())
    for tid in TEXTURES:
        d = os.path.join(dst, tid)
        if os.path.isdir(d) and any(n.endswith("_Color.jpg") for n in os.listdir(d)):
            continue
        os.makedirs(d, exist_ok=True)
        url = f"https://ambientcg.com/get?file={tid}_1K-JPG.zip"
        print("downloading", url)
        with _get(url) as r:
            z = zipfile.ZipFile(io.BytesIO(r.read()))
        for n in z.namelist():
            if n.endswith(".jpg") and "NormalDX" not in n:
                with open(os.path.join(d, os.path.basename(n)), "wb") as fh:
                    fh.write(z.read(n))


if __name__ == "__main__":
    fetch_textures()
    if not os.path.exists(NPZ_PATH) or "--force" in sys.argv:
        convert_step(fetch_step())
