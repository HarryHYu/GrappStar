#!/usr/bin/env python3
"""Download the CC0 environment assets used by render/env.py into render/assets/cache.

Plain stdlib Python (3.8+), so it runs with the system python or Blender's python:
    python3 render/assets/fetch_env.py              # 4K HDRI + 2K textures (about 70 MB on disk)
    python3 render/assets/fetch_env.py --hdri 8k    # sharper horizon for 4K output (about 100 MB, not 27 MB)
    python3 render/assets/fetch_env.py --hdri 2k    # smaller HDRI (about 7 MB)
    python3 render/assets/fetch_env.py --force      # re-download everything

All assets are CC0 from Poly Haven and ambientCG (see render/assets/CREDITS.md). Poly Haven URLs are resolved
through api.polyhaven.com/files/<id> with a fallback to the stable dl.polyhaven.org pattern; the ambientCG lawn
comes as a zip of which only the needed maps are kept. env.py builds without any of these files (procedural
fallbacks), they only make it look better.
"""
import argparse
import io
import json
import os
import shutil
import sys
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, "cache")

HDRI = "pretoria_gardens"   # sunny with cumulus (sun 52 deg), green trees and lawn all round
# Poly Haven texture id -> real-world size in metres ("dimensions"), used by env.py for scale
TEXTURES = {
    "dry_ground_rocks": 4.0,    # soil with pebbles: small bare spots in the lawn
    "bark_bluegum": 1.82,       # blue gum bark (re-toned in env.py): tree trunks, dead trees, stump, sticks
    "rough_wood": 0.5,          # weathered grey timber: post-and-rail fence, gate and shelter posts
}
# Poly Haven map key -> local file suffix
MAPS = {"Diffuse": "diff", "nor_gl": "nor_gl", "arm": "arm"}
# ambientCG material id -> (size m, {zip member suffix: local suffix})
AMBIENTCG = {
    "Grass004": (1.4, {"Color": "diff", "NormalGL": "nor_gl", "Roughness": "rough"}),   # mown lawn sward
}
UA = {"User-Agent": "grappstar-render-fetch/1.0"}


def _get(url, timeout=60):
    req = urllib.request.Request(url, headers=UA)
    return urllib.request.urlopen(req, timeout=timeout)


def _api_files(asset_id):
    try:
        with _get("https://api.polyhaven.com/files/%s" % asset_id, 30) as r:
            return json.load(r)
    except Exception as e:  # offline or API change: use URL pattern
        print("  api lookup failed for %s (%s), using URL pattern" % (asset_id, e))
        return None


def _download(url, dest, force):
    if os.path.exists(dest) and os.path.getsize(dest) > 0 and not force:
        print("  have  %s" % os.path.relpath(dest, HERE))
        return True
    tmp = dest + ".part"
    try:
        with _get(url) as r, open(tmp, "wb") as f:
            shutil.copyfileobj(r, f, 1 << 20)
        os.replace(tmp, dest)
        print("  got   %s (%.1f MB)" % (os.path.relpath(dest, HERE), os.path.getsize(dest) / 1e6))
        return True
    except Exception as e:
        if os.path.exists(tmp):
            os.remove(tmp)
        print("  FAIL  %s: %s" % (url, e))
        return False


def hdri_path(res="4k"):
    return os.path.join(CACHE, "hdri", "%s_%s.hdr" % (HDRI, res))


def texture_path(tex, key, res="2k"):
    return os.path.join(CACHE, "tex", tex, "%s_%s_%s.jpg" % (tex, key, res))


def _fetch_ambientcg(asset, res, members, force):
    dests = {m: texture_path(asset, s, res.lower()) for m, s in members.items()}
    if not force and all(os.path.exists(d) for d in dests.values()):
        for d in dests.values():
            print("  have  %s" % os.path.relpath(d, HERE))
        return True
    url = "https://ambientcg.com/get?file=%s_%s-JPG.zip" % (asset, res.upper())
    try:
        with _get(url, 300) as r:
            data = r.read()
        z = zipfile.ZipFile(io.BytesIO(data))
        names = z.namelist()
        for m, d in dests.items():
            src = [n for n in names if n.endswith("_%s.jpg" % m)][0]
            with z.open(src) as fsrc, open(d, "wb") as fdst:
                shutil.copyfileobj(fsrc, fdst)
            print("  got   %s (%.1f MB)" % (os.path.relpath(d, HERE), os.path.getsize(d) / 1e6))
        return True
    except Exception as e:
        print("  FAIL  %s: %s" % (url, e))
        return False


MODELS = ["grass_bermuda_01"]   # Poly Haven scanned lawn grass clumps (1K blend + its textures)


def _fetch_model(mid, force):
    d = os.path.join(CACHE, "models", mid)
    os.makedirs(os.path.join(d, "textures"), exist_ok=True)
    ok = True
    try:
        with _get("https://api.polyhaven.com/files/%s" % mid, 30) as r:
            b = json.load(r)["blend"]["1k"]["blend"]
        ok &= _download(b["url"], os.path.join(d, "%s_1k.blend" % mid), force)
        for rel, v in b.get("include", {}).items():
            ok &= _download(v["url"], os.path.join(d, rel), force)
    except Exception as e:
        print("  FAIL  model %s: %s" % (mid, e))
        ok = False
    return ok


def fetch(hdri_res="4k", tex_res="2k", force=False):
    ok = True
    os.makedirs(os.path.join(CACHE, "hdri"), exist_ok=True)
    print("HDRI %s (%s hdr)" % (HDRI, hdri_res))
    files = _api_files(HDRI)
    try:
        url = files["hdri"][hdri_res]["hdr"]["url"]
    except Exception:
        url = "https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/%s/%s_%s.hdr" % (hdri_res, HDRI, hdri_res)
    ok &= _download(url, hdri_path(hdri_res), force)

    for tex in TEXTURES:
        print("texture %s (Poly Haven, %s jpg)" % (tex, tex_res))
        os.makedirs(os.path.join(CACHE, "tex", tex), exist_ok=True)
        files = _api_files(tex)
        for mkey, suffix in MAPS.items():
            try:
                url = files[mkey][tex_res]["jpg"]["url"]
            except Exception:
                url = ("https://dl.polyhaven.org/file/ph-assets/Textures/jpg/%s/%s/%s_%s_%s.jpg"
                       % (tex_res, tex, tex, suffix, tex_res))
            ok &= _download(url, texture_path(tex, suffix, tex_res), force)

    for mid in MODELS:
        print("model %s (Poly Haven, 1k blend)" % mid)
        ok &= _fetch_model(mid, force)
    for asset, (_size, members) in AMBIENTCG.items():
        print("texture %s (ambientCG, %s jpg)" % (asset, tex_res))
        os.makedirs(os.path.join(CACHE, "tex", asset), exist_ok=True)
        ok &= _fetch_ambientcg(asset, tex_res, members, force)
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--hdri", default="4k", choices=["1k", "2k", "4k", "8k"])
    ap.add_argument("--tex", default="2k", choices=["1k", "2k"])
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    good = fetch(a.hdri, a.tex, a.force)
    print("done" if good else "done with failures (env.py falls back to procedural materials)")
    sys.exit(0 if good else 1)
