"""Compose the subsystem modules base/cad/parts/*.py into one assembly.

    from tools.assembly import Assembly
    asm = Assembly.load(P)                     # every subsystem except demo
    asm = Assembly.load(P, names=["demo"])     # only these
    W = asm.world(S)                           # {part name: 4x4 world transform}
    solid = asm.posed("reel_spool", S)         # Manifold in world coordinates
    asm.export("out/base")                     # asm.json + meshes/<part>.npz (local frames)

python -m tools.assembly OUT [--only a,b]  exports and prints a part table.
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import pkgutil
import sys
import time

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from cad.lib import Part, Joint, Allow, Clearance, prim  # noqa: E402
from tools.params import load_params  # noqa: E402

SKIP_DEFAULT = {"demo"}


def subsystem_names():
    import cad.parts as pk
    return sorted(m.name for m in pkgutil.iter_modules(pk.__path__) if not m.name.startswith("_"))


class Assembly:
    def __init__(self, P, modules: dict):
        self.P = P
        self.modules = modules
        self.parts: dict[str, Part] = {}
        self.joints: dict[str, Joint] = {}
        self.allowed: list[Allow] = []
        self.clearances: list[Clearance] = []
        self.extra_states: list[dict] = []
        self.steps: list[dict] = []
        self.errors: list[str] = []
        for sub, mod in modules.items():
            for j in mod.joints(P):
                if j.name in self.joints:
                    self.errors.append(f"duplicate joint {j.name} ({sub})")
                self.joints[j.name] = j
            for p in mod.parts(P):
                p.subsystem = p.subsystem or sub
                if p.name in self.parts:
                    self.errors.append(f"duplicate part {p.name} ({sub})")
                self.parts[p.name] = p
            if hasattr(mod, "allowed"):
                self.allowed += list(mod.allowed(P))
            if hasattr(mod, "clearances"):
                self.clearances += list(mod.clearances(P))
            if hasattr(mod, "audit_states"):
                for S in mod.audit_states(P):
                    S = dict(S); S.setdefault("phase", f"{sub}_extra"); self.extra_states.append(S)
            if hasattr(mod, "steps"):
                for st in mod.steps(P):
                    st = dict(st); st.setdefault("subsystem", sub); self.steps.append(st)
        for j in self.joints.values():
            if j.parent is not None and j.parent not in self.joints:
                self.errors.append(f"joint {j.name}: unknown parent {j.parent}")
            if j.gear is not None and j.gear[0] not in self.joints:
                self.errors.append(f"joint {j.name}: gear refers to unknown joint {j.gear[0]}")
        for p in self.parts.values():
            if p.joint is not None and p.joint not in self.joints:
                self.errors.append(f"part {p.name}: unknown joint {p.joint}")
        self._check_cycles()

    # ------------------------------------------------------------------ loading
    @classmethod
    def load(cls, P=None, names=None):
        P = P or load_params()
        names = names or [n for n in subsystem_names() if n not in SKIP_DEFAULT]
        mods = {}
        for n in names:
            mods[n] = importlib.import_module(f"cad.parts.{n}")
            for fn in ("parts", "joints", "kin"):
                if not hasattr(mods[n], fn):
                    raise AttributeError(f"cad/parts/{n}.py must define {fn}(P...)")
        return cls(P, mods)

    def _check_cycles(self):
        for j in self.joints:
            seen, k = set(), j
            while k is not None and k in self.joints:
                if k in seen:
                    self.errors.append(f"joint cycle through {j}")
                    break
                seen.add(k)
                k = self.joints[k].parent

    # ------------------------------------------------------------------ kinematics
    def values(self, S: dict) -> dict[str, float]:
        """Joint values for state S (kin() of every subsystem, then gear links, else 0)."""
        v = {}
        for sub, mod in self.modules.items():
            for k, x in (mod.kin(self.P, S) or {}).items():
                if k not in self.joints:
                    raise KeyError(f"{sub}.kin returned unknown joint {k}")
                v[k] = float(x)
        for _ in range(len(self.joints)):           # resolve gear chains in any order
            changed = False
            for j in self.joints.values():
                if j.name not in v and j.gear is not None and j.gear[0] in v:
                    other, ratio, *off = j.gear
                    v[j.name] = ratio * v[other] + (off[0] if off else 0.0)
                    changed = True
            if not changed:
                break
        for j in self.joints:
            v.setdefault(j, 0.0)
        return v

    def joint_world(self, S: dict, values=None) -> dict[str, np.ndarray]:
        v = values or self.values(S)
        out: dict[str, np.ndarray] = {}

        def get(name):
            if name in out:
                return out[name]
            j = self.joints[name]
            Mp = np.eye(4) if j.parent not in self.joints else get(j.parent)
            out[name] = Mp @ j.local(v[name])
            return out[name]

        for n in self.joints:
            get(n)
        return out

    def world(self, S: dict) -> dict[str, np.ndarray]:
        JW = self.joint_world(S)
        return {n: JW.get(p.joint, np.eye(4)) @ p.place for n, p in self.parts.items()}

    def posed(self, name: str, S: dict, W=None):
        W = W or self.world(S)
        return prim.place(self.parts[name].solid, W[name])

    # ------------------------------------------------------------------ export
    def manifest(self) -> dict:
        parts = []
        for n, p in self.parts.items():
            lo, hi = prim.bbox(p.solid)
            parts.append(dict(name=n, subsystem=p.subsystem, material=p.material, qty=p.qty, joint=p.joint,
                              place=np.round(p.place, 6).tolist(),
                              print_tf=None if p.print_tf is None else np.round(p.print_tf, 6).tolist(),
                              color=p.color, notes=p.notes, bought_ref=p.bought_ref, printed=p.printed,
                              volume_mm3=round(p.solid.volume(), 2), mass_g=None if p.mass_g() is None else round(p.mass_g(), 2),
                              bbox=[list(np.round(lo, 3)), list(np.round(hi, 3))], step=p.step))
        joints = [dict(name=j.name, parent=j.parent, kind=j.kind, origin=list(j.origin), axis=list(j.axis),
                       base=None if j.base is None else np.round(j.base, 6).tolist(), limits=j.limits, gear=j.gear)
                  for j in self.joints.values()]
        params = {k: getattr(self.P, k) for k in dir(self.P) if k.isupper() and _jsonable(getattr(self.P, k))}
        return dict(version=1, created=time.strftime("%Y-%m-%d %H:%M"), subsystems=list(self.modules), parts=parts,
                    joints=joints, steps=self.steps, params=params, errors=self.errors)

    def export(self, out: str):
        os.makedirs(os.path.join(out, "meshes"), exist_ok=True)
        for n, p in self.parts.items():
            v, f = prim.mesh_of(p.solid)
            np.savez_compressed(os.path.join(out, "meshes", f"{n}.npz"), v=v.astype(np.float32), f=f.astype(np.uint32))
        with open(os.path.join(out, "asm.json"), "w") as fh:
            json.dump(self.manifest(), fh, indent=1)
        return out

    def tracks(self, states: list[dict]) -> dict:
        """Per-part world transforms over a list of states: {"t": [...], "phase": [...], "parts": {name: [16 floats
        per state, row major]}}. Parts that never move get a single matrix."""
        mats = {n: [] for n in self.parts}
        for S in states:
            W = self.world(S)
            for n, M in W.items():
                mats[n].append(np.round(M, 5).reshape(-1).tolist())
        out = {}
        for n, lst in mats.items():
            out[n] = lst[:1] if all(m == lst[0] for m in lst) else lst
        return dict(t=[S.get("t", i) for i, S in enumerate(states)], phase=[S.get("phase", "") for S in states],
                    states=[{k: v for k, v in S.items() if _jsonable(v)} for S in states], parts=out)


def _jsonable(x):
    try:
        json.dumps(x)
        return True
    except (TypeError, ValueError):
        return False


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--only", default="")
    a = ap.parse_args(argv)
    names = [s for s in a.only.split(",") if s] or None
    asm = Assembly.load(load_params(), names)
    asm.export(a.out)
    print(f"{len(asm.parts)} parts, {len(asm.joints)} joints from {list(asm.modules)} -> {a.out}/asm.json")
    for e in asm.errors:
        print("ERROR", e)
    return 1 if asm.errors else 0


if __name__ == "__main__":
    sys.exit(main())
