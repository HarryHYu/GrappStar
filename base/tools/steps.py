"""Assembly steps page: python -m tools.steps OUT [--only demo] [--shots]

Writes OUT/assembly.html (standalone, offline) and OUT/assembly_body.html (body fragment for publishing). Steps come
from plan_steps (authored asm.steps ({title, text, parts, subsystem, stage}) in dependency order, every authored step
kept, parts in no step in auto steps; see plan_steps); the plan is also written to OUT/asm.json "steps". Each step shows the parts fitted so far, the new ones highlighted
and flying in from an exploded offset, with a bill of parts. --shots writes a few headless screenshots to OUT/shots/.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from fnmatch import fnmatchcase

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.assembly import Assembly  # noqa: E402
from tools.params import load_params  # noqa: E402
from tools import timeline as TL  # noqa: E402
from tools.render import build_data, write_pages, run_shots  # noqa: E402


SUB_ORDER = ("frame", "drum", "launch", "reel", "sensors")  # subsystem order for steps of equal (or no) stage
TOUCH_GAP = 0.2                                      # mm, same as the audit's contact rule
# band name -> subsystems, used when params.ASSEMBLY_STAGES gives no owner for a band
BAND_GUESS = (("deck", "frame"), ("motor", "frame"), ("electronics", "frame"), ("guards", "frame"),
              ("main_shaft", "drum"), ("drum_", "drum"), ("main_stack", "drum"), ("reel_", "reel"),
              ("launcher", "launch"), ("aim", "launch"), ("sensors", "sensors"))
ANY_BAND = "test_stage_only"                         # any subsystem may use this band


def _rank(sub: str) -> int:
    return SUB_ORDER.index(sub) if sub in SUB_ORDER else len(SUB_ORDER)


def stage_bands(P=None) -> dict[str, tuple[int, int, set | None]]:
    """{band: (lo, hi, subsystems allowed or None = any)} from params ASSEMBLY_STAGES = {band: (lo, hi, owner, what)}.
    Owner "cad-frame / cad-launch" -> {"frame", "launch"}; "all" (or the test_stage_only band) -> any; no owner ->
    guessed from the band name (BAND_GUESS)."""
    if P is None:
        try:
            P = load_params()
        except Exception:                                      # noqa: BLE001
            return {}
    out = {}
    for band, v in (getattr(P, "ASSEMBLY_STAGES", None) or {}).items():
        try:
            lo, hi = int(v[0]), int(v[1])
        except (TypeError, ValueError, IndexError):
            continue
        owner = v[2] if len(v) > 2 and isinstance(v[2], str) else ""
        subs = {o.strip()[4:] for o in owner.split("/") if o.strip().startswith("cad-")}
        if band == ANY_BAND or owner.strip() == "all":
            subs = None
        elif not subs:
            subs = {s for pre, s in BAND_GUESS if band == pre or (pre.endswith("_") and band.startswith(pre))}
        out[band] = (lo, hi, subs)
    return out


def band_of(stage, bands) -> str:
    for b, (lo, hi, _) in bands.items():
        if lo <= stage <= hi:
            return b
    return ""


def _has_geometry(asm) -> bool:
    return isinstance(getattr(asm, "modules", None), dict) and hasattr(asm, "world") and all(
        hasattr(p, "solid") for p in asm.parts.values())


def grounded_parts(asm) -> set[str]:
    """Parts matched by the subsystems' grounded(P) patterns (empty when the modules are only names)."""
    out = set()
    mods = getattr(asm, "modules", None)
    if not isinstance(mods, dict):
        return out
    for mod in mods.values():
        fn = getattr(mod, "grounded", None)
        if fn is None:
            continue
        for pat in fn(getattr(asm, "P", None)) or []:
            out |= {n for n in asm.parts if fnmatchcase(n, pat)}
    return out


def audit_contacts(out: str, names) -> dict[str, set] | None:
    """OUT/audit.json "contact_graph" if it exists and covers exactly these parts, else None."""
    fn = os.path.join(out, "audit.json")
    try:
        with open(fn) as fh:
            g = json.load(fh).get("contact_graph")
    except (OSError, ValueError, AttributeError):
        return None
    if not isinstance(g, dict) or set(g) != set(names):
        return None
    return {n: set(v) & set(names) for n, v in g.items()}


def rest_contacts(asm, S=None, gap: float = TOUCH_GAP) -> dict[str, set]:
    """Contact graph in the rest state: parts whose posed solids come within gap mm (overlap counts as contact).
    AABB broad phase, then Manifold.min_gap."""
    from cad.lib import prim
    S = S if S is not None else TL.state_at(asm.P, 0.0)
    W = asm.world(S)
    names = list(asm.parts)
    posed = {n: prim.place(asm.parts[n].solid, W[n]) for n in names}
    box = {n: np.array(posed[n].bounding_box(), float) for n in names}
    adj = {n: set() for n in names}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = box[a], box[b]
            if np.any(A[:3] > B[3:] + gap) or np.any(B[:3] > A[3:] + gap):
                continue
            if posed[a].min_gap(posed[b], gap + 1e-6) <= gap:
                adj[a].add(b); adj[b].add(a)
    return adj


def heading(n: int, st: dict) -> str:
    """'Step 12 (drum_core 131): Fit drum A', the step's place in the final order with its stage band."""
    return f"Step {n}" + (f" ({st['label']})" if st.get("label") else "") + f": {st['title']}"


def plan_steps(asm, contacts="auto", grounded=None, out: str | None = None, log=None,
               bands=None) -> tuple[list[dict], list[str]]:
    """Ordered assembly steps covering every part exactly once. Returns (steps, warnings); never raises on bad steps
    and never drops an authored step.

    asm: an Assembly, or any object with .steps (authored step dicts), .parts {name: obj with step, subsystem, joint,
    printed} and .modules (dict of modules, or list of subsystem names).
    Each returned step: title, text, parts (parts this step introduces), handle (parts introduced by an earlier step
    that this step works on again), subsystem, auto, stage (int or None), band (stage band name or ""), label
    ("drum_core 131" or ""), seq (position), warnings (list of str).

    Rules:
    - every authored step is kept. Unknown part names are left out and warned; a step with no new parts is a text
      step (shows the current state). Part.step = k adds the part to authored step k (1-based).
    - order: strictly by step["stage"] (params ASSEMBLY_STAGES bands), then subsystem order (frame, drum, launch,
      reel, sensors, others), then the subsystem's own order. Steps without a stage come after all staged steps
      (same tie rule) and are warned; a stage outside the owner's bands is warned. Steps carrying "seq" (the plan
      written back to asm.json) keep that order.
    - the first step in that order that names a part introduces it; later mentions go to handle.
    - parts in no step get auto steps per subsystem, inserted where their parts first connect to placed parts.
    - validator only (never reorders): a step whose new parts rest only on parts placed later (rest-state contacts,
      grounded(P) parts as roots) is warned, and so is a part that touches nothing.
    - contacts: "auto" = OUT/audit.json contact_graph if it matches, else computed from the geometry (Assembly only),
      else none; None = no contact check.
    - bands: {band: (lo, hi, subsystems or None)}; default stage_bands(asm.P or the params).
    """
    warn: list[str] = []

    def w(st, msg):
        st["warnings"].append(msg)
        warn.append(f"step '{st['title']}' ({st['subsystem'] or '-'}): {msg}")

    if bands is None:
        bands = stage_bands(getattr(asm, "P", None))
    names = list(asm.parts)
    steps = []
    for i, raw in enumerate(asm.steps):
        st = dict(title=raw.get("title") or "Step", text=raw.get("text", ""), raw=[], subsystem=raw.get("subsystem", ""),
                  auto=bool(raw.get("auto", False)), stage=None, idx=i, seq=raw.get("seq"), warnings=[])
        stage = raw.get("stage")
        if stage is not None:
            try:
                st["stage"] = int(stage)
            except (TypeError, ValueError):
                w(st, f"stage {stage!r} is not an integer, ignored")
        unknown = []
        for n in list(raw.get("parts") or []) + list(raw.get("handle") or []):
            if n not in asm.parts:
                unknown.append(n)
            elif n not in st["raw"]:
                st["raw"].append(n)
        if unknown:
            w(st, "unknown parts " + ", ".join(unknown))
        steps.append(st)
    named = {n for st in steps for n in st["raw"]}
    extra: dict[int, list[str]] = {}
    for n, p in asm.parts.items():
        if n in named or getattr(p, "step", None) is None:
            continue
        k = int(p.step)
        if 1 <= k <= len(steps):
            steps[k - 1]["raw"].append(n)
        else:
            extra.setdefault(k, []).append(n)
        named.add(n)
    for k in sorted(extra):
        subs = {asm.parts[n].subsystem for n in extra[k]}
        steps.append(dict(title=f"Step {k}", text="Parts numbered for this step.", raw=extra[k],
                          subsystem=subs.pop() if len(subs) == 1 else "", auto=False, stage=None, idx=len(steps),
                          seq=None, warnings=[]))

    # ---- stage checks and the order
    for st in steps:
        if st["auto"]:
            continue
        s = st["stage"]
        own = sorted(k for k, (lo, hi, subs) in bands.items() if subs is not None and st["subsystem"] in subs)
        if s is None:
            if own:                         # subsystems with no band of their own (demo) need no stage
                w(st, f"no stage (set step['stage'] inside its ASSEMBLY_STAGES bands: {', '.join(own)})")
            continue
        if not bands:
            continue
        b = band_of(s, bands)
        if not b:
            w(st, f"stage {s} is in no ASSEMBLY_STAGES band (own bands: {', '.join(own) or 'none'})")
        elif bands[b][2] is not None and st["subsystem"] not in bands[b][2]:
            w(st, f"stage {s} is in band {b}, which is not {st['subsystem'] or 'this subsystem'}'s "
                  f"(own bands: {', '.join(own) or 'none'})")
        st["band"] = b
    planned = steps and all(st["seq"] is not None for st in steps)
    big = float("inf")
    if planned:
        order = sorted(steps, key=lambda st: st["seq"])
    else:
        order = sorted(steps, key=lambda st: (big if st["stage"] is None else st["stage"], _rank(st["subsystem"]),
                                              st["idx"]))
    seen = set()
    for st in order:
        st["parts"] = [n for n in st["raw"] if n not in seen]
        st["handle"] = [n for n in st["raw"] if n in seen]
        seen.update(st["parts"])

    # ---- contacts and grounded parts
    if isinstance(contacts, str) and contacts == "auto":
        contacts = audit_contacts(out, names) if out else None
        if contacts is None and _has_geometry(asm):
            contacts = rest_contacts(asm)
    if grounded is None:
        grounded = grounded_parts(asm)
    grounded = set(grounded) & set(names)
    if contacts is not None:
        contacts = {n: set(contacts.get(n, ())) for n in names}

    def reach(parts, placed):
        """The parts that connect to placed or grounded parts through contacts among placed + parts."""
        new = set(parts)
        if contacts is None:
            return new
        roots = new & grounded
        if placed:
            roots |= {n for n in new if contacts[n] & placed}
        elif not grounded and parts:
            roots = {parts[0]}              # nothing grounded anywhere: the first step seeds the build
        got, todo = set(roots), list(roots)
        while todo:
            x = todo.pop()
            for y in contacts[x] & new:
                if y not in got:
                    got.add(y); todo.append(y)
        return got

    # ---- auto steps for parts in no step: inserted where they first connect (split when only some do)
    subs = list(asm.modules) + sorted({p.subsystem for p in asm.parts.values()} - set(asm.modules))
    for sub in sorted(subs, key=_rank):
        rest = [n for n, p in asm.parts.items() if n not in seen and p.subsystem == sub]
        if not rest:
            continue
        # fixed structure first, then moving parts, bought parts after the printed ones
        rest.sort(key=lambda n: (asm.parts[n].joint is not None, not asm.parts[n].printed))
        seen.update(rest)
        placed, k = set(), 0
        while rest and k <= len(order):
            got = reach(rest, placed) if contacts is not None else set(rest) if k == len(order) else set()
            if got:
                chunk = [n for n in rest if n in got]
                order.insert(k, dict(title=f"Fit the remaining {sub} parts", text="", parts=chunk, handle=[],
                                     subsystem=sub, auto=True, stage=None, warnings=[]))
                rest = [n for n in rest if n not in got]
                placed |= set(chunk)
                k += 1
                continue
            if k < len(order):
                placed |= set(order[k]["parts"])
            k += 1
        if rest:
            order.append(dict(title=f"Fit the remaining {sub} parts", text="", parts=rest, handle=[],
                              subsystem=sub, auto=True, stage=None, warnings=[]))

    # ---- validator: new parts must sit on parts placed before (never reorders)
    if contacts is not None:
        intro = {n: st for st in order for n in st["parts"]}
        placed = set()
        for st in order:
            missing = sorted(set(st["parts"]) - reach(st["parts"], placed))
            loose = [n for n in missing if not contacts[n] and len(names) > 1]
            if loose:
                w(st, ", ".join(loose) + (" touches" if len(loose) == 1 else " touch") + " nothing in the rest state")
            later = [n for n in missing if n not in loose]
            if later:
                on = sorted({f"'{intro[m]['title']}'" for n in later for m in contacts[n]
                             if m in intro and m not in placed and intro[m] is not st})
                more = f" and {len(on) - 4} more" if len(on) > 4 else ""
                w(st, ", ".join(later) + " rest only on parts placed later"
                  + (f" (by {', '.join(on[:4])}{more})" if on else ""))
            if not set(st["handle"]) <= placed:
                w(st, "works on parts placed later: " + ", ".join(sorted(set(st["handle"]) - placed)))
            placed |= set(st["parts"])

    res = []
    for k, st in enumerate(order):
        b = st.get("band") or ""
        res.append(dict(title=st["title"], text=st["text"] or ("Parts without an explicit assembly step: "
                                                                + ", ".join(st["parts"]) + "." if st["auto"] else ""),
                        parts=st["parts"], handle=st["handle"], subsystem=st["subsystem"], auto=st["auto"],
                        stage=st["stage"], band=b, label=f"{b} {st['stage']}" if b else
                        ("" if st["stage"] is None else str(st["stage"])), seq=k, warnings=st["warnings"]))
        res[-1]["heading"] = heading(k + 1, res[-1])
    for x in warn:
        (log or print)("WARNING", x)
    return res, warn


def write_plan(out: str, steps: list[dict]):
    """Put the planned steps into OUT/asm.json "steps" (the authored ones stay in "steps_authored"), so tools that
    plan from the manifest without geometry (guide) keep exactly this order: planned steps carry "seq"."""
    fn = os.path.join(out, "asm.json")
    with open(fn) as fh:
        man = json.load(fh)
    man.setdefault("steps_authored", man.get("steps", []))
    man["steps"] = steps
    with open(fn, "w") as fh:
        json.dump(man, fh, indent=1)


def build(out: str, names=None, P=None) -> tuple[str, list[dict]]:
    P = P or load_params()
    asm = Assembly.load(P, names)
    os.makedirs(out, exist_ok=True)
    asm.export(out)
    steps, warn = plan_steps(asm, out=out)
    write_plan(out, steps)
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
    job.append(dict(file=f, ops=[["ASM_GO", n - 1], ["ASM_FREEZE", 0.2]]))
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
