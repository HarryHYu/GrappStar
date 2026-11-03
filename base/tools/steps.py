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


SUB_ORDER = ("frame", "drum", "launch", "reel")     # subsystem order for ties: frame first, then drum, launch, reel
TOUCH_GAP = 0.2                                      # mm, same as the audit's contact rule


def _rank(sub: str) -> int:
    return SUB_ORDER.index(sub) if sub in SUB_ORDER else len(SUB_ORDER)


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


def plan_steps(asm, contacts="auto", grounded=None, out: str | None = None, log=None) -> tuple[list[dict], list[str]]:
    """Ordered assembly steps covering every part exactly once. Returns (steps, warnings); never raises on bad steps
    and never drops an authored step.

    asm: an Assembly, or any object with .steps (authored step dicts), .parts {name: obj with step, subsystem, joint,
    printed} and .modules (dict of modules, or list of subsystem names).
    Each returned step: title, text, parts (parts this step introduces), handle (parts placed earlier that the step
    works on again), subsystem, auto, stage (int or None), seq (position), warnings (list of str).

    Rules:
    - every authored step is kept. Unknown part names are left out of parts and listed in a warning; parts already
      introduced by an earlier authored step go to handle; a step left with no parts is a text step (shows the
      current state).
    - Part.step = k adds the part to authored step k (1-based); parts in no step get auto steps per subsystem.
    - order: step["stage"] (int, lower first) is a hard order between staged steps. Otherwise a step goes once all
      its new parts connect, through rest-state contacts, to parts already placed or to grounded parts (grounded(P)).
      Ties: subsystem order (frame, drum, launch, reel, others), then authored order. Each subsystem keeps its own
      step order; a step moved ahead of an earlier step of its subsystem (dependency) is warned. A step that can never
      be placed (no contact path, cycle) goes at the end in authored order with a warning.
    - contacts: "auto" = OUT/audit.json contact_graph if it matches, else computed from the geometry (Assembly only),
      else none; None = no contact information (authored order with stage and subsystem order only).
      Steps that carry "seq" (a plan written back to asm.json) keep that order for ties instead.
    """
    warn: list[str] = []

    def w(st, msg):
        txt = f"step '{st['title']}' ({st['subsystem'] or '-'}): {msg}"
        st["warnings"].append(msg)
        warn.append(txt)

    names = list(asm.parts)
    steps, seen = [], set()
    for i, raw in enumerate(asm.steps):
        stage = raw.get("stage")
        st = dict(title=raw.get("title") or "Step", text=raw.get("text", ""), parts=[], handle=[],
                  subsystem=raw.get("subsystem", ""), auto=bool(raw.get("auto", False)),
                  stage=None, idx=i, seq=raw.get("seq"), warnings=[])
        if stage is not None:
            try:
                st["stage"] = int(stage)
            except (TypeError, ValueError):
                w(st, f"stage {stage!r} is not an integer, ignored")
        unknown = []
        for n in list(raw.get("parts") or []) + list(raw.get("handle") or []):
            if n not in asm.parts:
                unknown.append(n)
            elif n in seen:
                if n not in st["handle"] and n not in st["parts"]:
                    st["handle"].append(n)
            else:
                st["parts"].append(n)
                seen.add(n)
        if unknown:
            w(st, "unknown parts " + ", ".join(unknown))
        steps.append(st)
    n_auth = len(steps)
    extra: dict[int, list[str]] = {}
    for n, p in asm.parts.items():
        if n in seen or getattr(p, "step", None) is None:
            continue
        k = int(p.step)
        if 1 <= k <= n_auth:
            steps[k - 1]["parts"].append(n)
        else:
            extra.setdefault(k, []).append(n)
        seen.add(n)
    for k in sorted(extra):
        subs = {asm.parts[n].subsystem for n in extra[k]}
        steps.append(dict(title=f"Step {k}", text="Parts numbered for this step.", parts=extra[k], handle=[],
                          subsystem=subs.pop() if len(subs) == 1 else "", auto=True, stage=None, idx=len(steps),
                          seq=None, warnings=[]))
    subs = list(asm.modules) + sorted({p.subsystem for p in asm.parts.values()} - set(asm.modules))
    for sub in subs:
        rest = [n for n, p in asm.parts.items() if n not in seen and p.subsystem == sub]
        if not rest:
            continue
        # fixed structure first, then moving parts, bought parts after the printed ones
        rest.sort(key=lambda n: (asm.parts[n].joint is not None, not asm.parts[n].printed))
        seen.update(rest)
        steps.append(dict(title=f"Fit the remaining {sub} parts", text="", parts=rest, handle=[], subsystem=sub,
                          auto=True, stage=None, idx=len(steps), seq=None, warnings=[]))

    # ---- contact graph and grounded parts
    if isinstance(contacts, str) and contacts == "auto":
        contacts = audit_contacts(out, names) if out else None
        if contacts is None and _has_geometry(asm):
            contacts = rest_contacts(asm)
    if grounded is None:
        grounded = grounded_parts(asm)
    grounded = set(grounded) & set(names)
    if contacts is not None:
        contacts = {n: set(contacts.get(n, ())) for n in names}

    planned = all(st["seq"] is not None for st in steps)

    def tie(st):
        return (st["seq"],) if planned else (_rank(st["subsystem"]), st["idx"])

    def reach(st, placed):
        """The new parts of st that connect to placed or grounded parts through contacts among placed + new."""
        new = set(st["parts"])
        if contacts is None:
            return new
        roots = new & grounded
        if placed:
            roots |= {n for n in new if contacts[n] & placed}
        elif not grounded and st["parts"]:
            roots = {st["parts"][0]}        # nothing grounded anywhere: the first step seeds the build
        got, todo = set(roots), list(roots)
        while todo:
            x = todo.pop()
            for y in contacts[x] & new:
                if y not in got:
                    got.add(y); todo.append(y)
        return got

    def ready(st, placed):
        # parts the step works on again must be in place first
        return set(st["handle"]) <= placed and (not st["parts"] or reach(st, placed) == set(st["parts"]))

    # a part that touches nothing in the rest state can never be placed by contact: its step goes to the end
    order, placed, stuck, left = [], set(), [], []
    for st in sorted(steps, key=tie):
        loose = [] if contacts is None else [n for n in st["parts"] if not contacts[n] and n not in grounded]
        if loose and len(names) > 1:
            if st["auto"] and len(loose) < len(st["parts"]):      # only the loose parts of an auto step wait
                st, st_l = dict(st, parts=[n for n in st["parts"] if n not in loose]), dict(st, parts=loose, warnings=[])
                left.append(st)
                st = st_l
            w(st, "cannot be placed: " + ", ".join(loose) + (" touches" if len(loose) == 1 else " touch")
              + " nothing in the rest state; put at the end")
            stuck.append(st)
        else:
            left.append(st)
    # authored order per subsystem (auto steps are not part of it)
    chain: dict[str, list[dict]] = {}
    for st in left:
        if not st["auto"]:
            chain.setdefault(st["subsystem"], []).append(st)

    def earlier(s):
        ch = chain.get(s["subsystem"], [])
        return [] if s["auto"] or s not in ch else [o for o in ch[:ch.index(s)] if o not in order]

    while left:
        staged = [s["stage"] for s in left if s["stage"] is not None]
        low = min(staged) if staged else None
        allowed = [s for s in left if s["stage"] is None or s["stage"] == low]
        ok = [s for s in allowed if ready(s, placed)]
        pick = next((s for s in ok if not earlier(s)), None)
        if pick is None and ok:
            pick = ok[0]
            w(pick, "moved ahead of " + ", ".join(f"'{o['title']}'" for o in earlier(pick))
              + " of its subsystem: their parts do not sit on anything placed yet (reorder the subsystem's steps "
              "or give them a stage)")
        if pick is None:
            # an auto step with some parts that already connect: fit those now, the rest later
            for s in allowed:
                if s["auto"]:
                    got = reach(s, placed)
                    if got:
                        part = dict(s, parts=[n for n in s["parts"] if n in got], warnings=[])
                        s["parts"] = [n for n in s["parts"] if n not in got]
                        pick = part
                        break
            if pick is not None:
                order.append(pick)
                placed |= set(pick["parts"])
                continue
            # dependency cycle (or parts joined only through later steps): authored order, warned
            pick = next((s for s in allowed if not earlier(s)), allowed[0])
            miss = sorted(set(pick["parts"]) - reach(pick, placed))
            w(pick, "dependency not met: " + ", ".join(miss) + " do not sit on anything placed before (cycle or a "
              "missing contact); placed in authored order")
        order.append(pick)
        placed |= set(pick["parts"])
        left.remove(pick)
    order += stuck
    res = []
    for k, st in enumerate(order):
        st = dict(st)
        st.pop("idx")
        st["seq"] = k
        if st["auto"] and not st["text"]:
            st["text"] = "Parts without an explicit assembly step: " + ", ".join(st["parts"]) + "."
        res.append(st)
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
