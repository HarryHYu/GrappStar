"""tools.steps.plan_steps: no authored step dropped, text steps, unknown parts, stage and dependency order, real base."""
import importlib
import types

import pytest

from cad.lib import Part, Joint, prim, tf
from tools.assembly import Assembly, subsystem_names, SKIP_DEFAULT
from tools.params import load_params
from tools.steps import plan_steps


def mod(parts, steps=(), grounded=None, joints=()):
    m = types.SimpleNamespace(parts=lambda P: parts, joints=lambda P: list(joints), kin=lambda P, S: {},
                              steps=lambda P: [dict(s) for s in steps])
    if grounded is not None:
        m.grounded = lambda P: grounded
    return m


def block(name, x, y, z, sx=10, sy=10, sz=10):
    """Box with its lower corner at (x, y, z)."""
    return Part(name, prim.place(prim.cbox(sx, sy, sz), tf.T(x + sx / 2, y + sy / 2, z + sz / 2)), print_tf=tf.I4)


def quiet(*a):
    pass


def ns(steps, parts, modules):
    """Manifest-like assembly without geometry (what tools.guide passes)."""
    P = {n: types.SimpleNamespace(step=None, subsystem=s, joint=None, printed=True) for n, s in parts.items()}
    return types.SimpleNamespace(steps=steps, parts=P, modules=modules)


def titles(res):
    return [s["title"] for s in res]


def test_no_step_dropped_text_and_handle_steps():
    A = ns([dict(title="Deck", text="", parts=["a"], subsystem="frame"),
            dict(title="Set the buck", text="Turn the pot to 5.0 V.", parts=[], subsystem="frame"),
            dict(title="Bolt the deck", text="", parts=["a"], subsystem="frame"),          # parts all used before
            dict(title="Gear", text="", parts=["b"], subsystem="frame")],
           {"a": "frame", "b": "frame", "c": "frame"}, ["frame"])
    res, warn = plan_steps(A, contacts=None, log=quiet)
    auto = [s for s in res if s["auto"]]
    assert len(res) == len(A.steps) + len(auto) and len(auto) == 1
    assert titles(res)[:4] == ["Deck", "Set the buck", "Bolt the deck", "Gear"]
    assert res[1]["parts"] == [] and res[1]["text"].startswith("Turn")
    assert res[2]["parts"] == [] and res[2]["handle"] == ["a"]
    assert sorted(n for s in res for n in s["parts"]) == ["a", "b", "c"]          # every part exactly once
    assert warn == [] and all(s["warnings"] == [] for s in res)


def test_unknown_parts_kept_and_warned(capsys):
    A = ns([dict(title="Ghost", text="x", parts=["nope", "a"], subsystem="frame"),
            dict(title="All ghost", text="y", parts=["zip"], subsystem="frame")], {"a": "frame"}, ["frame"])
    res, warn = plan_steps(A, contacts=None)
    assert titles(res) == ["Ghost", "All ghost"]
    assert res[0]["parts"] == ["a"] and "nope" in res[0]["warnings"][0]
    assert res[1]["parts"] == [] and "zip" in res[1]["warnings"][0]
    assert len(warn) == 2 and "WARNING" in capsys.readouterr().out


def test_stage_ordering():
    A = ns([dict(title="frame late", parts=["f"], subsystem="frame", stage=2),
            dict(title="reel early", parts=["r"], subsystem="reel", stage=1),
            dict(title="drum free", parts=["d"], subsystem="drum")],
           {"f": "frame", "r": "reel", "d": "drum"}, ["drum", "frame", "reel"])
    res, warn = plan_steps(A, contacts=None, log=quiet)
    assert titles(res).index("reel early") < titles(res).index("frame late")
    assert res[titles(res).index("reel early")]["stage"] == 1
    assert not warn


def test_subsystem_order_without_contacts():
    A = ns([dict(title="drum 1", parts=["d"], subsystem="drum"), dict(title="frame 1", parts=["f"], subsystem="frame"),
            dict(title="x 1", parts=["x"], subsystem="aaa")],
           {"f": "frame", "d": "drum", "x": "aaa"}, ["aaa", "drum", "frame"])
    res, _ = plan_steps(A, contacts=None, log=quiet)
    assert titles(res) == ["frame 1", "drum 1", "x 1"]


def deck_drum(drum_steps_first=True):
    deck = block("frame_deck", -50, -50, -10, 100, 100, 10)            # top at z = 0
    drum = block("drum_drum", 0, 0, 0)                                 # sits on the deck
    lid = block("drum_lid", 0, 0, 10)                                  # sits on the drum
    loose = block("drum_loose", 200, 200, 200)                         # touches nothing
    auto = block("drum_cap", 0, 0, 20, 4, 4, 2)                        # in no step, sits on the lid
    mods = {"drum": mod([drum, lid, loose, auto], [dict(title="Fit the lid", text="", parts=["drum_lid"]),
                                                    dict(title="Fit the drum", text="", parts=["drum_drum"]),
                                                    dict(title="Loose", text="", parts=["drum_loose"])]),
            "aux": mod([], [dict(title="Wipe it", text="Clean the deck.", parts=[])]),
            "frame": mod([deck], [dict(title="Deck", text="", parts=["frame_deck"])], grounded=["frame_deck*"])}
    return Assembly(load_params(), mods)


def test_dependency_ordering_geometry():
    asm = deck_drum()
    res, warn = plan_steps(asm, log=quiet)                # contacts computed from the posed solids
    t = titles(res)
    assert len(res) == len(asm.steps) + sum(s["auto"] for s in res)
    assert t.index("Deck") < t.index("Fit the drum") < t.index("Fit the lid") < t.index("Fit the remaining drum parts")
    assert t[-1] == "Loose" and any("touches nothing" in x for x in res[-1]["warnings"])
    lid = res[t.index("Fit the lid")]
    assert any("moved ahead" in x for x in res[t.index("Fit the drum")]["warnings"]) and not lid["warnings"]
    assert "Wipe it" in t                                  # text step of another subsystem kept
    assert len(warn) == 2


def test_dependency_cycle_warned_not_raised():
    A = ns([dict(title="one", parts=["a"], subsystem="frame"), dict(title="two", parts=["b"], subsystem="frame")],
           {"a": "frame", "b": "frame", "g": "frame"}, ["frame"])
    C = {"a": {"b"}, "b": {"a"}, "g": set()}               # a and b only touch each other, g is grounded
    res, warn = plan_steps(A, contacts=C, grounded={"g"}, log=quiet)
    assert sorted(titles(res)) == sorted(["one", "two", "Fit the remaining frame parts"])
    assert titles(res).index("one") < titles(res).index("two")
    assert any("dependency not met" in x for x in warn)


def test_plan_is_stable_when_replanned():
    """The plan written back to asm.json (steps with seq) gives the same list without geometry (tools.guide)."""
    asm = deck_drum()
    res, _ = plan_steps(asm, log=quiet)
    parts = {n: p.subsystem for n, p in asm.parts.items()}
    again, warn = plan_steps(ns(res, parts, list(asm.modules)), contacts=None, log=quiet)
    assert [(s["title"], s["parts"], s["handle"]) for s in again] == [(s["title"], s["parts"], s["handle"]) for s in res]
    assert not warn


def test_real_base_plan_has_no_warnings():
    names = [n for n in subsystem_names() if n not in SKIP_DEFAULT]
    for n in names:
        try:
            importlib.import_module(f"cad.parts.{n}")
        except Exception as e:                              # noqa: BLE001
            pytest.skip(f"subsystem {n} does not import: {e}")
    asm = Assembly.load(load_params(), names)
    res, warn = plan_steps(asm, log=quiet)
    for k, s in enumerate(res, 1):
        print(k, s["subsystem"], s["title"], s["parts"], ("handle " + ",".join(s["handle"])) if s["handle"] else "")
    for x in warn:
        print("WARNING", x)
    assert len(res) == len(asm.steps) + sum(s["auto"] for s in res)
    assert sorted(n for s in res for n in s["parts"]) == sorted(asm.parts)
    assert not warn, "assembly step warnings:\n" + "\n".join(warn)
