"""tools.steps.plan_steps: no authored step dropped, text and handle steps, unknown parts, strict stage order with
band checks, the dependency validator, auto steps, labels, and the real base."""
import importlib
import types

import pytest

from cad.lib import Part, prim, tf
from tools.assembly import Assembly, subsystem_names, SKIP_DEFAULT
from tools.params import load_params
from tools.steps import plan_steps, stage_bands, heading

# synthetic bands: frame 100-119, drum 130-139, reel 150-159, guards shared by frame and drum, test band for all
BANDS = dict(deck=(100, 109, {"frame"}), motor=(110, 119, {"frame"}), drum_core=(130, 139, {"drum"}),
             reel_capstan=(150, 159, {"reel"}), guards=(230, 239, {"frame", "drum"}), test_stage_only=(900, 999, None))


def mod(parts, steps=(), grounded=None):
    m = types.SimpleNamespace(parts=lambda P: parts, joints=lambda P: [], kin=lambda P, S: {},
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
    A = ns([dict(title="Deck", text="", parts=["a"], subsystem="frame", stage=100),
            dict(title="Set the buck", text="Turn the pot to 5.0 V.", parts=[], subsystem="frame", stage=101),
            dict(title="Bolt the deck", text="", parts=["a"], subsystem="frame", stage=102),   # parts all used before
            dict(title="Gear", text="", parts=["b"], subsystem="frame", stage=103)],
           {"a": "frame", "b": "frame", "c": "frame"}, ["frame"])
    res, warn = plan_steps(A, contacts=None, log=quiet, bands=BANDS)
    auto = [s for s in res if s["auto"]]
    assert len(res) == len(A.steps) + len(auto) and len(auto) == 1
    assert titles(res)[:4] == ["Deck", "Set the buck", "Bolt the deck", "Gear"]
    assert res[1]["parts"] == [] and res[1]["text"].startswith("Turn")
    assert res[2]["parts"] == [] and res[2]["handle"] == ["a"]          # "also handle", not moved or dropped
    assert sorted(n for s in res for n in s["parts"]) == ["a", "b", "c"]          # every part exactly once
    assert warn == [] and all(s["warnings"] == [] for s in res)


def test_unknown_parts_kept_and_warned(capsys):
    A = ns([dict(title="Ghost", text="x", parts=["nope", "a"], subsystem="frame", stage=100),
            dict(title="All ghost", text="y", parts=["zip"], subsystem="frame", stage=101)], {"a": "frame"}, ["frame"])
    res, warn = plan_steps(A, contacts=None, bands=BANDS)
    assert titles(res) == ["Ghost", "All ghost"]
    assert res[0]["parts"] == ["a"] and "nope" in res[0]["warnings"][0]
    assert res[1]["parts"] == [] and "zip" in res[1]["warnings"][0]
    assert len(warn) == 2 and "WARNING" in capsys.readouterr().out


def test_strict_stage_order_labels_and_no_stage():
    A = ns([dict(title="frame late", parts=["f"], subsystem="frame", stage=110),
            dict(title="no stage", parts=["g"], subsystem="frame"),
            dict(title="reel early", parts=["r"], subsystem="reel", stage=150),
            dict(title="drum same", parts=["d"], subsystem="drum", stage=110),          # outside drum's bands
            dict(title="drum test", parts=["e"], subsystem="drum", stage=999),          # any subsystem may
            dict(title="frame first", parts=["h"], subsystem="frame", stage=100)],
           {"f": "frame", "g": "frame", "r": "reel", "d": "drum", "e": "drum", "h": "frame"},
           ["drum", "frame", "reel"])
    res, warn = plan_steps(A, contacts=None, log=quiet, bands=BANDS)
    # stage, then subsystem order (frame before drum at 110), steps without a stage last
    assert titles(res) == ["frame first", "frame late", "drum same", "reel early", "drum test", "no stage"]
    assert [s["label"] for s in res] == ["deck 100", "motor 110", "motor 110", "reel_capstan 150",
                                         "test_stage_only 999", ""]
    assert heading(2, res[1]) == "Step 2 (motor 110): frame late" and res[1]["heading"] == heading(2, res[1])
    assert len(warn) == 2
    assert "band motor" in res[2]["warnings"][0] and "drum_core" in res[2]["warnings"][0]
    assert res[-1]["warnings"][0].startswith("no stage")


def test_stage_in_no_band_and_shared_band():
    A = ns([dict(title="nowhere", parts=["a"], subsystem="frame", stage=5),
            dict(title="guard f", parts=["b"], subsystem="frame", stage=230),
            dict(title="guard d", parts=["c"], subsystem="drum", stage=231),
            dict(title="demo", parts=["x"], subsystem="demo")],                         # owns no band: no stage ok
           {"a": "frame", "b": "frame", "c": "drum", "x": "demo"}, ["drum", "demo", "frame"])
    res, warn = plan_steps(A, contacts=None, log=quiet, bands=BANDS)
    assert len(warn) == 1 and "stage 5 is in no ASSEMBLY_STAGES band" in warn[0]
    assert titles(res) == ["nowhere", "guard f", "guard d", "demo"]


def test_bands_from_params_owner_and_guess():
    P = types.SimpleNamespace(ASSEMBLY_STAGES=dict(
        deck=(100, 109, "cad-frame", "x"), guards=(230, 239, "cad-frame / cad-launch", "x"),
        drum_core=(130, 139), reel_storage=(180, 189, "", "x"), aim=(200, 209),
        test_stage_only=(900, 999, "all", "x")))
    B = stage_bands(P)
    assert B["deck"] == (100, 109, {"frame"}) and B["guards"][2] == {"frame", "launch"}
    assert B["drum_core"][2] == {"drum"} and B["reel_storage"][2] == {"reel"} and B["aim"][2] == {"launch"}
    assert B["test_stage_only"][2] is None
    real = stage_bands(load_params())
    if real:                                    # params 1.3.4: every band maps to an owner
        assert all(v[2] is None or v[2] for v in real.values()), real


def test_dependency_is_validated_not_reordered():
    A = ns([dict(title="lid", parts=["lid"], subsystem="frame", stage=100),
            dict(title="base", parts=["base"], subsystem="frame", stage=101)],
           {"lid": "frame", "base": "frame", "float": "frame"}, ["frame"])
    C = {"lid": {"base"}, "base": {"lid"}, "float": set()}
    res, warn = plan_steps(A, contacts=C, grounded={"base"}, log=quiet, bands=BANDS)
    assert titles(res)[:2] == ["lid", "base"]                                  # strict stage order kept
    assert "rest only on parts placed later (by 'base')" in res[0]["warnings"][0]
    assert any("touches nothing" in x for x in res[-1]["warnings"])           # the auto step of the loose part
    assert len(warn) == 2


def deck_drum():
    deck = block("frame_deck", -50, -50, -10, 100, 100, 10)            # top at z = 0
    drum = block("drum_drum", 0, 0, 0)                                 # sits on the deck
    lid = block("drum_lid", 0, 0, 10)                                  # sits on the drum
    cap = block("drum_cap", 0, 0, 20, 4, 4, 2)                         # in no step, sits on the lid
    mods = {"drum": mod([drum, lid, cap], [dict(title="Fit the drum", text="", parts=["drum_drum"], stage=130),
                                           dict(title="Fit the lid", text="", parts=["drum_lid"], stage=131),
                                           dict(title="Check", text="Turn it by hand.", parts=[], stage=132)]),
            "frame": mod([deck], [dict(title="Deck", text="", parts=["frame_deck"], stage=100)],
                         grounded=["frame_deck*"])}
    return Assembly(load_params(), mods)


def test_geometry_contacts_and_auto_step_insertion():
    asm = deck_drum()
    res, warn = plan_steps(asm, log=quiet, bands=BANDS)        # contacts computed from the posed solids
    assert titles(res) == ["Deck", "Fit the drum", "Fit the lid", "Fit the remaining drum parts", "Check"]
    assert res[3]["auto"] and res[3]["parts"] == ["drum_cap"] and res[3]["label"] == ""
    assert not warn
    # deck staged after the drum: kept in stage order, the drum is warned (it sits on the deck, placed later)
    asm.steps[-1]["stage"] = 135
    res, warn = plan_steps(asm, log=quiet, bands=dict(BANDS, deck=(100, 139, {"frame"})))
    assert titles(res)[0] == "Fit the drum" and "frame_deck" not in res[0]["parts"]
    assert any("rest only on parts placed later (by 'Deck'" in x for x in res[0]["warnings"])


def test_plan_is_stable_when_replanned():
    """The plan written back to asm.json (steps with seq) gives the same list without geometry (tools.guide)."""
    asm = deck_drum()
    res, _ = plan_steps(asm, log=quiet, bands=BANDS)
    parts = {n: p.subsystem for n, p in asm.parts.items()}
    again, warn = plan_steps(ns(res, parts, list(asm.modules)), contacts=None, log=quiet, bands=BANDS)
    key = ("title", "parts", "handle", "label", "heading")
    assert [[s[k] for k in key] for s in again] == [[s[k] for k in key] for s in res]
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
    for s in res:
        print(s["heading"], s["parts"], ("handle " + ",".join(s["handle"])) if s["handle"] else "")
    for x in warn:
        print("WARNING", x)
    assert len(res) == len(asm.steps) + sum(s["auto"] for s in res)
    assert sorted(n for s in res for n in s["parts"]) == sorted(asm.parts)
    stages = [s["stage"] for s in res if s["stage"] is not None]
    assert stages == sorted(stages)
    assert not warn, "assembly step warnings:\n" + "\n".join(warn)
