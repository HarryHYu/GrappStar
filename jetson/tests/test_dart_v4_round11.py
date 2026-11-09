"""fw-dart review round 11: /arm refused while ANCHORED; v4.3: NOT_UPRIGHT (the dart could not right itself) after /auto."""
import urllib.request

from mission_harness import Rig


def test_mock_refuses_arm_while_anchored(tmp_path):
    r = Rig(tmp_path)
    try:
        with r.md.lock:
            r.md.seq = "anchored"
        assert r.md.phase == "ANCHORED"
        url = f"http://127.0.0.1:{r.md.port}/arm?on=1&auto=0"
        try:
            urllib.request.urlopen(url, timeout=2)
            raise AssertionError("arm accepted while anchored")
        except urllib.error.HTTPError as e:
            assert e.code == 409 and e.read().decode() == "refused: anchored: release first"
    finally:
        r.close()


def anchor_at_cock(r):
    """Preflight refuses an anchored dart; this models one that anchors after it (key A, a bench hand): the
    first arm request finds the dart ANCHORED."""
    real_arm = r.dart.arm
    first = {"done": False}
    def arm(on=True, auto=True):
        if on and not first["done"]:
            first["done"] = True
            with r.md.lock:
                r.md.seq = "anchored"
        elif on and r.world.dart == "free":
            r.world.dart = "seated"       # the harness world: a dart released on the launcher is still seated
        return real_arm(on, auto)
    r.dart.arm = arm


def test_arm_while_anchored_releases_first_then_fires(tmp_path):
    r = Rig(tmp_path)
    anchor_at_cock(r)
    seen = []
    def release(p):
        seen.append(list(p.choices))
        return "Release the dart"
    r.policy["is ANCHORED"] = release
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
        assert seen == [["Release the dart", "Abort"]]
        paths = [q[1] for q in r.md.requests]
        i_rel = paths.index("/release")
        assert "/arm" in paths[i_rel:]                  # armed again after the release
        assert "Continue without" not in " ".join(a[1] for a in r.answers)
    finally:
        r.close()


def test_arm_while_anchored_abort_never_fires(tmp_path):
    r = Rig(tmp_path)
    anchor_at_cock(r)
    r.policy["is ANCHORED"] = "Abort"
    try:
        m = r.run(60)
        assert m.state != "DONE"
        assert "FIRE" not in r.steps()
    finally:
        r.close()


def test_not_upright_offers_release_reshoot_or_force(tmp_path):
    """Dart v4.3: the righting gives up after 6 moves (fault NOT_UPRIGHT): the operator chooses from the camera picture
    between Release and reshoot and Force anchor (no Retry: a new /auto would try the same moves again)."""
    r = Rig(tmp_path)
    r.md.orient, r.md.cannot_right = "inverted", True
    seen = []
    def release(p):
        seen.append(list(p.choices))
        r.md.orient, r.md.cannot_right = "upright", False   # set right by the reshoot
        return "Release and reshoot"
    r.policy["NOT_UPRIGHT"] = release
    try:
        m = r.run(120)
        assert m.state == "DONE", m.error
        assert seen == [["Release and reshoot", "Force anchor", "Abort"]]
        assert m.reshoots == 1 and r.steps().count("ANCHOR") == 2
        paths = [q[1] for q in r.md.requests]
        assert paths.count("/auto") == 2
    finally:
        r.close()
