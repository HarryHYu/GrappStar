"""Physics tests: import this directory's modules (not same-named modules elsewhere in the repo, e.g.
base/cad/parts/launch.py) and fail if any test writes to committed outputs or to kedge/."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
MINE = ("common", "launch", "flight", "payout", "capstan", "drivetrain", "gears", "ballistics", "sensitivity",
        "summary", "calib", "attitude", "recoil", "retrieve", "spec_bridge")

if HERE in sys.path:
    sys.path.remove(HERE)
sys.path.insert(0, HERE)
for name in MINE:
    m = sys.modules.get(name)
    if m is not None and not os.path.abspath(getattr(m, "__file__", "") or "").startswith(HERE):
        del sys.modules[name]

WATCH = [os.path.join(ROOT, "base", "out"), os.path.join(ROOT, "kedge", "out30")]


def _snapshot():
    snap = {}
    for d in WATCH:
        for dp, _, fs in os.walk(d):
            for f in fs:
                p = os.path.join(dp, f)
                try:
                    snap[p] = os.stat(p).st_mtime_ns
                except OSError:
                    pass
    return snap


@pytest.fixture(autouse=True, scope="session")
def _no_writes_to_outputs():
    before = _snapshot()
    yield
    after = _snapshot()
    changed = [p for p in after if before.get(p) != after[p]]
    assert not changed, f"physics tests wrote committed files: {changed[:5]}"
