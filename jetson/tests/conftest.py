"""Test setup: import path, a per-test time limit (no extra plugin needed), and the "slow" marker.

    python3 -m pytest                  # the fast suite (default, under 10 minutes on one core)
    python3 -m pytest -m slow          # only the slow ones (real firmware simulators, full app run)
    ./check_all.sh                     # everything
Time limit per test: GRAPPSTAR_TEST_TIMEOUT seconds (default 180, slow tests 600).
"""
import os
import signal
import sys
import threading

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULT_LIMIT = float(os.environ.get("GRAPPSTAR_TEST_TIMEOUT", "180"))


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: long tests against the real firmware simulators (run by check_all.sh)")


@pytest.fixture(autouse=True)
def _time_limit(request):
    """Fail a test that runs too long instead of hanging the whole suite."""
    if not hasattr(signal, "SIGALRM") or threading.current_thread() is not threading.main_thread():
        yield
        return
    limit = DEFAULT_LIMIT * (10 / 3 if request.node.get_closest_marker("slow") else 1)

    def boom(signum, frame):
        raise TimeoutError(f"test exceeded its {limit:.0f} s limit")
    old = signal.signal(signal.SIGALRM, boom)
    signal.setitimer(signal.ITIMER_REAL, limit)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)
