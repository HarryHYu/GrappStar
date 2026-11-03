import pytest

from calib import fit_spring, wiper_block


def test_fit_recovers_rate_and_f0():
    k, f0, L0 = 0.125, 7.0, 165.1
    pairs = [(m, L0 + (m * 9.81 - f0) / k) for m in (1.0, 2.0, 3.0)]
    r = fit_spring(L0, pairs)
    assert r["rate_n_mm"] == pytest.approx(k, rel=1e-9)
    assert r["f_init_n"] == pytest.approx(f0, rel=1e-9)


def test_fit_drops_unopened_points():
    pairs = [(0.5, 165.1), (1.0, 187.0), (2.0, 265.5)]
    r = fit_spring(165.1, pairs)
    assert len(r["points"]) == 2


def test_wiper_block():
    w = wiper_block(12.0, 44, 1.4, 3.0, 100, 17)
    assert w["t_stall"] == pytest.approx(2.943)
