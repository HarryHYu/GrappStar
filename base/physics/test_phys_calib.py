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


def test_fit_v_scale_recovers_factor():
    from calib import fit_v_scale
    from common import Params
    from flight import simulate
    from launcher import effective_muzzle
    p = Params()
    k = 0.92
    shots = []
    for th in (20.0, 36.0):
        v = effective_muzzle(p, th)["v"] * k            # the speed ballistics.py scales
        shots.append((th, 0.5, simulate(p, v, th, brake_n=0.5, reel_model=True)["range_m"]))
    r = fit_v_scale(shots, p)
    assert abs(r["v_scale"] - k) < 0.005


def test_wiper_spec_names():
    from calib import wiper_spec
    w = wiper_spec(wiper_block(12.0, 44, 1.4, 3.0, 100, 17))
    assert set(w) == {"n0_rpm", "stall_nm", "stall_a", "free_a"}
    assert w["stall_nm"] == pytest.approx(2.943)


def test_spring_verdict():
    from calib import spring_force, spring_verdict
    assert spring_force(dict(f_init_n=7.0, rate_n_mm=0.125), 265.0) == pytest.approx(40.125)
    assert spring_verdict(38.0) == "OK"
    assert spring_verdict(39.0) == "over catalogue, accept for low cycles"
    assert spring_verdict(42.0).startswith("too high")


def test_error_messages_per_subcommand():
    from calib import main
    with pytest.raises(SystemExit) as e:
        main(["spring", "165.1", "1.0:187"])
    assert "calib spring" in str(e.value) and "kg" in str(e.value)
    with pytest.raises(SystemExit) as e:
        main(["range", "36:0.5"])
    assert "calib range" in str(e.value) and "mass" not in str(e.value) and "coils" not in str(e.value)


def test_range_prints_spec_hint(capsys, monkeypatch):
    import calib
    monkeypatch.setattr(calib, "fit_v_scale", lambda shots: dict(v_scale=0.95, rows=[], rms_m=0.0))
    calib.main(["range", "36:0.5:4.0"])
    assert "put LAUNCH v_scale = 0.950 in base/spec/params.py, then ./run_all.sh" in capsys.readouterr().out
