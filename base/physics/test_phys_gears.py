import math

import pytest

from common import PLA, PPA_CF
import printed_gears as gears


def test_y_table():
    assert gears.lewis_y(20) == pytest.approx(0.322)
    assert gears.lewis_y(23) == pytest.approx((0.331 + 0.337) / 2)
    with pytest.raises(ValueError):
        gears.lewis_y(10)


def test_lewis_hand():
    # 2 N m, m 2, z 20, b 10 mm: d 40 mm, Wt 100 N, sigma = 100 / (0.010 * 0.002 * 0.322) = 15.53 MPa
    r = gears.lewis(2.0, 2.0, 20, 10.0, PLA, rpm=0)
    assert r["wt_n"] == pytest.approx(100.0)
    assert r["kv"] == 1.0
    assert r["sigma_mpa"] == pytest.approx(15.53, abs=0.01)
    assert r["sf"] == pytest.approx(12 / 15.53, abs=0.005)
    assert not r["pass"]


def test_barth():
    # 60 rpm on 40 mm pitch: V = 0.1257 m/s, Kv = 1.0206
    r = gears.lewis(2.0, 2.0, 20, 10.0, PPA_CF, rpm=60)
    assert r["v_pitch"] == pytest.approx(math.pi * 0.04, rel=1e-6)
    assert r["kv"] == pytest.approx(1.0206, abs=1e-4)


def test_shaft_torsion():
    r = gears.shaft_torsion(12.0)
    assert r["tau_nom_mpa"] == pytest.approx(16 * 12 / (math.pi * 0.02 ** 3) / 1e6)   # 7.64
    assert r["pass"] and not r["strict_pass"] and not r["endmill_pass"]
    assert r["t_allow_kt"] == pytest.approx(9.42, abs=0.01)
    assert r["t_allow_endmill"] == pytest.approx(6.28, abs=0.01)
    assert not gears.shaft_torsion(12.5)["pass"]
    r = gears.shaft_torsion(8.0)
    assert r["strict_pass"] and not r["endmill_pass"]


def test_key_steel():
    # 6 N m on 20 mm: F 600 N. Bearing on the 3 mm keyway walls: 600 / (3 x 14) = 14.29 MPa.
    # Shaft ligament behind the keyway: sqrt(10^2 - 7^2) - 3 = 4.14 mm -> 600 / (4.14 x 14) = 10.35 MPa
    r = gears.key(6.0, 14.0)
    m = r["modes"]
    assert "shear_key" not in m
    assert r["ligament_shaft_mm"] == pytest.approx(math.sqrt(51) - 3, abs=1e-6)
    assert m["bearing_shaft"]["stress_mpa"] == pytest.approx(14.286, abs=0.01)
    assert m["bearing_hub"]["stress_mpa"] == pytest.approx(14.286, abs=0.01)
    assert m["shear_shaft"]["stress_mpa"] == pytest.approx(600 / (4.1414 * 14), abs=0.01)
    assert r["governing"] == "shear_shaft"
    assert r["pass"]
    assert r["t_max"] == pytest.approx(6.0 * 12 / m["shear_shaft"]["stress_mpa"], rel=1e-6)
    assert not gears.key(12.0, 14.0)["pass"]


def test_key_printed():
    r = gears.key(6.0, 14.0, key_material="printed", hub_printed=False)
    assert r["modes"]["shear_key"]["stress_mpa"] == pytest.approx(7.143, abs=0.01)
    assert "bearing_hub" not in r["modes"]


def test_bearing_sprag():
    b = gears.bearing608(685.0, 100)
    assert b["s0"] == pytest.approx(2.0)
    assert b["pass"]
    assert not gears.bearing608(100, 40000)["pass"]
    assert gears.sprag(49, 100)["pass"]
    assert not gears.sprag(51, 100)["pass"]
    assert not gears.sprag(10, 7000)["pass"]
