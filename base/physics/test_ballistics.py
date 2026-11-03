from ballistics import build, validate
from common import Params


def test_contract_c3_shape():
    p = Params()
    p.flight.update(pitch_min=30.0, pitch_max=40.0)
    doc, meta = build(p, step=5.0, launch_over=dict(stroke=0.24))
    assert validate(doc) == []
    assert [r["pitch_deg"] for r in doc["table"]] == [30.0, 35.0, 40.0]
    for r in doc["table"]:
        assert r["range_m"][0] > r["range_m"][1] > r["range_m"][2] > 0


def test_validate_catches_extra_keys():
    doc = dict(version=1, pitch_min=0, pitch_max=1, drag_n=[0.5], table=[], extra=1)
    assert validate(doc)


def test_validate_catches_bad_rows():
    good = dict(pitch_deg=30.0, v_muzzle=7.0, range_m=[3.0, 2.9, 2.8], t_flight_s=[0.7] * 3, apex_m=[0.8] * 3)
    base = dict(version=1, pitch_min=30.0, pitch_max=30.0, drag_n=[0.5, 0.75, 1.0])
    assert validate(dict(base, table=[good])) == []
    assert validate(dict(base, table=[{k: v for k, v in good.items() if k != "pitch_deg"}]))
    assert validate(dict(base, table=[dict(good, range_m=[float("nan"), 2.9, 2.8])]))
    assert validate(dict(base, drag_n=[0.5, 1.0, 2.0], table=[good]))
