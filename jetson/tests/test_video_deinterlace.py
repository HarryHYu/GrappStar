import numpy as np
import pytest

from grappstar.video.deinterlace import deinterlace, normalize_mode, MODES


def comb(h=480, w=720):
    """Even rows 0, odd rows 200: the worst possible combing."""
    img = np.zeros((h, w, 3), np.uint8)
    img[1::2] = 200
    return img


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("h", [480, 479, 576, 2])
def test_shapes_preserved(mode, h):
    img = comb(h, 64)
    out = deinterlace(img, mode)
    assert out.shape == img.shape and out.dtype == np.uint8


def test_modes_values():
    img = comb(8, 4)
    assert deinterlace(img, "none") is img
    assert (deinterlace(img, "bob") == 0).all()          # even field doubled
    assert (deinterlace(img, "blend")[:-1] == 100).all()  # neighbours averaged
    lin = deinterlace(img, "linear")
    assert (lin[0::2] == 0).all() and (lin[1::2] == 0).all()


def test_linear_interpolates_gradient():
    ramp = np.repeat(np.arange(0, 200, 10, dtype=np.uint8)[:, None, None], 3, axis=2)
    ramp = np.repeat(ramp, 5, axis=1)
    lin = deinterlace(ramp, "linear")
    assert np.array_equal(lin[1:-1], ramp[1:-1])


def test_normalize_mode():
    assert normalize_mode(False) == "none"
    assert normalize_mode(True) == "linear"
    assert normalize_mode("BOB") == "bob"
    with pytest.raises(ValueError):
        normalize_mode("weave")
