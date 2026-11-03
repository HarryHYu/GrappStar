import numpy as np
import pytest

from grappstar.sim.terrain_textures import CLASSES, texture


@pytest.mark.parametrize("cls", CLASSES)
def test_texture_shape_dtype(cls):
    img = texture(cls, 120, 200, seed=3)
    assert img.shape == (120, 200, 3)
    assert img.dtype == np.uint8


def test_texture_deterministic_and_seeded():
    a = texture("loose_soil", 100, 100, seed=5)
    b = texture("loose_soil", 100, 100, seed=5)
    c = texture("loose_soil", 100, 100, seed=6)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_texture_noise_and_analog():
    clean = texture("sand_dust", 160, 160, seed=1).astype(float)
    noisy = texture("sand_dust", 160, 160, seed=1, noise=0.06).astype(float)
    assert np.abs(noisy - clean).mean() > 5
    ana = texture("hard_ground", 160, 160, seed=1, noise=0.02, analog=True)
    assert ana.shape == (160, 160, 3)


def test_texture_class_character():
    veg = texture("vegetation", 200, 200, seed=2).astype(float)
    assert veg[..., 1].mean() > veg[..., 2].mean()          # green dominates red
    sand = texture("sand_dust", 200, 200, seed=2).astype(float)
    soil = texture("loose_soil", 200, 200, seed=2).astype(float)
    assert sand.mean() > soil.mean() + 40                    # sand is pale, soil is dark
    void = np.mean([texture("water_void", 120, 120, seed=s).mean() for s in range(6)])
    assert void < 90


def test_texture_bad_class():
    with pytest.raises(ValueError):
        texture("lava", 10, 10)
