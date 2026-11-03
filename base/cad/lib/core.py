"""Core dataclasses of the base CAD API v1 (see base/cad/README.md). Frozen: additive changes only."""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from fnmatch import fnmatchcase

import numpy as np
from manifold3d import Manifold

from . import tf

MATERIALS = {
    # name: density g/cm3, printed?
    "PLA": (1.24, True),
    "PPA-CF": (1.25, True),
    "TPU": (1.21, True),
    "PETG": (1.27, True),
    "steel": (7.85, False),
    "bought": (None, False),
}
NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass
class Joint:
    name: str
    parent: str | None = None
    kind: str = "fixed"            # rev | slide | fixed
    origin: tuple = (0.0, 0.0, 0.0)
    axis: tuple = (0.0, 0.0, 1.0)
    base: np.ndarray | None = None
    limits: tuple | None = None
    gear: tuple | None = None      # (other_joint, ratio, offset)

    def __post_init__(self):
        if self.kind not in ("rev", "slide", "fixed"):
            raise ValueError(f"joint {self.name}: kind must be rev, slide or fixed, got {self.kind!r}")
        if not NAME_RE.match(self.name):
            raise ValueError(f"joint name {self.name!r} must be snake_case")
        a = np.asarray(self.axis, float)
        n = np.linalg.norm(a)
        if n < 1e-9:
            raise ValueError(f"joint {self.name}: zero axis")
        self.axis = tuple(a / n)
        self.origin = tuple(float(v) for v in self.origin)

    def local(self, value: float) -> np.ndarray:
        """Transform joint frame -> parent frame at this joint value."""
        M = tf.T(self.origin)
        if self.base is not None:
            M = M @ np.asarray(self.base, float)
        if self.kind == "rev":
            M = M @ tf.R(self.axis, value)
        elif self.kind == "slide":
            M = M @ tf.T(*(np.asarray(self.axis) * value))
        return M


@dataclass
class Part:
    name: str
    solid: Manifold
    material: str = "PLA"
    qty: int = 1
    joint: str | None = None
    place: np.ndarray = field(default_factory=lambda: np.eye(4))
    print_tf: np.ndarray | None = None
    color: str = "#c8ccd2"
    subsystem: str = ""
    notes: str = ""
    bought_ref: str = ""
    infill: float = 0.4
    step: int | None = None

    def __post_init__(self):
        if not NAME_RE.match(self.name):
            raise ValueError(f"part name {self.name!r} must be snake_case")
        if self.material not in MATERIALS:
            raise ValueError(f"part {self.name}: unknown material {self.material!r} (use {list(MATERIALS)})")
        if not isinstance(self.solid, Manifold):
            raise TypeError(f"part {self.name}: solid must be a manifold3d.Manifold")
        self.place = np.asarray(self.place, float)
        if self.print_tf is not None:
            self.print_tf = np.asarray(self.print_tf, float)

    @property
    def printed(self) -> bool:
        return MATERIALS[self.material][1]

    @property
    def density(self):
        return MATERIALS[self.material][0]

    def mass_g(self, shell_mm: float = 1.6) -> float | None:
        """Estimated mass in grams. Printed parts: shell (walls) solid plus infill fraction of the core.
        Bought parts return None (give it in notes)."""
        rho = self.density
        if rho is None:
            return None
        v = self.solid.volume() / 1000.0          # cm3
        if not self.printed:
            return v * rho
        a = self.solid.surface_area() / 100.0     # cm2
        shell = min(v, a * shell_mm / 10.0)
        return (shell + (v - shell) * self.infill) * rho


@dataclass
class Allow:
    a: str
    b: str
    why: str = ""
    max_vol: float | None = None

    def match(self, x: str, y: str) -> bool:
        return (fnmatchcase(x, self.a) and fnmatchcase(y, self.b)) or (fnmatchcase(y, self.a) and fnmatchcase(x, self.b))


@dataclass
class Clearance:
    a: str
    b: str
    min_mm: float
    why: str = ""

    def match(self, x: str, y: str) -> bool:
        return (fnmatchcase(x, self.a) and fnmatchcase(y, self.b)) or (fnmatchcase(y, self.a) and fnmatchcase(x, self.b))
