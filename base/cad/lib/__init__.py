"""Shared CAD library for the GrappStar base (API v1, see base/cad/README.md)."""
import importlib

from .core import Part, Joint, Allow, Clearance, Mesh, MATERIALS
from . import tf, prim, fits

_LAZY = ("gears", "worm", "screw", "clutch", "bearings", "shafts", "fasteners", "clips", "switch", "motors",
         "line", "springs", "cams")


def __getattr__(name):
    if name in _LAZY:
        mod = importlib.import_module(f".{name}", __name__)
        globals()[name] = mod
        return mod
    raise AttributeError(name)


__all__ = ["Part", "Joint", "Allow", "Clearance", "Mesh", "MATERIALS", "tf", "prim", "fits", *_LAZY]
