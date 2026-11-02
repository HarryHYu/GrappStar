"""Overlay base/spec/params.py onto the physics defaults. Filled in once the spec lands (names mapped here only)."""
import os
import importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = os.path.join(HERE, "..", "spec", "params.py")


def load_spec_module(path=None):
    path = path or SPEC
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("base_spec_params", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def apply(p, path=None):
    mod = load_spec_module(path)
    if mod is None:
        return p
    p.source = "base/spec/params.py"
    return p
