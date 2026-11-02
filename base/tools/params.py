"""Params loader: base/spec/params.py when it exists, else the stub. load_params() -> module."""
import importlib
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)


def load_params():
    try:
        P = importlib.import_module("spec.params")
    except ModuleNotFoundError:
        from tools import params_stub as P
    from tools import params_stub as D
    for k in dir(D):            # fill the few names the tools rely on
        if k.isupper() and not hasattr(P, k):
            setattr(P, k, getattr(D, k))
    return P
