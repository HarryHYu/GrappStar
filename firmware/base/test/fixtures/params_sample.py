"""Sample params.py for the generator test: every value shape the generator handles."""
import math
VERSION = "test-1"
BED = (256.0, 256.0, 260.0)
WIPER = dict(stall_a=18.0, free_a=1.5, n0_rpm=45.0, measured=False, note="12 V", holes=None)
SPRAG = dict(roles={1: "cock", 2: "capstan"}, torque_nm=50)
NAMES = ["a", "b"]
STEPS = [dict(t=0.0, what="x", v=1), dict(t=1.5, what="y", v=2)]
GRID = [(0, 1), (2, 3)]
MIXED = [1, "a"]
AIM = dict(crank=20.0, coupler=80.0, rocker=40.0, ground=90.0, ratio=30.0, rocker_to_pitch=(-60.0, 1.0))
def helper(x):
    return x
