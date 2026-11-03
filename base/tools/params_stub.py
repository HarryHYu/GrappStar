"""Stand-in for base/spec/params.py until the coordinator ships it. Only names the tools themselves need.
Subsystem workers: read params with getattr(P, NAME, default) so this stub never breaks you."""

BED = (256.0, 256.0, 260.0)     # Bambu X2D build volume (mm)
DRUM_POS = {"LAUNCH": 0.0, "NEUTRAL": 60.0, "REEL": 120.0, "WRAPPED": 300.0}   # selector drum angles (deg), placeholder
WIPER_RPM = 45.0                # placeholder until Harry measures the motor
LINE_M = 20.0
