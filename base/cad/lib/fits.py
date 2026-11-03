"""Fit allowances for the Bambu X2D (0.4 nozzle). One place to tune after Harry prints the fit coupon.
All values are diametral (added to the nominal diameter) unless named _R (radial)."""

# holes and pins (PLA, holes printed vertical)
HOLE_CLEAR = 0.35        # free running / clearance on a bolt or shaft (diametral)
HOLE_SLIDE = 0.25        # sliding fit (pin that must move by hand)
HOLE_PRESS = -0.10       # light press of a steel part into a printed bore (smooth bore, no ribs)
HOLE_HORIZ_EXTRA = 0.15  # extra for holes printed horizontal (sag on the roof)

# 608 bearing seats with crush ribs
BRG_RIB_BORE = 0.30      # bore oversize between the ribs (bearing never touches the wall)
BRG_RIB_INTERF = 0.15    # each rib reaches this far inside the nominal OD (radial interference)

# printed-on-printed
SHAFT_IN_BORE = 0.30     # printed shaft running in a printed bore (diametral)
KEY_CLEAR = 0.20         # key in keyway, each side
GEAR_BACKLASH = 0.15     # circumferential backlash at the pitch circle (mm), shared between the pair
GEAR_CENTER_EXTRA = 0.10 # added to gear centre distances
LAYER = 0.2              # layer height used for heights of bridges and sacrificial layers

# fasteners
M3_CLEAR_D = 3.4
M3_CLOSE_D = 3.2
M3_TAP_D = 2.6           # self tapping into PLA
M3_INSERT_D = 4.0        # M3x4x5 heat-set insert hole (CNC Kitchen style: 4.0 top, 4.0 straight)
M3_INSERT_L = 5.0
M3_HEAD_D = 5.5 + 0.6    # SHCS head 5.5, counterbore 6.1
M3_HEAD_H = 3.0
M3_NUT_AF = 5.5 + 0.3    # nut trap across flats
M3_NUT_H = 2.4 + 0.3
M3_NYLOC_H = 4.0 + 0.3
M8_CLEAR_D = 8.5
M8_NUT_AF = 13.0 + 0.4
M8_NUT_H = 6.5 + 0.3
