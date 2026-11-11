from build123d import *
import sys, math
PK = sys.argv[1]
CLR = 0.2
def bx(x0,x1,y0,y1,z0,z1): return Pos((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)*Box(x1-x0,y1-y0,z1-z0)
H = 4.0
plate = bx(0, 98, 0, 18, 0, H)
cuts = [bx(3, 3+12.4, 3, 3+12.4, -1, H+1),                       # worm N20 square pocket (either face)
        bx(19, 19+12.4, 3, 3+10.4, -1, H+1),                       # drill motor pocket 12.4 x 10.4
        Pos(40, 9, 0)*Cylinder(8.15/2, 2*H+2), Pos(40, 9, H-0.65)*Cylinder(9.8/2, 1.4, align=(Align.CENTER,Align.CENTER,Align.MIN)),   # F693ZZ seat + flange recess
        Pos(53, 9, 0)*Cylinder(7.4/2, 2*H+2),                      # coupler bore in the hull boss (plain)
        Pos(62, 9, 0)*(Cylinder(3.25/2, 2*H+2) - bx(-3, 3, 3.25/2-0.5, 3, -H-2, H+2)),   # D bore for the N20 shaft (hub)
        Pos(70, 9, 0)*Cylinder(2.5/2, 2*H+2), Pos(76, 9, 0)*Cylinder(3.3/2, 2*H+2),     # M3 pilot (self-tap) and clearance
        Pos(84, 9, 0)*(Cylinder(7.25/2, 2*H+2) - bx(-4, 4, 7.25/2-0.6, 4, -H-2, H+2)),   # pod head D bore for the coupler
        Pos(93, 9, 0)*Cylinder(3.8/2, 2*H+2)]                     # drill shaft clearance 3.8
for c in cuts: plate = plate - c
peg = Cylinder(3.5, 14, align=(Align.CENTER,Align.CENTER,Align.MIN)) - bx(-4,4,3.5-0.6,4,-1,15)
peg = peg - (Cylinder(3.2/2, 9.4, align=(Align.CENTER,Align.CENTER,Align.MIN)) - bx(-2,2,1.6-0.5,2,-1,10))
peg = peg - Pos(0,0,14-7)*Cylinder(1.25, 7.1, align=(Align.CENTER,Align.CENTER,Align.MIN))
peg = Pos(30, 35, 0)*peg
bore = Pos(70, 50, 0)*(Cylinder(21, 4, align=(Align.CENTER,Align.CENTER,Align.MIN)) - Cylinder(19.5, 7, align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(Location((0,0,-1))))
spig = Pos(70, 50, 0)*(Cylinder(19.3, 4, align=(Align.CENTER,Align.CENTER,Align.MIN)) - Cylinder(17.8, 7, align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(Location((0,0,-1))))
spig = spig.moved(Location((-48, 0, 0)))
# v31 row: JST-XH 2-pin and 3-pin header seats (the tail cap slots), USB-C plug window (hull flank), pod cap + counterbore
row = bx(0, 98, 22, 34, 0, H)
for cut in (bx(3, 3 + 6.1, 24, 24 + 7.7, -1, H + 1),            # JST-XH 2-pin header seat (power loop, tail top)
            bx(14, 14 + 6.1, 23, 23 + 10.2, -1, H + 1),         # JST-XH 3-pin header seat (charge, tail bottom)
            bx(26, 26 + 12.5, 24.5, 24.5 + 7.0, -1, H + 1),     # USB-C overmold window: your cable must plug through it
            Pos(50, 28, H - 1.2) * Cylinder(10.4 / 2, 2.4, align=(Align.CENTER, Align.CENTER, Align.MIN)),   # pod head counterbore
            Pos(50, 28, 0) * (Cylinder(7.25 / 2, 2 * H + 2) - bx(-4, 4, 7.25 / 2 - 0.6, 4, -H - 2, H + 2))):  # pod head D-bore
    row = row - cut
cap = Pos(70, 28, 0) * (Cylinder(5.0, 1.5, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(3.3 / 2, 4, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, -1))))
# v34 row (PLA, print FIRST): the press fits. Arm-head D bore 7.05 (flat 2.925 from the axis) and cheek D bore 6.05 (flat 2.525) take the two
# PPA-CF test pegs below; the 3-pin charge seat 6.05 x 10.2 and the 2-pin seat 6.05 x 7.9 take the JST headers.
def dbore(d, flat, x, y):
    return Pos(x, y, 0) * (Cylinder(d / 2, 2 * H + 2) - bx(-d, d, flat, d, -H - 2, H + 2))
def rrect(w, h, x, y, r=0.4):
    return Pos(x, y, 0) * extrude(RectangleRounded(w, h, r), amount=2 * H + 2).moved(Location((0, 0, -H - 1)))
row34 = bx(0, 98, 38, 54, 0, H)
for cut in (dbore(7.05, 3.525 - 0.6, 10, 46), dbore(6.05, 3.025 - 0.5, 24, 46), rrect(6.05, 10.2, 40, 46), rrect(6.05, 7.9, 52, 46)):
    row34 = row34 - cut
# PPA-CF test pegs (fit_coupon_pegs.stl, plate C): Ø7 D peg (flat 2.9, like the pod coupler) and Ø6 D peg (flat 2.5, like the leg coupler),
# each 10 long with the press-fit stub socket Ø3.05, flat 1.03 from the axis, 6 deep: push the real N20 stub in
def peg34(d, flat, x):
    pg = Cylinder(d / 2, 10, align=(Align.CENTER, Align.CENTER, Align.MIN)) - bx(-d, d, flat, d, -1, 11)
    so = Pos(0, 0, 10 - 6) * (Cylinder(3.05 / 2, 6.1, align=(Align.CENTER, Align.CENTER, Align.MIN)) - bx(-2, 2, 1.03, 2, -1, 8))
    return Pos(x, 0, 0) * (pg - so)
pegs = Compound(children=[peg34(7.0, 2.9, 0), peg34(6.0, 2.5, 12)])
# v34 coupon, laid out with NO overlaps (the old layout piled the v31 / v34 rows onto the rings and the old peg):
#  bar A (y 0..18): worm N20 square pocket, drill motor pocket, pivot-block coupler bore 7.4, leg coupler hull bore 6.3, skin plug
#        counterbore 7.6, M3 pilot 2.5 (radial screws, pod caps), M3 clearance 3.3
#  bar B (y 24..40): press-fit D bores 7.05 (arm head) and 6.05 (cheek) for the two PPA-CF pegs, 3-pin and 2-pin JST seats, USB-C window
#  rings (y 48..90): hull bore Ø39 / nose spigot Ø38.6, side by side, apart
barA = bx(0, 98, 0, 18, 0, H)
for cut in (bx(3, 3 + 12.4, 3, 3 + 12.4, -1, H + 1), bx(19, 19 + 12.4, 3, 3 + 10.4, -1, H + 1),
            Pos(41, 9, 0) * Cylinder(7.4 / 2, 2 * H + 2), Pos(52, 9, 0) * Cylinder(6.3 / 2, 2 * H + 2), Pos(63, 9, 0) * Cylinder(7.6 / 2, 2 * H + 2),
            Pos(74, 9, 0) * Cylinder(2.5 / 2, 2 * H + 2), Pos(83, 9, 0) * Cylinder(3.3 / 2, 2 * H + 2)):
    barA = barA - cut
barB = bx(0, 98, 24, 40, 0, H)
for cut in (dbore(7.05, 3.525 - 0.6, 10, 32), dbore(6.05, 3.025 - 0.5, 23, 32), rrect(6.05, 10.2, 36, 32), rrect(6.05, 7.9, 48, 32),
            rrect(12.5, 7.0, 66, 32)):
    barB = barB - cut
ringB = Pos(25, 69, 0) * (Cylinder(21, H, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(19.5, 7, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, -1))))
ringS = Pos(73, 69, 0) * (Cylinder(19.3, H, align=(Align.CENTER, Align.CENTER, Align.MIN)) - Cylinder(17.8, 7, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, -1))))
c = Compound(children=[barA, barB, ringB, ringS])
export_stl(c, f"{PK}/fit_coupon.stl", tolerance=0.02, angular_tolerance=0.15); export_step(c, f"{PK}/fit_coupon.step")
export_stl(pegs, f"{PK}/fit_coupon_pegs.stl", tolerance=0.01, angular_tolerance=0.1); export_step(pegs, f"{PK}/fit_coupon_pegs.step")
print("coupon", c.volume/1000, "cm3; pegs", pegs.volume / 1000, "cm3")
