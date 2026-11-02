GrappStar reel, full size v7 (21 Sep 2026)
Sized for the 1 mm Dyneema: dia 40 capstan (8 Nm stall = 400 N), spool that holds ~25 m, 8 mm axles. Eleven printed pieces, no screws, no glue.
Regenerate with full_v7.py (CadQuery); every size is a named constant at the top. Assembled 203 x 152 x 80 mm, fits a 256 mm bed.

v5: the lug is a plain ring, dia 6.8 x 4 long, on the cup's face edge with its axis tangential, 0.5 mm in front of the face (any lower
and the dart line cuts through the face when the cup turns) and 1 mm outside the wall (so the line from the guide reaches it without
touching the cup's edge). A 45 deg block under it makes it printable. Every hole you can see is a plain round one; the D flats that
carry torque are on the axles and hidden inside (drum bore, rear half of the spool bore, the crank's lip). Spool is held by a C-clip.

PRINT (PLA, 0.2 mm, 4 walls, 30% infill, no supports; print exactly as exported)
  frame          x1   flat, on the bed. Open frame: 10 mm border, a spar down the capstan line, one across the guide line, feet on 8 mm pads
  drum           x1   base flange down
  axle_capstan   x1   lying on its flat
  axle_spool     x1   lying on its flat
  wrap_cup       x1   open end down, ring up. The cup's face is a 13.6 mm annular bridge between hub and wall, fine in PLA
  cap            x2   flat (C-clips)
  lock_pin       x1   head down
  crank          x2   flat, hub up
  spool          x1   front flange down. Rear flange is a 56 deg cone so it needs no support

ASSEMBLY (all pushes, in this order)
  Capstan: hold the drum in front of its bracket, flat up, feed the capstan axle in from behind, thin end first: through the bracket,
  through the drum (turn the drum onto the flat), then slide the cup on over the thin end, groove end toward the drum, ring up.
  Push a C-clip sideways onto the 4.5 mm neck that shows in front of the cup; the 5.6 head keeps it on.
  Snap a crank onto the back end: line up its flat with the axle's, push until it clicks (about 40 N). The lip is behind the head; it cannot walk off.
  Spool: spool in front of its bracket, hub end toward the bracket. Spool axle in from behind through the bracket and the spool (its D
  section keys into the back half of the spool), C-clip onto the neck in front, crank onto the back end.
  Lock pin: through the lock post into the boss on the cup, head outward. Snug; pull it 9 mm to free the cup.
  Every part is trapped: cranks by their lips, cup and spool by their clips, drum between bracket and cup.

LINE PATH
  spool -> guide eye -> over the drum -> into the groove on the cup -> ring on the cup -> exit eye -> dart.
  Eyes are plain dia 4.4 round holes coned out to 6.4 at each face. The guide and the spool sit 21 mm behind the capstan's base flange, so the tail leaves the drum at an angle and the turns walk toward the base.
  Launch: cup pinned, spool free, dart pulls line off the spool.  Retrieve: spool crank winds the slack.
  Wrap: pin out, turn the cup; each turn lays one turn on the capstan behind it. The dart side sweeps in front of the face and, for
  half of each turn, rides round the outside of the ring.  Tow: pin in, drum crank turns the capstan inside the still cup; turns live
  between the cup's rim and the base flange.  Unwrap: pin out, cup back, spool takes up.

CUP GROOVE
  From the rear mouth of the ring the line drops 1 mm into a groove cut into the outside of the cup: U section, 1.2 mm across the line,
  1.0 mm deep with a rounded floor, 30 deg helix, 58 deg of wrap, running out through the rim. Both ends are funnelled.
  In tow the loaded line slides through this wrap: cup friction costs roughly 1.3x on drum tension.

SNAPS AND FITS
  Axles: 8.0 D rod (flat plane 2.7 from the axis), 7.6 neck, chamfered 8.2 head at the crank end; 4.5 neck + 5.6 head at the clip end.
  Cranks: round bore, then a 7.9 lip carrying the D flat inside four slotted leaves 2.25 thick x 10.5 long: 0.15 interference each side,
  about 40 N to click on, the square shoulder of the head stops them coming off, the flat in the lip takes the torque.
  C-clips: dia 11 x 2, 4.6 bore, 3.8 opening, on the 4.5 necks under the 5.6 heads. End play 0.3 + 0.3.
  Keyed bores 8.4 D (drum, back half of the spool); 8.5 round (frame, crank, front of the spool); 8.6 round in the cup. Cup over drum 0.6 radial.
  Lock pin 4.15 in 4.3 holes. If a snap is tight, warm the part a little.

STILL TO DO
  Measure wiper stall torque and shaft; then the drum gets a motor hub in place of the rear crank and the frame grows a motor mount.
  Spool crank becomes the fast motor. N20 on the cup if hand wrapping proves the mechanism.
  Exit eye carries the full tow load over a PLA edge: fine for bench pulls, PPA-CF for real tows.
