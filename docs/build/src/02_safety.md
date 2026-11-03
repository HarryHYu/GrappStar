# 2. Safety

Read this before any hands-on step.

## Electrical

- **Battery unplugged before you touch, add or move any wire.** On the dart that means the power loop plug out. The
  first XIAO died from wiring with the battery live.
- Battery unplugged before you plug any board into the Mac by USB.
- Check for shorts with the meter on beep before the first power-on of anything you wired: red to black must not beep.
- The 3S LiPo can deliver over 100 A into a short. Keep the 20 A and 10 A fuses in their leads. Never bypass them.
- LiPo care: charge on the iMax B6 in balance mode, on a non-flammable surface, never unattended. Stop using the
  pack at 3.3 V per cell under load (9.9 V). A puffed pack goes in a LiPo bag and to battery recycling.
- 18350 cells: never let both cells touch metal at the same time. Charge out of the dart in the XTAR, or through the
  tail balance socket with the loop plug out.
- Never connect the DRV8833 motor supply (VM) to the LiPo directly: 10.8 V maximum. It runs from the 9.0 V buck.
- The dart VTX must have its antenna on before it is powered.

## Mechanical

- The wiper motor is strong (about 15 N m) and self-locking. Hands out of the gears, drum and rail while the battery
  is plugged in. Pull the battery plug to stop everything.
- The launch springs store energy. Cocked carriage = loaded. Never put a hand in front of the rail or the carriage.
  To unload a cocked base without firing, see troubleshooting.
- From the first spring test on: safety glasses, nobody in front of the rail, the area clear for 30 m.
- **Never fire a dart, dummy or real, without the tether tied on.** The tether keeps it nose first; without it the
  dart can tumble or come down tail first.
- Augers and drills are sharp and turn with force. Hold the dart by the hull, never near a pod or the foot.
- Soldering: iron on its stand, fan on for fumes, 3 s at most per joint near PLA. Heat gun on low, no lighter near PLA.
