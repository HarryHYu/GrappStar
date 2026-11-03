"""Rover drive hook. The rover's drive train and its controller are not known yet (to confirm with Harry), so the
mission never drives the rover itself unless a RoverDrive subclass says it can.

During the tow the mission calls tow_assist() each time the line reaches the tension limit. The default
OperatorRover asks the operator to drive the rover by hand (RC transmitter or whatever drives it today) and to press
a button when the line is slack again or the rover is free.

To automate later: subclass RoverDrive, implement drive(speed, seconds) and stop() for the real drive (for example a
serial motor controller), set can_drive = True, and pass it to the App (app.App(rover=MyRover())).
"""
from __future__ import annotations

from typing import Optional


class RoverDrive:
    can_drive = False
    name = "none"

    def drive(self, speed: float, seconds: float) -> None:
        """speed -1..1 (positive = toward the dart). Blocks for seconds, then stops."""
        raise NotImplementedError("no rover drive interface: drive by hand")

    def stop(self) -> None:
        pass

    def tow_assist(self, mission, tension_n: Optional[float]) -> str:
        """Called with the line held at the limit. Return "slack" (take up line again), "done" (rover free) or
        "abort"."""
        t = f"{tension_n:.0f} N" if tension_n is not None else "the limit"
        c = mission.ask(f"Line is holding at {t}. Drive the rover gently toward the dart now. "
                        "Press Take up slack when the line goes slack, or Rover free when it is out.",
                        ["Take up slack", "Rover free", "Abort"])
        return {"Take up slack": "slack", "Rover free": "done"}.get(c, "abort")


class OperatorRover(RoverDrive):
    name = "operator"


class AutoRover(RoverDrive):
    """Template for a drivable rover: drives forward a little each time the line holds, a fixed number of times."""
    can_drive = True
    name = "auto"

    def __init__(self, speed: float = 0.3, step_s: float = 1.0, max_steps: int = 10):
        self.speed, self.step_s, self.max_steps, self.steps = speed, step_s, max_steps, 0

    def tow_assist(self, mission, tension_n):
        if self.steps >= self.max_steps:
            return "done"
        self.steps += 1
        self.drive(self.speed, self.step_s)
        return "slack"
