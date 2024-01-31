
from collections.abc import Callable

import robot
from ntcore import NetworkTableInstance

Stage = Callable[[robot.Robot], bool]

class Auto():
    def __init__(self, stagelist: list[Stage], time:float) -> None:
        self.list = stagelist
        self.listindex = 0
        self.stagestart = time
        self.table = NetworkTableInstance.getDefault().getTable("autos")

    def update(self, r: robot.Robot, time: float) -> None:
        self.table.putString("stage", f"{self.listindex}/{len(self.list)}")
        self.table.putNumber("stageTime", self.stagestart)

        if self.listindex < len(self.list):
            s = self.list[self.listindex]
            done = s(r)
            if done:
                self.listindex += 1
                self.stagestart = time