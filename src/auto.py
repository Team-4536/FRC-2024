
from collections.abc import Callable
from typing import TYPE_CHECKING

from ntcore import NetworkTableInstance

if TYPE_CHECKING:
    from robot import Robot

Stage = Callable[['Robot'], bool]

class Auto():
    def __init__(self, stagelist: list[Stage], time:float) -> None:
        self.list = stagelist
        self.listindex = 0
        self.stagestart = time
        self.table = NetworkTableInstance.getDefault().getTable("autos")

    def update(self, r: "Robot") -> None:
        self.table.putString("stage", f"{self.listindex}/{len(self.list)}")
        self.table.putNumber("stageTime", self.stagestart)

        if self.listindex < len(self.list):
            s = self.list[self.listindex]
            done = s(r)
            if done:
                self.listindex += 1
                self.stagestart = r.time.timeSinceInit