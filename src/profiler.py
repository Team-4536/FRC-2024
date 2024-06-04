from typing import Callable
import wpilib
from ntcore import NetworkTableInstance

class __Sample():
    def __init__(self, name: str) -> None:
        self.startTime: float = 0.0
        self.endTime: float = 0.0
        self.functionName: str = name

    def __enter__(self):
        self.startTime = wpilib.getTime()

    def __exit__(self, *args):
        self.endTime = wpilib.getTime()
        __completeSamples.append(self)


# Data for the state of this module - dont mess with please
# accumulates samples recorded with profileRegion and profileFn
__completeSamples: list[__Sample] = []

# dumps all accumulated samples in the a file named perfLog.csv
def flushToFile() -> None:
    f = open("perfLog", 'w')
    f.write(f"name, start, end\n")
    for sample in __completeSamples:
        f.write(f"{sample.functionName}, {sample.startTime}, {sample.endTime}\n")
    f.close()
    __completeSamples.clear()

def profileFn(fn: Callable) -> Callable:
    def inner():
        with __Sample(fn.__name__):
            fn()
    return inner

def profileRegion(name: str) -> __Sample:
    return __Sample(name)