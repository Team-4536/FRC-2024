from typing import Callable

import wpilib

# Data for the state of this module - dont mess with please
# accumulates samples recorded with profileRegion and profileFn
_completeSamples: list['__Sample'] = []

class __Sample():
    def __init__(self, name: str) -> None:
        self.startTime: float = 0.0
        self.endTime: float = 0.0
        self.functionName: str = name

    def __enter__(self):
        self.startTime = wpilib.getTime()

    def __exit__(self, *args):
        self.endTime = wpilib.getTime()
        global _completeSamples
        _completeSamples.append(self)
        print(f"sample from {self.functionName} finished")


# dumps all accumulated samples in the a file named perfLog.csv
# reference time adjusts all start and ends to be relative to the given time
def flushToFile(referenceTime: float) -> None:
    f = open("perfLog", 'w')
    f.write("name, start, end\n")
    for sample in _completeSamples:
        f.write(f"{sample.functionName}, {sample.startTime - referenceTime}, {sample.endTime - referenceTime}\n")
    f.close()
    _completeSamples.clear()

# Decorator that profiles a function on each call
def profileFn(fn: Callable) -> Callable:
    def inner(*args, **kwargs):
        with __Sample(fn.__qualname__):
            fn(*args, **kwargs)
    return inner

# intended to be used in a with statement, user should not do anything with the returned sample
def profileRegion(name: str) -> __Sample:
    return __Sample(name)