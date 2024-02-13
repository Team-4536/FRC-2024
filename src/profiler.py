import wpilib
from ntcore import NetworkTableInstance

startTime: float = 0

def start():
    global startTime
    startTime = wpilib.getTime()

def end(title: str):
    NetworkTableInstance.getDefault().getTable("profiling").putNumber(title, wpilib.getTime() - startTime)