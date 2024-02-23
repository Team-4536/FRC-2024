import math
from wpimath.geometry import Rotation2d, Translation2d
import wpilib
import numpy


class Scalar:
    def __init__(self, deadZone = .1, exponent = 1):
        self.deadZone = deadZone
        self.exponent = exponent

    def scale(self, input):
        if abs(input) < self.deadZone:
            return 0
        else:
            return ((abs(input) - self.deadZone)/(1 - self.deadZone))**self.exponent * math.copysign(1, input)

    def __call__(self, input):
        return self.scale(input)

    def setDeadZone(self, deadZone):
        self.deadZone = deadZone

    def setExponent(self, exponent):
        self.exponent = exponent

class CircularScalar:
    def __init__(self, deadzone: float, exponent: int):
        self.scalar = Scalar(deadzone, exponent)

    def Scale(self, x: float, y: float):
        angle = math.atan2(y, x)
        mag = math.hypot(x, y)

        ScaledMag = self.scalar.scale(mag)

        stickXY = Translation2d(ScaledMag, 0)
        stickXY = stickXY.rotateBy(Rotation2d(angle))
        
        return stickXY.x, stickXY.y