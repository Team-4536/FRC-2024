import math


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
