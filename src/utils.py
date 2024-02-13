import math


class Scalar:
    def __init__(self, deadZone=0.1, exponent=1):
        self.deadzone = deadZone
        self.exponent = exponent

    def scale(self, input):
        if abs(input) <= self._deadZone:
            return 0
        else:
            delta = abs(input) - self._deadZone
            sign = math.copysign(1, input)
            return sign * (delta / self._scale) ** self._exponent

    def __call__(self, input):
        return self.scale(input)

    @property
    def deadzone(self):
        return self._deadZone

    @deadzone.setter
    def deadzone(self, deadZone):
        deadZone = abs(deadZone)
        self._deadZone = deadZone
        self._scale = 1 - deadZone

    @property
    def exponent(self):
        return self._exponent

    @exponent.setter
    def exponent(self, exponent):
        self._exponent = exponent
