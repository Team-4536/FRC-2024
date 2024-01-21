import math

class scaler:
    def __init__(self, deadZone = .1, power = 1):
        self.deadZone = deadZone
        self.power = power
    
    def scale(self, input):
        if abs(input) < self.deadZone:
            return 0
        else:
            return ((abs(input) - self.deadZone)/(1 - self.deadZone))**self.power * math.copysign(1, input)
    
    def setDeadZone(self, deadZone):
        self.deadZone = deadZone
    
    def setPower(self, power):
        self.power = power