import math

from ntcore import NetworkTable, NetworkTableInstance
from real import signum

createdControllers: list['PIDController'] = []
pidTable: NetworkTable = NetworkTableInstance.getDefault().getTable("pid")

class PIDController:
    def __init__(self, name: str, kp: float = 0, ki: float = 0, kd: float = 0, kff: float = 0) -> None:
        self.name = name
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.kff: float = kff
        self.ktest: float = 0
        self.integral: float = 0
        self.prevErr: float = 0
        self.integralZone: float = 0
        createdControllers.append(self)

    # function returns the recommended force towards the target
    def tick(self, target: float, position: float, dt: float) -> float:
        error = target - position
        return self.tickErr(error, target, dt)

    # output will be the same sign as the input error
    def tickErr(self, error: float, target: float, dt: float) -> float:
        return self._tick(error, target, target - error, dt)

    # this is the funcion that inheriting classes should override, touching the other ones can cause problems
    def _tick(self, error: float, target: float, position: float, dt: float) -> float:
        derivative = (error - self.prevErr) / dt
        self.integral += self.ki * error * dt
        if abs(self.integral) > self.integralZone:
            self.integral = (self.integralZone * signum(self.integral))
        out = (self.kp * error) + (self.integral) + (self.kd * derivative) + (self.kff * target) + self.ktest
        self.prevErr = error
        return out

    def reset(self) -> None:
        self.integral = 0
        self.prevErr = 0

    def _publish(self) -> None:
        t = pidTable.getSubTable(self.name)
        t.putNumber("integral", self.integral)
        if t.getNumber("Kp", None) is None:
            t.putNumber("Kp", self.kp)
            t.putNumber("Ki", self.ki)
            t.putNumber("Kd", self.kd)
            t.putNumber("Kff", self.kff)
            t.putNumber("Ktest", self.ktest)
            t.putNumber("integralZone", self.integralZone)
        else:
            self.kp = t.getNumber("Kp", 0)
            self.ki = t.getNumber("Ki", 0)
            self.kd = t.getNumber("Kd", 0)
            self.kff = t.getNumber("Kff", 0)
            self.ktest = t.getNumber("Ktest", 0)
            self.integralZone = t.getNumber("integralZone", 0)

class PIDControllerForArm(PIDController):
    def __init__(self, name: str, kp: float = 0, ki: float = 0, kd: float = 0, kff: float = 0, kg: float = 0, balanceAngle: float = 0.1) -> None:
        super().__init__(name = name, kp = kp, ki = ki, kd = kd, kff = kff)
        self.balanceAngle = balanceAngle
        self.kg = kg

    def _tick(self, error: float, target: float, position: float, dt: float) -> float:
        out = super()._tick(error, target, position, dt)
        out += self.kg * math.cos(position + self.balanceAngle)
        return out

    def _publish(self) -> None:
        t = pidTable.getSubTable(self.name)
        if t.getNumber("Kp", None) is None:
            t.putNumber("Kg", self.kg)
        else:
            self.kg = t.getNumber("Kg", 0)
        super()._publish()

class PIDControllerForCam(PIDController):
    def __init__(self, name: str, kp: float = 0, ki: float = 0, ks: float = 0, intigralZone: float = 0) -> None:
        super().__init__(name = name, kp = kp, ki = ki, kd = 0, kff = 0)
        self.ks = ks
        self.integralZone = intigralZone

    def _tick(self, error: float, target: float, position: float, dt: float) -> float:
        out = super()._tick(error, target, position, dt)
        out += self.ks * signum(error)
        return out

    def _publish(self) -> None:
        t = pidTable.getSubTable(self.name)
        if t.getNumber("Kp", None) is None:
            t.putNumber("Ks", self.ks)
        else:
            self.ks = t.getNumber("Ks", 0)
        super()._publish()

def updatePIDsInNT():
    for c in createdControllers:
        c._publish()
