from enum import Enum

from ntcore import NetworkTableInstance
from PIDController import PIDController, PIDControllerForArm
from robotHAL import RobotHALBuffer


class ShooterTarget(Enum):
    NONE = 0
    AMP = 1
    PODIUM = 2
    SUBWOOFER = 3

class StateMachine():
    READY_FOR_RING = 0
    FEEDING = 1
    AIMING = 2
    REVVING = 3
    SHOOTING = 4

    SPEED_SMOOTH_SCALAR = 0.1
    AIM_SMOOTH_SCALAR = 0.05

    # 0 is target aim, 1 is target speeds
    ampSetpoint = (1.6, 100)
    podiumSetpoint = (0, 0)
    subwooferSetpoint = (0, 250)

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ShooterStateMachineSettings")
        self.table.putNumber("kff", 0.00181)
        self.table.putNumber("kp", 0.0008)
        self.table.putNumber("aim kp", 0.4)
        self.table.putNumber("aim kg", 0.04)

        self.table.putNumber("podiumAim", 0)
        self.table.putNumber("podiumSpeed", 0)

        self.aimSetpoint = 0
        self.speedSetpoint = 0
        self.PIDspeedSetpoint = 0
        self.PIDaimSetpoint = 0
        self.state = self.READY_FOR_RING

        self.aimPID = PIDControllerForArm(0, 0, 0, 0, 0, 0)
        self.shooterPID = PIDController(0, 0, 0, 0.2)
        self.intakeShooterPID = PIDController(0., 0, 0)

        self.onTarget: bool = True # variable for autos to check in on this, as well as for not firing to early

        self.inputAim: ShooterTarget = ShooterTarget.NONE
        self.inputRev: bool = False
        self.inputShoot: bool = False

    # none will not change currently targeted pos
    def aim(self, target: ShooterTarget):
        self.table.putNumber("aimed", target.value)
        self.inputAim = target
    def rev(self, rev: bool):
        self.table.putBoolean("revved", rev)
        self.inputRev = rev
    def shoot(self, shoot: bool):
        self.table.putBoolean("shooting", shoot)
        self.inputShoot = shoot

    def publishInfo(self):
        self.table.putNumber("state", self.state)
        self.table.putNumber("targetSpeed", self.speedSetpoint)
        self.table.putNumber("targetSpeedSmoothed", self.PIDspeedSetpoint)
        self.table.putNumber("targetAim", self.aimSetpoint)
        self.table.putNumber("targetAimSmoothed", self.PIDaimSetpoint)
        self.table.putBoolean("onTarget", self.onTarget)


    # To send commands to the state machine, use aim(), rev(), and shoot() before calling this
    # Note that calling aim from READY_FOR_RING will feed and then aim
    def update(self, hal: RobotHALBuffer, time: float, dt: float):
        self.shooterPID.kff = self.table.getNumber("kff", 0)
        self.shooterPID.kp = self.table.getNumber("kp", 0)
        self.aimPID.kp = self.table.getNumber("aim kp", 0)
        self.aimPID.kg = self.table.getNumber("aim kg", 0)

        self.podiumSetpoint = (self.table.getNumber("podiumAim", 0.0), self.table.getNumber("podiumSpeed", 0.0))

        if(self.inputAim != ShooterTarget.NONE):
            if(self.inputAim == ShooterTarget.AMP):
                self.aimSetpoint = self.ampSetpoint[0]
                self.speedSetpoint = self.ampSetpoint[1]
            elif(self.inputAim == ShooterTarget.PODIUM):
                self.aimSetpoint = self.podiumSetpoint[0]
                self.speedSetpoint = self.podiumSetpoint[1]
            elif(self.inputAim == ShooterTarget.SUBWOOFER):
                self.aimSetpoint = self.subwooferSetpoint[0]
                self.speedSetpoint = self.subwooferSetpoint[1]

        self.onTarget = abs(hal.shooterAimPos - self.aimSetpoint) < 0.1 and abs(hal.shooterAngVelocityMeasured - self.speedSetpoint) < 10

        if(self.state == self.READY_FOR_RING):
            aimTarget = 0
            speedTarget = 0
            if(self.inputAim != ShooterTarget.NONE):
                self.state = self.FEEDING

        elif(self.state == self.FEEDING):
            aimTarget = 0
            speedTarget = 0
            hal.shooterIntakeSpeed = 0.1
            hal.intakeSpeeds[1] = 0.1
            if hal.shooterSensor:
                self.state = self.AIMING

        elif(self.state == self.AIMING):
            aimTarget = self.aimSetpoint
            speedTarget = 0
            if(self.inputRev):
                self.state = self.REVVING

        elif(self.state == self.REVVING):
            aimTarget = self.aimSetpoint
            speedTarget = self.speedSetpoint
            if(self.inputShoot):
                if(self.onTarget):
                    self.state = self.SHOOTING
                    self.time = time
            elif(not self.inputRev):
                self.state = self.AIMING

        elif(self.state == self.SHOOTING):
            aimTarget = self.aimSetpoint
            speedTarget = self.speedSetpoint
            hal.shooterIntakeSpeed = 0.4
            hal.intakeSpeeds[1] = 0.4
            if(time - self.time > 1.0):
               self.state = self.READY_FOR_RING

        else:
            aimTarget = 0
            speedTarget = 0

        self.PIDaimSetpoint = (aimTarget - self.PIDaimSetpoint) * self.AIM_SMOOTH_SCALAR + self.PIDaimSetpoint
        hal.shooterAimSpeed = self.aimPID.tick(self.PIDaimSetpoint, hal.shooterAimPos, dt)

        self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * self.SPEED_SMOOTH_SCALAR + self.PIDspeedSetpoint
        hal.shooterSpeed = self.shooterPID.tick(self.PIDspeedSetpoint, hal.shooterAngVelocityMeasured, dt)

        self.inputAim = ShooterTarget.NONE
        self.inputRev = False
        self.inputShoot = False

        return self.state
