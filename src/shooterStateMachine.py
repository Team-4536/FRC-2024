from ntcore import NetworkTableInstance
from PIDController import PIDController, PIDControllerForArm
from robotHAL import RobotHALBuffer

#from wpimath.controller import PIDController
#from robot import RobotInputs

class StateMachine():
    READY_FOR_RING = 0
    FEEDING = 1
    AIMING = 2
    SHOOTING = 3

    # 0 is target aim, 1 is target speeds
    ampSetpoint = (0, 0)
    podiumSetpoint = (0, 0)
    subwooferSetpoint = (0, 0)

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ShooterStateMachineSettings")
        self.table.putNumber("kff", 0.00181)
        self.table.putNumber("kp", 0.0008)
        self.table.putNumber("targetSpeed", 200)

        self.aimSetpoint = 0
        self.speedSetpoint = 0
        self.state = self.READY_FOR_RING

        self.aimPID = PIDControllerForArm(0, 0, 0, 0, 0, 0)
        self.shooterPID = PIDController(0, 0, 0, 0.2)
        self.intakeShooterPID = PIDController(0., 0, 0)


    def update(self, hal: RobotHALBuffer, inputAmp: bool, inputPodium: bool, inputSubwoofer: bool, inputShoot: bool, time, dt) -> int:
        self.shooterPID.kff = self.table.getNumber("kff", 0)
        self.shooterPID.kp = self.table.getNumber("kp", 0)

        if(inputAmp):
            if(self.state == self.READY_FOR_RING):
                self.state = self.FEEDING
            self.aimSetpoint = self.ampSetpoint[0]
            self.speedSetpoint = self.ampSetpoint[1]

        if(inputPodium):
            if(self.state == self.READY_FOR_RING):
                self.state = self.FEEDING
            self.aimSetpoint = self.podiumSetpoint[0]
            self.speedSetpoint = self.podiumSetpoint[1]

        if(inputSubwoofer):
            if(self.state == self.READY_FOR_RING):
                self.state = self.FEEDING
            self.aimSetpoint = self.subwooferSetpoint[0]
            self.speedSetpoint = self.subwooferSetpoint[1]


        if(self.state == self.READY_FOR_RING):
            aimTarget = 0
            speedTarget = 0

        elif(self.state == self.FEEDING):
            aimTarget = 0
            speedTarget = 0
            hal.shooterIntakeSpeed = 0.2
            hal.intakeSpeeds[1] = 0.2
            if hal.shooterSensor:
                self.state = self.AIMING

        elif(self.state == self.AIMING):
            aimTarget = self.aimSetpoint
            speedTarget = self.speedSetpoint
            if(inputShoot):
                self.state = self.SHOOTING
                self.time = time

        elif(self.state == self.SHOOTING):
            aimTarget = self.aimSetpoint
            speedTarget = self.speedSetpoint
            hal.shooterIntakeSpeed = 0.4
            hal.intakeSpeeds[1] = 0.4
            if(time - self.time > 4.0):
               self.state = self.READY_FOR_RING

        else:
            aimTarget = 0
            speedTarget = 0

        # hal.shooterAimSpeed = self.aimPID.tick(self.aimSetpoint, hal.shooterAimPos, dt)
        hal.shooterAimSpeed = aimTarget
        hal.shooterSpeed = self.shooterPID.tick(speedTarget, hal.shooterAngVelocityMeasured, dt)

        return self.state
