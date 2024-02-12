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

    SHOOTER_SCALER = 0.1
    AIM_SCALER = 0.1

    # 0 is target aim, 1 is target speeds
    ampSetpoint = (0, 0)
    podiumSetpoint = (0, 0)
    subwooferSetpoint = (0, 200)

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ShooterStateMachineSettings")
        self.table.putNumber("kff", 0.00181)
        self.table.putNumber("kp", 0.0008)
        self.table.putNumber("Arm kp", 0)
        self.table.putNumber("Arm kg", 0)
        self.table.putNumber("armTarget", 0)
        self.table.putNumber("shooter scaler", 0)

        self.aimSetpoint = 0
        self.speedSetpoint = 0
        self.PIDspeedSetpoint = 0
        self.PIDaimTarget = 0
        self.state = self.READY_FOR_RING

        self.aimPID = PIDControllerForArm(0, 0, 0, 0, 0, 0)
        self.shooterPID = PIDController(0, 0, 0, 0.2)
        self.intakeShooterPID = PIDController(0., 0, 0)


    def update(self, hal: RobotHALBuffer, inputAmp: bool, inputPodium: bool, inputSubwoofer: bool, inputShoot: bool, time: float, dt: float):
        self.shooterPID.kff = self.table.getNumber("kff", 0)
        self.shooterPID.kp = self.table.getNumber("kp", 0)
        self.aimPID.kp = self.table.getNumber("Arm kp", 0)
        self.aimPID.kg = self.table.getNumber("Arm kg", 0)
        self.SHOOTER_SCALER = self.table.getNumber("shooter scaler", 0)

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
            hal.shooterIntakeSpeed = 0.1
            hal.intakeSpeeds[1] = 0.1
            if hal.shooterSensor:
                self.state = self.AIMING

        elif(self.state == self.AIMING):
            #aimTarget = self.aimSetpoint
            speedTarget = self.speedSetpoint

            aimTarget = self.table.getNumber("armTarget", 0)

            if(inputShoot):
                self.state = self.SHOOTING
                self.time = time

        elif(self.state == self.SHOOTING):
            aimTarget = self.table.getNumber("armTarget", 0) #self.aimSetpoint
            speedTarget = self.speedSetpoint
            hal.shooterIntakeSpeed = 0.4
            hal.intakeSpeeds[1] = 0.4
            if(time - self.time > 1.0):
               self.state = self.READY_FOR_RING

        else:
            aimTarget = 0
            speedTarget = 0

        # hal.shooterAimSpeed = self.aimPID.tick(self.aimSetpoint, hal.shooterAimPos, dt)
        self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * self.SHOOTER_SCALER + self.PIDspeedSetpoint
        self.PIDaimTarget = (aimTarget - self.PIDaimTarget) * self.AIM_SCALER + self.PIDaimTarget
        self.table.putNumber("PIDarmTarget", self.PIDaimTarget)
        hal.shooterAimSpeed = self.aimPID.tick(self.PIDaimTarget, hal.shooterAimPos, dt)
        hal.shooterSpeed = self.shooterPID.tick(self.PIDspeedSetpoint, hal.shooterAngVelocityMeasured, dt)

        return self.state
