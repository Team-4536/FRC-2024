from ntcore import NetworkTableInstance
from PIDController import PIDController, PIDControllerForArm
from robotHAL import RobotHALBuffer

#from wpimath.controller import PIDController
#from robot import RobotInputs

class StateMachine():
    READY_FOR_RING = 0
    FEEDING = 1
    AIMING = 2
    REVVING = 3
    SHOOTING = 4

    AIM_SCALAR = 1

    # 0 is target aim, 1 is target speeds
    ampSetpoint = (0, 0)
    podiumSetpoint = (0, 0)
    subwooferSetpoint = (0, 250)

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ShooterStateMachineSettings")
        self.table.putNumber("kff", 0.00181)
        self.table.putNumber("kp", 0.0008)
        self.table.putNumber("aim kp", 0)
        self.table.putNumber("aim kg", 0.04)
        self.table.putNumber("aim target", 0)
        self.table.putNumber("aim scaler", 1)

        self.aimSetpoint = 0
        self.speedSetpoint = 0
        self.PIDspeedSetpoint = 0
        self.PIDaimTarget = 0
        self.state = self.READY_FOR_RING

        self.aimPID = PIDControllerForArm(0, 0, 0, 0, 0, 0)
        self.shooterPID = PIDController(0, 0, 0, 0.2)
        self.intakeShooterPID = PIDController(0., 0, 0)


    def update(self, hal: RobotHALBuffer, inputAmp: bool, inputPodium: bool, inputSubwoofer: bool, inputRev: bool, inputShoot: bool, time: float, dt: float):
        self.shooterPID.kff = self.table.getNumber("kff", 0)
        self.shooterPID.kp = self.table.getNumber("kp", 0)
        self.aimPID.kp = self.table.getNumber("aim kp", 0)
        self.aimPID.kg = self.table.getNumber("aim kg", 0)
        self.AIM_SCALAR = self.table.getNumber("aim scaler", 0)

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
            aimTarget = self.aimSetpoint
            speedTarget = 0
            if(inputRev):
                self.state = self.REVVING

        elif(self.state == self.REVVING):
            aimTarget = self.table.getNumber("aim target", 0) #self.aimSetpoint
            speedTarget = self.speedSetpoint
            if(not inputRev):
                self.state = self.AIMING
            if(inputShoot):
                self.state = self.SHOOTING
                self.time = time


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

        # self.PIDaimTarget = (aimTarget - self.PIDaimTarget) * self.AIM_SCALAR + self.PIDaimTarget
        aimTarget = self.table.getNumber("aim target", 0)
        hal.shooterAimSpeed = self.aimPID.tick(aimTarget, hal.shooterAimPos, dt)

        self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * self.AIM_SCALAR + self.PIDspeedSetpoint
        hal.shooterSpeed = self.shooterPID.tick(speedTarget, hal.shooterAngVelocityMeasured, dt)

        self.table.putNumber("PIDaimTarget", self.PIDaimTarget)

        return self.state
