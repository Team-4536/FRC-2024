from ntcore import NetworkTableInstance
from robotHAL import RobotHALBuffer, RobotHAL
#from wpimath.controller import PIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from PIDController import PIDControllerForArm, PIDController
#from robot import RobotInputs

class StateMachine():
    READY_FOR_RING = 0
    FEED_AMP = 1
    FEED_PODIUM= 2
    FEED_SUBWOOFER = 3
    AIM_AMP = 4
    AIM_PODIUM = 5
    AIM_SUBWOOFER = 6
    SHOOTING = 8

    ampSetpoint = 0
    podiumSetpoint = 0
    subwooferSetpoint = 0


    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ShooterStateMachineSettings")
        self.table.putNumber("kff", 0.00181)
        self.table.putNumber("kp", 0.0008)
        self.table.putNumber("targetSpeed", 200)

        self.state = self.READY_FOR_RING
        self.aimPID = PIDControllerForArm(0, 0, 0, 0, 0, 0)
        self.aimSetpoint = 0
        self.shooterPID = PIDController(0, 0, 0, 0.2)
        self.shooterVelocityTarget = 0
        self.intakeShooterPID = PIDController(0., 0, 0)
        self.intakeShooterVelocity = 0


    def update(self, hal: RobotHALBuffer, inputAmp: bool, inputPodium: bool, inputSubwoofer: bool, inputShoot: bool, time, dt, trigger: bool) -> int:
        self.shooterPID.kff = self.table.getNumber("kff", 0)
        self.shooterPID.kp = self.table.getNumber("kp", 0)
        


        if(inputAmp):
            if(self.state == self.FEED_PODIUM or self.state == self.FEED_SUBWOOFER or self.state == self.READY_FOR_RING):
                self.state = self.AIM_AMP #CHANG THIS

            if(self.state == self.AIM_PODIUM or self.state == self.AIM_SUBWOOFER):
                self.state = self.AIM_AMP

        if(inputPodium):
            if(self.state == self.FEED_AMP or self.state == self.FEED_SUBWOOFER or self.state == self.READY_FOR_RING):
                self.state = self.FEED_SUBWOOFER

            if(self.state == self.AIM_AMP or self.state == self.AIM_SUBWOOFER):
                self.state = self.AIM_SUBWOOFER

        if(inputSubwoofer):
            if(self.state == self.FEED_AMP or self.state == self.FEED_PODIUM or self.state == self.READY_FOR_RING):
                self.state = self.FEED_SUBWOOFER

            if(self.state == self.AIM_AMP or self.state == self.AIM_PODIUM):
                self.state = self.AIM_SUBWOOFER


        if(self.state == self.READY_FOR_RING):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0
            self.intakeShooterVelocity = 0
            

        #Check for feeding instruction

        elif(self.state == self.FEED_AMP):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0
            self.intakeShooterVelocity = .2
            hal.intakeSpeeds[1] = .2

            if(hal.shooterSensor == True):
                self.state = self.AIM_AMP
            
            

        elif(self.state == self.FEED_PODIUM):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0
            self.intakeShooterVelocity = .2
            hal.intakeSpeeds[1] = .2

            if(hal.shooterSensor == True):
                self.state = self.AIM_PODIUM
            
            

        elif(self.state == self.FEED_SUBWOOFER):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0
            self.intakeShooterVelocity = .2
            hal.intakeSpeeds[1] = .2

            if(hal.shooterSensor == True):
                self.state = self.AIM_SUBWOOFER
            
            

        #Check for aiming instructions
                
        elif(self.state == self.AIM_AMP):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0 #self.table.getNumber("targetSpeed", 0)
            self.intakeShooterVelocity = 0

            if(trigger):
                hal.intakeSpeeds[1] = 0.2
                self.intakeShooterVelocity = 0.2

            if(inputShoot):
                self.state = self.SHOOTING
                self.time = time
            
            

        elif(self.state == self.AIM_PODIUM):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0 #self.table.getNumber("targetSpeed", 0)
            self.intakeShooterVelocity = 0
            if(trigger):
                hal.intakeSpeeds[1] = 0.2
                self.intakeShooterVelocity = 0.2
        
            if(inputShoot):
                self.state = self.SHOOTING
                self.time = time
            
            

        elif(self.state == self.AIM_SUBWOOFER):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = 0 #self.table.getNumber("targetSpeed", 0)
            self.intakeShooterVelocity = 0
            if(trigger):
                hal.intakeSpeeds[1] = 0.2
                self.intakeShooterVelocity = 0.2
        
            if(inputShoot):
                self.state = self.SHOOTING
                self.time = time
            
            


        elif(self.state == self.SHOOTING):
            self.aimSetpoint = 0
            self.shooterVelocityTarget = self.table.getNumber("targetSpeed", 0)
            self.intakeShooterVelocity = 1
            hal.intakeSpeeds[1] = .6
            self.table.putNumber("IN SHOOTING", 1)
            #if(time - self.time > 4.0):
            #    self.state = self.READY_FOR_RING
            if(inputShoot == False):
                self.table.putNumber("IN SHOOTING", 0)
                self.state = self.READY_FOR_RING

        
        # hal.shooterAimSpeed = 0#self.aimPID.tick(self.aimSetpoint, hal.shooterAimPos, dt)
        hal.shooterSpeed = self.shooterPID.tick(self.shooterVelocityTarget, hal.shooterAngVelocityMeasured, dt)
        hal.shooterIntakeSpeed = self.intakeShooterVelocity

        return self.state
        


