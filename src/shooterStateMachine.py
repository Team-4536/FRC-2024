from robotHAL import RobotHALBuffer, RobotHAL
from wpimath.controller import PIDController
from wpimath.controller import SimpleMotorFeedforwardMeters

class StateMachine():
    READY_FOR_RING = 0
    FEED_AMP = 1
    FEED_PODIUM= 2
    FEED_SUBWOOFER = 3
    AIM_AMP = 4
    AIM_PODIUM = 5
    AIM_SUBWOOFER = 6
    READY_TO_SHOOT = 7
    SHOOTING = 8

    ampSetpoint = 0
    podiumSetpoint = 0
    subwooferSetpoint = 0


    def __init__(self):
        self.state = self.READY_FOR_RING
        self.aimPID = PIDController(0, 0, 0)
        self.aimSetpoint = 0
        self.shooterPID = SimpleMotorFeedforwardMeters(0, 0, 0)
        self.shooterVelocity = 0
        self.intakeShooterPID = SimpleMotorFeedforwardMeters(0, 0, 0)
        self.intakeShooterVelocity = 0


    def update(self, hal: RobotHALBuffer, inputAmp: bool, inputPodium: bool, inputSubwoofer: bool, inputShoot: bool, time) -> int:

        if(inputAmp):
            if(self.state == self.FEED_PODIUM or self.state == self.FEED_SUBWOOFER or self.state == self.READY_FOR_RING):
                self.state = self.FEED_AMP

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
            self.shooterVelocity = 0
            self.intakeShooterVelocity = 0
            

        #Check for feeding instruction

        elif(self.state == self.FEED_AMP):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 1

            if(hal.shooterSensor == True):
                self.state = self.AIM_AMP
            
            

        elif(self.state == self.FEED_PODIUM):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 1

            if(hal.shooterSensor == True):
                self.state = self.AIM_PODIUM
            
            

        elif(self.state == self.FEED_SUBWOOFER):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 1

            if(hal.shooterSensor == True):
                self.state = self.AIM_SUBWOOFER
            
            

        #Check for aiming instructions
                
        elif(self.state == self.AIM_AMP):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 0

            if(self.aimPID.atSetpoint() and inputShoot):
                self.state = self.SHOOTING
                self.time = time
            
            

        elif(self.state == self.AIM_PODIUM):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 0
        
            if(self.aimPID.atSetpoint() and inputShoot):
                self.state = self.SHOOTING
                self.time = time
            
            

        elif(self.state == self.AIM_SUBWOOFER):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 0
        
            if(self.aimPID.atSetpoint() and inputShoot):
                self.state = self.SHOOTING
                self.time = time
            
            


        elif(self.state == self.SHOOTING):
            self.aimSetpoint = 0
            self.shooterVelocity = 1
            self.intakeShooterVelocity = 0

            if(time - self.time > 4.0):
                self.state = self.READY_FOR_RING

        
        hal.shooterAimSpeed = self.aimPID.calculate(hal.shooterPos, self.aimSetpoint)
        hal.shooterSpeed = self.shooterPID.calculate(self.shooterVelocity)
        hal.shooterIntakeSpeed = self.intakeShooterPID.calculate(self.intakeShooterVelocity)

        return self.state
        


