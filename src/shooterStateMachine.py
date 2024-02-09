from robotHAL import RobotHALBuffer, RobotHAL

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


    #SHOT = 5
    def __init__(self):
        self.state = self.READY_FOR_RING

    def update(self, hal: RobotHALBuffer, inputAmp: bool, inputPodium: bool, inputSubwoofer: bool, inputShoot: bool):

        if(inputAmp):
            if(self.state == self.FEED_PODIUM or self.state == self.FEED_SUBWOOFER):
                self.state = self.FEED_AMP

            if(self.state == self.AIM_PODIUM or self.state == self.AIM_SUBWOOFER):
                self.state = self.AIM_AMP

        if(inputPodium):
            if(self.state == self.FEED_AMP or self.state == self.FEED_SUBWOOFER):
                self.state = self.FEED_SUBWOOFER

            if(self.state == self.AIM_AMP or self.state == self.AIM_SUBWOOFER):
                self.state = self.AIM_SUBWOOFER

        if(inputSubwoofer):
            if(self.state == self.FEED_AMP or self.state == self.FEED_PODIUM):
                self.state = self.FEED_SUBWOOFER

            if(self.state == self.AIM_AMP or self.state == self.AIM_PODIUM):
                self.state = self.AIM_SUBWOOFER




        if(self.state == self.READY_FOR_RING):
            hal.shooterSpeed = 0.0
            hal.shooterIntakeSpeed = 0.0

        #Check for feeding instruction

        if(self.state == self.FEED_AMP):
            hal.shooterSpeed = 0.0
            hal.shooterIntakeSpeed = .5 #

            if(hal.shooterSensor == True):
                self.state = self.AIM_AMP

        if(self.state == self.FEED_PODIUM):
            hal.shooterSpeed = 0.0
            hal.shooterIntakeSpeed = 0.5

            if(hal.shooterSensor == True):
                self.state = self.AIM_PODIUM

        if(self.state == self.FEED_SUBWOOFER):
            hal.shooterSpeed = 0.0
            hal.shooterIntakeSpeed = 0.5

            if(hal.shooterSensor == True):
                self.state = self.AIM_SUBWOOFER

        #Check for aiming instructions
                
        if(self.state == self.AIM_AMP):
            hal.shooterSpeed = 0.8
            hal.shooterIntakeSpeed = 0.0
            #set value to aim amp
            #hal.shooterAimSpeed = ?

            if(hal.shooterAimPos == """some value"""):
                self.state = self.READY_TO_SHOOT

        if(self.state == self.AIM_PODIUM):
            hal.shooterSpeed = 0.8
            hal.shooterIntakeSpeed = 0.0
            #set value to aim amp
            #hal.shooterAimSpeed = ?
        
            if(hal.shooterAimPos == """some value"""):
                self.state = self.READY_TO_SHOOT

        if(self.state == self.AIM_SUBWOOFER):
            hal.shooterSpeed = 0.8
            hal.shooterIntakeSpeed = 0.0
            #set value to aim amp
            #hal.shooterAimSpeed = ?
        
            if(hal.shooterAimPos == """some value"""):
                self.state = self.READY_TO_SHOOT

        if(self.state == self.READY_TO_SHOOT):
            hal.shooterSpeed = 0.8
            hal.shooterIntakeSpeed

            if(self.state == self.READY_TO_SHOOT):
                hal.shooterSpeed = 0.8
                hal.shooterIntakeSpeed = 0.8

        if(self.state == self.SHOOTING):
            hal.shooterSpeed = 0.8
            hal.shooterIntakeSpeed = 0.8




        



        