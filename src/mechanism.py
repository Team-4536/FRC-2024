from robotHAL import RobotHALBuffer


class Mechanism():
    def __init__(self, halBuffer: RobotHALBuffer) -> None:
        self.hal = halBuffer


    def scoreSpeaker(self):
        # once the motor is installed and stuff, make sure to set the correct angle before shooting

        self.hal.shooterSpeed = 0.25
        self.hal.shooterIntakeSpeed = 0.2 # speed not tested

    def scoreAmp(self):
        self.loadShooter()

        # aim shooter to amp angle

        self.hal.shooterSpeed = 0.1
        self.hal.shooterIntakeSpeed = 0.2 # speed not tested

    def runIntake(self):
        self.hal.intakeSpeeds[0] = 0.4 # slow for prototype intake so it doesnt rip itself apart and die
        self.hal.intakeSpeeds[1] = 0.4
        # stop once sensor is triggered

    def loadShooter(self):
        # write something here eventually
        pass

    def loadBlueIntake(self):
        # write something here eventually
        pass

    def climbUp(self):
        # write something here eventually
        pass

    def climbDown(self):
        # write something here eventually
        pass

    def scoreTrap(self):
        # write something here eventually
        pass

    def clearMech(self):
        # spit out ring
        self.hal.shooterSpeed = -0.1
        self.hal.shooterIntakeSpeed = -0.4
        self.hal.intakeSpeeds[0] = -0.4
        self.hal.intakeSpeeds[1] = -0.4

    def reverseIntake(self):
        self.hal.intakeSpeeds[0] = -0.4
        self.hal.intakeSpeeds[1] = -0.4