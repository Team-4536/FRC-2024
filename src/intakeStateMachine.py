from ntcore import NetworkTableInstance
from robotHAL import RobotHALBuffer


class IntakeStateMachine:
    START = 0
    INTAKING = 1
    STORING = 2
    def __init__(self):
        self.state = self.START

    def update(self, hal: RobotHALBuffer, beIntaking: bool):
        NetworkTableInstance.getDefault().getTable("stateMachines").putNumber("intake state", self.state)

        if(self.state == self.START):
            hal.intakeSpeeds = [0, 0]
            if(beIntaking):
                self.state = self.INTAKING

        elif(self.state == self.INTAKING):
            hal.intakeSpeeds = [0.4, 0.4]
            if(not beIntaking):
                self.state = self.START
#if(hal.intakeSensor):
                self.state = self.STORING

        elif(self.state == self.STORING):
            hal.intakeSpeeds = [0, 0]
        #    if not hal.intakeSensor:
           #     self.state = self.START