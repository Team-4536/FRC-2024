from ntcore import NetworkTableInstance
from robotHAL import RobotHALBuffer


class IntakeStateMachine:
    START = 0
    INTAKING = 1
    STORING = 2
    def __init__(self):
        self.state = self.START

    def update(self, hal: RobotHALBuffer, startIntaking: bool):
        NetworkTableInstance.getDefault().getTable("stateMachines").putNumber("intake state", self.state)

        if(self.state == self.START):
            hal.intakeSpeeds = [0, 0]
            if(startIntaking):
                self.state = self.INTAKING

        if(self.state == self.INTAKING):
            hal.intakeSpeeds = [0.4, 0.4]
            if(hal.intakeSensor):
                self.state = self.STORING

        if(self.state == self.STORING):
            hal.intakeSpeeds = [0, 0]
            if not hal.intakeSensor:
                self.state = self.START