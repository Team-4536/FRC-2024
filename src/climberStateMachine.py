from ntcore import NetworkTableInstance
from robotHAL import RobotHALBuffer

class ClimberStateMachine():
    START = 0
    LOWERING = 1
    DOWN = 2

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ClimberStateMachine")
        self.state = self.LOWERING
    
    def publishInfo(self):
        self.table.putNumber("climber state machine state", self.state)

    def update(self, hal: RobotHALBuffer):
        if (self.state == self.START):
            self.state = self.LOWERING
        if(self.state == self.LOWERING):
            print("FUCK")
            hal.climberSpeed = -3
            if(hal.climberLimitPressed):
                self.state = self.DOWN
        elif(self.state == self.DOWN):
            hal.climberSpeed = 0
            pass
        return self.state
