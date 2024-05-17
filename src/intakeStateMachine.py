from ntcore import NetworkTableInstance
from robotHAL import RobotHALBuffer


class IntakeStateMachine:
    def __init__(self, hal: RobotHALBuffer):
        self.__generator = self.__update(hal)
        self.__intakeThisFrame: bool = False

        self.__table = NetworkTableInstance.getDefault().getTable("stateMachines")
        self.__table.putBoolean("storing", False)
        self.__table.putBoolean("intakeThisFrame", False)

    def setIntaking(self, intake: bool) -> None:
        self.intakeThisFrame = intake

    def update(self):
        self.__generator.__next__()

    def __update(self, hal: RobotHALBuffer):
        while True:
            hal.intakeSpeeds = [0, 0]

            if not hal.intakeSensor:
                if self.__intakeThisFrame:
                    hal.intakeSpeeds = [0.7, 0.7]

            yield 0
