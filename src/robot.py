import wpilib
import timing


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        pass


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.time = timing.TimeData(None)
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.input: RobotInputs = RobotInputs(self.driveCtrlr, self.armCtrlr)

    def robotPeriodic(self) -> None:
        self.time = timing.TimeData(self.time)

    def teleopInit(self) -> None:
        self.input = RobotInputs(self.driveCtrlr, self.armCtrlr)

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousExit(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        pass
