import wpilib


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

if __name__ == "__main__":
    wpilib.run(Robot)
