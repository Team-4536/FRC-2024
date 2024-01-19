import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from phoenix6.hardware import CANcoder
import robotHAL
from ntcore import NetworkTableInstance
from wpimath.kinematics import SwerveModuleState

from swerveDrive import SwerveDrive
from timing import TimeData


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        pass

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("targetAngle", 0.000)
        self.table.putNumber("targetSpeed", 0.000)

        self.driveCtrlr = wpilib.XboxController(0)
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), [])
        self.time = TimeData(None)

    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        if(self.driveCtrlr.getAButtonPressed()):
            self.hal.yaw = 0

        targetA = self.table.getNumber("targetAngle", 0.0)
        targetS = self.table.getNumber("targetSpeed", 0.0)

        self.drive.update(self.time.dt, self.hal, SwerveModuleState(targetS, Rotation2d(targetA)))
        self.hardware.update(self.hal)

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)


if __name__ == "__main__":
    wpilib.run(Robot)