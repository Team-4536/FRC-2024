import robotHAL
import swerveDrive
import timing
import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveModulePosition


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        self.driveX = drive.getLeftX()
        self.driveY = drive.getLeftY()
        self.turning = drive.getRightX()

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.time = timing.TimeData(None)

        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.input: RobotInputs = RobotInputs(self.driveCtrlr, self.armCtrlr)

        self.telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")

        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)

        # (Rob): I would have built the SwerveModulePosition class directly into the HALBuffer, if it weren't for the fact that python can't 'pickle' them. (???)
        self.drive = swerveDrive.SwerveDrive(Rotation2d(0), Pose2d(0, 0, 0),
            [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)])

    def robotPeriodic(self) -> None:
        self.time = timing.TimeData(self.time)
        self.hal.publish(self.telemetryTable)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.input = RobotInputs(self.driveCtrlr, self.armCtrlr)

        speed = (self.input.driveX * 0.1, self.input.driveY * 0.1, self.input.turning * 0.1)
        self.drive.update(self.time.dt, self.hal, speed)

        self.hardware.update(self.hal)

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)

if __name__ == "__main__":
    wpilib.run(Robot)