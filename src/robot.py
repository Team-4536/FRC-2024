

import robotHAL
import wpilib
import wpimath.controller
from inputs import deadZone
from ntcore import NetworkTableInstance
from PIDController import PIDController
from real import lerp
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.controller import (
    HolonomicDriveController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from wpimath.trajectory import TrajectoryUtil, TrapezoidProfileRadians


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        self.driveX: float = deadZone(drive.getLeftX())
        self.driveY: float = deadZone(-drive.getLeftY())
        self.turning: float = deadZone(drive.getRightX())
        self.speedCtrl: float = drive.getRightTriggerAxis()

        self.gyroReset: bool = drive.getYButtonPressed()
        self.brakeButton: bool = drive.getBButtonPressed()
        self.absToggle: bool = drive.getXButtonPressed()


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.input = RobotInputs(self.driveCtrlr, self.armCtrlr)

        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), wheelPositions)
        self.time = TimeData(None)

        self.abs = True


    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.input = RobotInputs(self.driveCtrlr, self.armCtrlr)

        if self.input.gyroReset:
            self.hal.yaw = 0

        if self.input.absToggle:
            self.abs = not self.abs

        defaultSpeed = 1
        maxSpeed = 4
        speedControlEdited = lerp(defaultSpeed, maxSpeed, self.input.speedCtrl)
        turnScalar = 3

        driveVector = Translation2d(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited)
        driveVector = driveVector.rotateBy(Rotation2d(-self.hal.yaw))

        if self.abs:
            speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), -self.input.turning * turnScalar)
        else:
            speed = ChassisSpeeds(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited, -self.input.turning * turnScalar)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)

        self.drive.update(self.time.dt, self.hal, speed)
        self.hardware.update(self.hal)

    def autonomousInit(self) -> None:
        self.autoStartTime = self.time.timeSinceInit
        XControllerP = 1
        XControllerI = 0
        XControllerD = 0
        YControllerP = 1
        YControllerI = 0
        YControllerD = 0
        RControllerP = 1
        RControllerI = 0
        RControllerD = 0
        T_PConstraintsVolocityMax = 6.28
        T_PConstraintsRotaionAccelerationMax = 1
        self.XController = wpimath.controller.PIDController(
            XControllerP, XControllerI, XControllerD)
        self.YController = wpimath.controller.PIDController(
            YControllerP, YControllerI, YControllerD)
        self.RotationController = ProfiledPIDControllerRadians(
            RControllerP, RControllerI, RControllerD, TrapezoidProfileRadians.Constraints(T_PConstraintsVolocityMax, T_PConstraintsRotaionAccelerationMax))
        trajectoryJSON = "src/deploy/output/test.wpilib.json"
        self.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryJSON)
        self.holonomicController = HolonomicDriveController(
            self.XController, self.YController, self.RotationController)

    def autonomousPeriodic(self) -> None:
        trajectoryHeadingAngle = 0
        # self.hal.stopMotors()
        CurrentPose = self.drive.odometry.getPose()
        goal = self.trajectory.sample(self.time.timeSinceInit - self.autoStartTime)
        self.table.putNumber("pathTargetX", goal.pose.X())
        self.table.putNumber("pathTargetY", goal.pose.Y())
        self.table.putNumber("velocitytargt", goal.velocity)
        adjustedSpeeds = self.holonomicController.calculate(
            CurrentPose, goal, Rotation2d(trajectoryHeadingAngle))
        self.drive.update(self.time.dt, self.hal, adjustedSpeeds)
        self.hardware.update(self.hal)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)


if __name__ == "__main__":
    wpilib.run(Robot)