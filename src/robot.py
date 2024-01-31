import auto
import robotHAL
import stages
import wpilib
import wpimath.controller
from ntcore import NetworkTableInstance
from real import lerp
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import Scalar
from wpimath.controller import (
    HolonomicDriveController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from wpimath.trajectory import TrajectoryUtil, TrapezoidProfileRadians


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        self.xScalar = Scalar(deadZone = .1, exponent = 1)
        self.yScalar = Scalar(deadZone = .1, exponent = 1)
        self.rotScalar = Scalar(deadZone = .1, exponent = 1)


        ##flipped x and y inputs so they are relative to bot
        self.driveX: float = self.xScalar(-drive.getLeftY())
        self.driveY: float = self.yScalar(-drive.getLeftX())
        self.turning: float = self.rotScalar(drive.getRightX())

        self.speedCtrl: float = drive.getRightTriggerAxis()

        self.gyroReset: bool = drive.getYButtonPressed()
        self.brakeButton: bool = drive.getBButtonPressed()
        self.absToggle: bool = drive.getXButtonPressed()
        self.odometryReset: bool = drive.getStartButtonPressed()


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.input = RobotInputs(self.driveCtrlr, self.armCtrlr)

        #def myOdometryReset(self) -> None:

        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), wheelPositions)
        self.time = TimeData(None)

        self.abs = True


    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)

        if self.input.odometryReset:
            wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
            self.drive.odometry.resetPosition(Rotation2d(self.hal.yaw), tuple(wheelPositions) ,Pose2d(0, 0, Rotation2d(self.hal.yaw))) # type: ignore

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

        self.drive.update(self.time.dt, self.hal, speed)
        self.hardware.update(self.hal)

    def autonomousInit(self) -> None:
        self.hal.yaw = 0
        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive.odometry.resetPosition(Rotation2d(self.hal.yaw), tuple(wheelPositions) ,Pose2d(0, 0, Rotation2d(self.hal.yaw))) # type: ignore
        self.autoStartTime = self.time.timeSinceInit
        XControllerP = 1
        XControllerI = 0
        XControllerD = 0
        self.table.putNumber("path/Xp", XControllerP)
        YControllerP = 1
        YControllerI = 0
        YControllerD = 0
        self.table.putNumber("path/Yp", YControllerP)
        RControllerP = 7
        RControllerI = 0
        RControllerD = 0
        self.table.putNumber('path/Rp', RControllerP)
        T_PConstraintsVolocityMax = 6.28
        T_PConstraintsRotaionAccelerationMax = 1
        self.XController = wpimath.controller.PIDController(
            XControllerP, XControllerI, XControllerD)
        self.YController = wpimath.controller.PIDController(
            YControllerP, YControllerI, YControllerD)
        self.RotationController = ProfiledPIDControllerRadians(
            RControllerP, RControllerI, RControllerD, TrapezoidProfileRadians.Constraints(T_PConstraintsVolocityMax, T_PConstraintsRotaionAccelerationMax))
        if self.isSimulation():
            trajectoryJSON = "src/deploy/output/test.wpilib.json"
        else:
            trajectoryJSON = "/home/lvuser/py/deploy/output/test.wpilib.json"
        #trajectoryJSON = "/home/lvuser/py/deploy/output/test.wpilib.json"
        self.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryJSON)
        self.holonomicController = HolonomicDriveController(
             self.XController, self.YController, self.RotationController)
        # self.table.putNumber("path/TargetX", 0)
        # self.table.putNumber("path/TargetY", 0)
        # self.table.putNumber("path/TargetR", 0)

        self.auto = auto.Auto([
            stages.makeTelemetryStage("init"),
            stages.makePathStage(self.trajectory),
            stages.makeTelemetryStage("done")
            ], self.time.timeSinceInit)

    def autonomousPeriodic(self) -> None:
        # currentPose = self.drive.odometry.getPose()
        # goal = self.trajectory.sample(self.time.timeSinceInit - self.autoStartTime)

        # # goal = Trajectory.State()
        # targetX = self.table.getNumber("path/TargetX", 0)
        # targetY = self.table.getNumber("path/TargetY", 0)
        # targetR = self.table.getNumber("path/TargetR", 0)
        # # assert(targetX is not None)
        # # assert(targetY is not None)
        # # assert(targetR is not None)
        # # # #goal = TrapezoidProfile.State(0, 0)
        # self.table.putNumber("pathTargetX", goal.pose.X())
        # self.table.putNumber("pathTargetY", goal.pose.Y())
        # self.table.putNumber('pathTargetRotation', goal.pose.rotation().radians())
        # self.table.putNumber("velocitytargt", goal.velocity)

        # xSpeed = self.XController.calculate(currentPose.X(), targetX)
        # ySpeed = self.YController.calculate(currentPose.Y(), targetY)
        # rSpeed = self.RotationController.calculate(currentPose.rotation().radians(), targetR)
        # t = Translation2d(xSpeed, ySpeed).rotateBy(Rotation2d(-self.hal.yaw))
        #driveSpeed = ChassisSpeeds(t.x, t.y, rSpeed)

        # adjustedSpeeds = self.holonomicController.calculate(
        #     currentPose, goal, goal.pose.rotation())
        # self.XController.setP(self.table.getNumber("path/Xp", 1.0))
        # self.YController.setP(self.table.getNumber('path/Yp', 1.0))
        # self.RotationController.setP(self.table.getNumber('path/Rp', 1.0))

        self.hal.stopMotors()
        self.auto.update(self)
        self.hardware.update(self.hal)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)


if __name__ == "__main__":
    wpilib.run(Robot)
