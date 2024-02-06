from re import X
from tkinter import colorchooser

import auto
import robotHAL
import stages
import wpilib
import wpimath.controller
from hal import getAnalogGyroAngle
from ntcore import NetworkTableInstance
from phoenix6.hardware import CANcoder
from PIDController import PIDController
from real import lerp
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import Scalar
from wpimath.controller import HolonomicDriveController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from wpimath.trajectory import Trajectory, TrajectoryUtil, TrapezoidProfileRadians


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

AUTO_NONE = "none"
AUTO_RED = "red"
AUTO_BLUE = "blue"
AUTO_SHOOTCURRENT = "shoot-current"
AUTO_SHOOTEXIT = 'shoot-and-exit'
AUTO_MIDDLE = 'get-middle-ring'
AUTO_RIGHT = 'get-right-ring'
AUTO_LEFT = 'get-left-ring'
AUTO_ALL = 'get-all-rings'


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

        self.ColorChooser = wpilib.SendableChooser()
        self.ColorChooser.setDefaultOption(AUTO_BLUE, AUTO_BLUE)
        self.ColorChooser.addOption(AUTO_RED, AUTO_RED)
        self.RingChooser = wpilib.SendableChooser()
        self.RingChooser.setDefaultOption(AUTO_NONE, AUTO_NONE)
        self.RingChooser.addOption(AUTO_MIDDLE, AUTO_MIDDLE)
        self.RingChooser.addOption(AUTO_LEFT, AUTO_LEFT)
        self.RingChooser.addOption(AUTO_RIGHT, AUTO_RIGHT)
        self.RingChooser.addOption(AUTO_ALL, AUTO_ALL)
        wpilib.SmartDashboard.putData('color chooser', self.ColorChooser)
        wpilib.SmartDashboard.putData('ring chooser', self.RingChooser)



    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)

        if self.input.odometryReset:
            self.drive.resetOdometry(Pose2d(0,0,Rotation2d(0)), self.hal)
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
        self.drive.resetOdometry(Pose2d(1.166,5.522,Rotation2d(0)), self.hal)
        self.table.putNumber("path/Xp", 1)
        self.table.putNumber("path/Yp", 1)
        self.table.putNumber('path/Rp', 7)
        self.XController = wpimath.controller.PIDController(self.table.getNumber("path/Xp", 0.0), 0, 0)
        self.YController = wpimath.controller.PIDController(self.table.getNumber("path/Yp", 0.0), 0, 0)
        self.RotationController = ProfiledPIDControllerRadians(self.table.getNumber("path/Rp", 0.0), 0, 0,
             TrapezoidProfileRadians.Constraints(6.28, 1))
        self.holonomicController = HolonomicDriveController(self.XController, self.YController, self.RotationController)

        trajPath = ""
        filePostfix = ""
        if self.isSimulation():
            trajPath = "src/deploy/output/"
        else:
            trajPath = "/home/lvuser/py/deploy/output/"

        if self.ColorChooser.getSelected() == AUTO_RED:
            filePostfix = "Red.wpilib.json"
        else:
            filePostfix = "Blue.wpilib.json"

        self.trajectory_middleA = TrajectoryUtil.fromPathweaverJson(trajPath + "middle" + filePostfix)
        self.trajectory_middleB = TrajectoryUtil.fromPathweaverJson(trajPath + "middleBack" + filePostfix)
        self.trajectory_leftA = TrajectoryUtil.fromPathweaverJson(trajPath + "left" + filePostfix)
        self.trajectory_leftB = TrajectoryUtil.fromPathweaverJson(trajPath + "leftBack" + filePostfix)
        self.trajectory_rightA = TrajectoryUtil.fromPathweaverJson(trajPath + "right" + filePostfix)
        self.trajectory_rightB = TrajectoryUtil.fromPathweaverJson(trajPath + "rightBack" + filePostfix)

        stageList: list[auto.Stage] = []
        firstPose = Pose2d()
        if self.RingChooser.getSelected() == AUTO_MIDDLE:
                firstPose = Trajectory.initialPose(self.trajectory_middleA)
                stageList = [
                    stages.makeTelemetryStage('middle ring'),
                    stages.makePathStage(self.trajectory_middleA),
                    stages.makeIntakeStage(10, 0.2),
                    stages.makePathStage(self.trajectory_middleB)
                    ]
        else:
            assert(False)

        self.hal.input(firstPose.rotation())
        self.drive.resetOdometry(firstPose, self.hal)

        self.auto = auto.Auto(stageList, self.time.timeSinceInit)

    def autonomousPeriodic(self) -> None:
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




