from gettext import translation
from xml.sax.xmlreader import InputSource
import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from utils import Scalar
from phoenix6.hardware import CANcoder
import robotHAL
from ntcore import NetworkTableInstance
from wpimath.kinematics import SwerveModuleState
import math
from swerveDrive import SwerveDrive
from timing import TimeData
from real import lerp
from wpimath.geometry import Translation2d
from robotHAL import RobotHAL, RobotHALBuffer
from mechanism import Mechanism


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        # scalars
        self.xScalar = Scalar(deadZone = .1, exponent = 1)
        self.yScalar = Scalar(deadZone = .1, exponent = 1)
        self.rotScalar = Scalar(deadZone = .1, exponent = 1)
        self.intakeScalar = Scalar(deadZone = .1, exponent = 1)
        self.testScalar = Scalar(deadZone = .1, exponent = 1)

        # drive
        ##flipped x and y inputs so they are relative to bot
        self.driveX: float = self.xScalar(-drive.getLeftY())
        self.driveY: float = self.yScalar(-drive.getLeftX())
        self.turning: float = self.rotScalar(drive.getRightX())

        self.speedCtrl: float = drive.getRightTriggerAxis()

        self.gyroReset: bool = drive.getYButtonPressed()
        self.brakeButton: bool = drive.getBButtonPressed()
        self.absToggle: bool = drive.getXButtonPressed()

        # arm controller
        self.intake: bool = arm.getAButton()
        self.shootSpeaker: bool = arm.getYButton()
        self.shootAmp: bool = arm.getBButton()
        self.shooterIntake: bool = arm.getLeftBumper()

        self.shooterJoystick: float = self.testScalar(-arm.getRightY())


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

        #testing stuff
        self.hal.intakeSpeeds[0] = self.table.getNumber("GreenIntakeTargetSpeed", 0.0)
        self.hal.intakeSpeeds[1] = self.table.getNumber("BlueIntakeTargetSpeed", 0.0)
        
        if self.input.intake:
            for i in range(2):
                self.hal.intakeSpeeds[i] = 0.4
        else:
            self.hal.intakeSpeeds[0] = 0
            self.hal.intakeSpeeds[1] = 0

        # for testing
        self.hal.shooterSpeed = self.input.shooterJoystick

        self.table.putNumber("ShooterSpeed", self.input.shooterJoystick)

        if self.input.shootSpeaker:
            self.hal.shooterSpeed = 0.25 # <-- not tested

        if self.input.shootAmp:
            self.hal.shooterSpeed = 0.1 # <-- not tested

        if self.input.shooterIntake:
            self.hal.shooterIntakeSpeed = 0.1

        #self.drive.update(self.time.dt, self.hal, speed)
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
