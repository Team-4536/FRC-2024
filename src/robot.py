import math
import profiler
import robotHAL
import wpilib
from intakeStateMachine import IntakeStateMachine
from mechanism import Mechanism
from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from shooterStateMachine import StateMachine
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import Scalar
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from PIDController import PIDController


class RobotInputs():
    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4) 

        self.xScalar = Scalar(deadZone = .1, exponent = 1)
        self.yScalar = Scalar(deadZone = .1, exponent = 1)
        self.rotScalar = Scalar(deadZone = .1, exponent = 1)

        self.driveX: float = 0.0
        self.driveY: float = 0.0
        self.turning: float = 0.0
        self.speedCtrl: float = 0.0
        self.gyroReset: bool = False
        self.brakeButton: bool = False
        self.absToggle: bool = False

        self.turningPIDButton: float = 0.0

        self.intake: bool = False

        self.shooterAimManual: float = 0.0
        # self.shooterFeedManual: float = 0.0
        # self.shooterSpeedManual: float = 0.0

        self.ampShot: bool = False
        self.podiumShot: bool = False
        self.subwooferShot: bool = False
        self.rev: bool = False
        self.shoot: bool = False

        self.camTemp: float = 0.0
        # self.stateMachineOverrideToggle: bool = False

    def update(self) -> None:
        ##flipped x and y inputs so they are relative to bot
        self.driveX = self.xScalar(-self.driveCtrlr.getLeftY())
        self.driveY = self.yScalar(-self.driveCtrlr.getLeftX())

        self.turning = self.rotScalar(self.driveCtrlr.getRightX())
       
        self.turningPIDButton = self.driveCtrlr.getYButton()

        self.speedCtrl = self.driveCtrlr.getRightTriggerAxis()

        self.gyroReset = self.driveCtrlr.getYButtonPressed()
        self.brakeButton = self.driveCtrlr.getBButtonPressed()
        self.absToggle = self.driveCtrlr.getXButtonPressed()


        # arm controller
        self.intake = self.armCtrlr.getAButton()

        self.shooterAimManual = -self.armCtrlr.getLeftY()

        #POV is also known as the Dpad
        if(self.armCtrlr.getPOV() != -1):
            self.ampShot = self.armCtrlr.getPOV() < 190 and self.armCtrlr.getPOV() > 170 #down
            self.podiumShot = self.armCtrlr.getPOV() < 280  and self.armCtrlr.getPOV() > 260 #left
            self.subwooferShot = self.armCtrlr.getPOV() < 10 or self.armCtrlr.getPOV() > 350 # up
        else:
            self.ampShot = False
            self.podiumShot = False
            self.subwooferShot = False

        self.rev = self.armCtrlr.getLeftTriggerAxis() > 0.2
        self.shoot = self.armCtrlr.getLeftBumper()

        self.camTemp = -self.armCtrlr.getRightY()


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.input = RobotInputs()

        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), wheelPositions)
        self.time = TimeData(None)

        self.abs = True
        self.driveGyroYawOffset = 0.0 # the last angle that drivers reset the field oriented drive to zero at

        self.intakeStateMachine = IntakeStateMachine()

        self.shooterStateMachine = StateMachine()

    def robotPeriodic(self) -> None:
        profiler.start()
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putBoolean("ctrl/absOffset", self.abs)

        self.table.putNumber("shooterStateMachine/state", self.shooterStateMachine.state)
        self.table.putBoolean("shooterStateMachine/amp", self.input.ampShot)
        self.table.putNumber("shooterStateMachine/targetSpeed", self.shooterStateMachine.speedSetpoint)
        self.table.putNumber("shooterStateMachine/targetSpeedActual", self.shooterStateMachine.PIDspeedSetpoint)
        self.table.putNumber("shooterStateMachine/targetAim", self.shooterStateMachine.aimSetpoint)
        self.table.putNumber("shooterStateMachine/targetAimActual", self.shooterStateMachine.PIDaimSetpoint)

        profiler.end("robotPeriodic")
    
    def teleopInit(self) -> None:
        self.shooterStateMachine.state = 0

        self.turnPID = PIDController(0, 0, 0)

        self.currentAngle = self.hal.yaw
    def teleopPeriodic(self) -> None:
        frameStart = wpilib.getTime()
        self.input.update()
        self.hal.stopMotors()

        if self.input.gyroReset:
            self.driveGyroYawOffset = self.hal.yaw

        if self.input.absToggle:
            self.abs = not self.abs

        speedControlEdited = lerp(1.5, 5.0, self.input.speedCtrl)
        turnScalar = 3

        profiler.start()
        self.table.putNumber("ctrl/driveX", self.input.driveX)
        self.table.putNumber("ctrl/driveY", self.input.driveY)
        driveVector = Translation2d(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited)
        if self.abs:
            driveVector = driveVector.rotateBy(Rotation2d(-self.hal.yaw + self.driveGyroYawOffset))

     

        if self.input.driveCtrlr.getPOV() < 190 and self.input.driveCtrlr.getPOV() > 170:
            targetAngle = 180
        elif self.input.driveCtrlr.getPOV() < 280  and self.input.driveCtrlr.getPOV() > 260: #left
            targetAngle = 90
        elif self.input.driveCtrlr.getPOV() < 10 or self.input.driveCtrlr.getPOV() > 350:
            targetAngle = 0

        if not self.input.turningPIDButton:
            speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), -self.input.turning * turnScalar)
        else:
            speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), self.turnPID.tickErr(angleWrap(self.currentAngle - targetAngle), targetAngle, self.time.dt) * turnScalar)
            
        self.drive.update(self.time.dt, self.hal, speed)
    
        profiler.end("drive updates")

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x)
        self.table.putNumber("odomY", pose.y)

        self.table.putNumber("POV", self.input.armCtrlr.getPOV())
        self.table.putBoolean("amp", self.input.ampShot)
        self.table.putBoolean("shoot", self.input.shoot)

        profiler.start()
        self.intakeStateMachine.update(self.hal, self.input.intake)
        profiler.end("intake state machine")

        profiler.start()
        self.shooterStateMachine.update(
            self.hal,
            self.input.ampShot,
            self.input.podiumShot,
            self.input.subwooferShot,
            self.input.rev,
            self.input.shoot,
            self.input.shooterAimManual,
            self.time.timeSinceInit,
            self.time.dt)
        profiler.end("shooter state machine")

        self.hal.camSpeed = self.input.camTemp * 0.2

        profiler.start()
        self.hardware.update(self.hal)
        profiler.end("hardware update")
        self.table.putNumber("TIME FOR FRAME", wpilib.getTime() - frameStart)

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
