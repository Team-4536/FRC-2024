import robotHAL
import wpilib
from intakeStateMachine import IntakeStateMachine
from mechanism import Mechanism
from ntcore import NetworkTableInstance
from real import lerp
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import Scalar
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from shooterStateMachine import StateMachine

class RobotInputs():
    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)

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

        self.intake: bool = False

        self.tempShooterAim: float = 0.0

        self.ampShot: bool = False
        self.podiumShot: bool = False
        self.subwooferShot: bool = False
        self.shoot: bool = False

    def update(self) -> None:
        ##flipped x and y inputs so they are relative to bot
        self.driveX = self.xScalar(-self.driveCtrlr.getLeftY())
        self.driveY = self.yScalar(-self.driveCtrlr.getLeftX())
        self.turning = self.rotScalar(self.driveCtrlr.getRightX())

        self.speedCtrl = self.driveCtrlr.getRightTriggerAxis()

        self.gyroReset = self.driveCtrlr.getYButtonPressed()
        self.brakeButton = self.driveCtrlr.getBButtonPressed()
        self.absToggle = self.driveCtrlr.getXButtonPressed()

        # arm controller
        self.intake = self.armCtrlr.getAButton()

        self.tempShooterAim = -self.armCtrlr.getLeftY()
        self.tempShooterSpin: float = 1 if self.armCtrlr.getYButton() else 0
        # self.shootSpeaker: bool = arm.getYButton()
        # self.shootAmp: bool = arm.getBButton()
        # self.shooterIntake: bool = arm.getLeftBumper()

        #POV is also known as the Dpad
        if(self.armCtrlr.getPOV != -1):
            self.ampShot = self.armCtrlr.getPOV() < 100 and self.armCtrlr.getPOV() > 80  #right
            self.podiumShot = self.armCtrlr.getPOV() < 190 and self.armCtrlr.getPOV() > 170 #down
            self.subwooferShot = self.armCtrlr.getPOV() < 280  and self.armCtrlr.getPOV() > 260 #left
            self.shoot = self.armCtrlr.getPOV() < 10 or self.armCtrlr.getPOV() > 350
        else:
            self.ampShot = True
            self.podiumShot = False
            self.subwooferShot = False
            self.shoot = False


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

        self.shooterStateMachineState = 0


    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putBoolean("ctrl/absOffset", self.abs)

        self.table.putNumber("shooterStateMachine/state", self.shooterStateMachineState)
        self.table.putBoolean("shooterStateMachine/amp", self.input.ampShot)



    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.input.update()
        self.hal.stopMotors()

        if self.input.gyroReset:
            self.driveGyroYawOffset = self.hal.yaw

        if self.input.absToggle:
            self.abs = not self.abs

        speedControlEdited = lerp(1.5, 5.0, self.input.speedCtrl)
        turnScalar = 3

        self.table.putNumber("ctrl/driveX", self.input.driveX)
        self.table.putNumber("ctrl/driveY", self.input.driveY)
        driveVector = Translation2d(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited)
        if self.abs:
            driveVector = driveVector.rotateBy(Rotation2d(-self.hal.yaw + self.driveGyroYawOffset))
        speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), -self.input.turning * turnScalar)
        self.drive.update(self.time.dt, self.hal, speed)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x)
        self.table.putNumber("odomY", pose.y)

        self.hal.shooterAimSpeed = self.input.tempShooterAim * 0.1
        self.hal.shooterSpeed = self.input.tempShooterSpin * 0.4
        self.hal.shooterIntakeSpeed = self.input.tempShooterSpin * 0.4



        self.intakeStateMachine.update(self.hal, self.input.intake)
        self.shooterStateMachineState = self.shooterStateMachine.update(self.hal, self.input.ampShot, self.input.podiumShot, self.input.subwooferShot, self.input.shoot, False, self.time.dt)


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
