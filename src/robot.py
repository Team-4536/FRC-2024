import robotHAL
import wpilib
import math
from mechanism import Mechanism
from ntcore import NetworkTableInstance
from real import lerp
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import Scalar
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition

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

        self.intake: float = 0.0

    def update(self) -> None:
        ##flipped x and y inputs so they are relative to bot
        self.driveX = self.xScalar(-self.driveCtrlr.getLeftY())
        self.driveY = self.yScalar(-self.driveCtrlr.getLeftX())
        self.turning = self.rotScalar(self.driveCtrlr.getRightX())

        self.speedCtrl = self.driveCtrlr.getRightTriggerAxis()
        self.limelightRingButton = self.driveCtrlr.getAButton()
        self.gyroReset = self.driveCtrlr.getYButtonPressed()
        self.brakeButton = self.driveCtrlr.getBButtonPressed()
        self.absToggle = self.driveCtrlr.getXButtonPressed()

        # arm controller
        self.intake = float(self.armCtrlr.getAButton()) - float(self.armCtrlr.getXButton())
        # self.shootSpeaker: bool = arm.getYButton()
        # self.shootAmp: bool = arm.getBButton()
        # self.shooterIntake: bool = arm.getLeftBumper()


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.limelightTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.tx = self.limelightTable.putNumber("tx", 50)
        self.ty = self.limelightTable.putNumber("ty", 50)
        self.input = RobotInputs()
        self.minimumLimelightYValue = -300

        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), wheelPositions)
        self.time = TimeData(None)

        self.abs = True
        self.driveGyroYawOffset = 0.0 # the last angle that drivers reset the field oriented drive to zero at

        self.mech = Mechanism(self.hal)


    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putBoolean("ctrl/absOffset", self.abs)

    def teleopInit(self) -> None:
        pass

    def limelightRingDrive(self):
        self.tx = self.limelightTable.getNumber("tx", 0.0)
        self.ty = self.limelightTable.getNumber("ty", 0.0)
        self.ringTurnAngle = self.tx
        self.ringTurnRadians = math.radians(self.ringTurnAngle)
        self.drive.update(self.time.dt, self.hal, ChassisSpeeds(0, 0, self.ringTurnRadians*0.1))
        self.limelightYError = self.ty - self.minimumLimelightYValue
        if self.limelightYError <100:
            self.hal.intakeSpeeds = [0.2, 0.2]
            self.limelightTable.putNumber("intakeRunning", 1)
        else:
            self.limelightTable.putNumber("intakeRunning", 0)
        #this is manually modifying tx which should be done by the limelight on the robot
        if(self.tx>0):
            self.tx = self.limelightTable.putNumber("tx", self.tx - 1)
        elif(self.tx<0):
            self.tx = self.limelightTable.putNumber("tx", self.tx + 1)
        #we need this    
        if self.tx == 0:
            self.drive.update(self.time.dt, self.hal, ChassisSpeeds(self.limelightYError, 0, 0))
        #this is manually modifying the ty which should be done by the limelight on the robot
            if self.ty > self.minimumLimelightYValue:
                self.ty = self.limelightTable.putNumber("ty", self.ty - 1)

      
    def teleopPeriodic(self) -> None:

        self.input.update()
        self.hal.stopMotors()

        if self.input.gyroReset:
            self.driveGyroYawOffset = self.hal.yaw

        if self.input.absToggle:
            self.abs = not self.abs

        speedControlEdited = lerp(1.5, 5.0, self.input.speedCtrl)
        turnScalar = 3
        if self.input.limelightRingButton:
            self.limelightRingDrive()
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

        if self.input.intake:
            self.hal.intakeSpeeds = [0.4 * self.input.intake, 0.4 * self.input.intake]
        else:
            self.hal.intakeSpeeds = [0.0, 0.0]

        #shooter
        # if self.input.shootSpeaker:
        #     self.hal.shooterSpeed = 0.25

        # if self.input.shootAmp:
        #     self.hal.shooterSpeed = 0.1

        # if self.input.shooterIntake:
        #     self.hal.shooterIntakeSpeed = 0.1

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


