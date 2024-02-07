import auto
import robotHAL
import stages
import wpilib
import wpimath.controller
from mechanism import Mechanism
from ntcore import NetworkTableInstance
from real import lerp
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import Scalar
from wpimath.controller import HolonomicDriveController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from wpimath.trajectory import Trajectory, TrajectoryUtil, TrapezoidProfileRadians


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
        self.odometryReset: bool = False

        self.intake: float = 0.0

    def update(self) -> None:
        ##flipped x and y inputs so they are relative to bot
        self.driveX = self.xScalar(-self.driveCtrlr.getLeftY())
        self.driveY = self.yScalar(-self.driveCtrlr.getLeftX())
        self.turning = self.rotScalar(self.driveCtrlr.getRightX())

        self.speedCtrl = self.driveCtrlr.getRightTriggerAxis()

        self.odometryReset = self.driveCtrlr.getStartButtonPressed()
        self.gyroReset = self.driveCtrlr.getYButtonPressed()
        self.brakeButton = self.driveCtrlr.getBButtonPressed()
        self.absToggle = self.driveCtrlr.getXButtonPressed()

        self.intake = float(self.armCtrlr.getAButton()) - float(self.armCtrlr.getXButton())
        # self.shootSpeaker: bool = arm.getYButton()
        # self.shootAmp: bool = arm.getBButton()
        # self.shooterIntake: bool = arm.getLeftBumper()

AUTO_SIDE_RED = "red"
AUTO_SIDE_BLUE = "blue"

AUTO_NONE = "none"
AUTO_SHOOT = "shoot"
AUTO_SHOOT_AND_EXIT = "shoot and exit"

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

        self.mech = Mechanism(self.hal)

        self.autoSideChooser = wpilib.SendableChooser()
        self.autoSideChooser.setDefaultOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)
        wpilib.SmartDashboard.putData('auto side chooser', self.autoSideChooser)
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption(AUTO_NONE, AUTO_NONE)
        self.autoChooser.addOption(AUTO_SHOOT, AUTO_SHOOT)
        self.autoChooser.addOption(AUTO_SHOOT_AND_EXIT, AUTO_SHOOT_AND_EXIT)
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)



    def robotPeriodic(self) -> None:
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        self.drive.updateOdometry(self.hal)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)

        if self.input.odometryReset:
            self.drive.resetOdometry(Pose2d(0,0,Rotation2d(0)), self.hal)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putBoolean("ctrl/absOffset", self.abs)

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

        if self.autoSideChooser.getSelected() == AUTO_SIDE_RED:
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
        if self.autoChooser.getSelected() == AUTO_NONE:
            stageList = []
        elif self.autoChooser.getSelected() == AUTO_SHOOT:
            stageList = []
        else:
            assert(False)
        self.auto = auto.Auto(stageList, self.time.timeSinceInit)

        self.hardware.gyro.setAngleAdjustment(180)
        self.hardware.update(self.hal)
        self.drive.resetOdometry(firstPose, self.hal)

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




