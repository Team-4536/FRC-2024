import profiler
import robotHAL
import wpilib
import timing
import swerveDrive

from intakeStateMachine import IntakeStateMachine
from mechanism import Mechanism
from ntcore import NetworkTableInstance
from real import lerp
from shooterStateMachine import StateMachine
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
         

        self.time = timing.TimeData(None)

        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        #self.input: RobotInputs = RobotInputs(self.driveCtrlr, self.armCtrlr)

        self.telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")
        self.limelightTable = NetworkTableInstance.getDefault().getTable("limelight-mb")
        self.robotPoseTable = NetworkTableInstance.getDefault().getTable("robot pose")

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

        wpilib.SmartDashboard.putData('Field', self.drive.field)

    def robotPeriodic(self) -> None:
        profiler.start()
        self.time = TimeData(self.time)
        self.hal.publish(self.table)
        

        #self.drive = swerveDrive.SwerveDrive(Rotation2d(0), Pose2d(5.60, 1.39, 2),
        #    [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)])

    
        
        self.visionPose = self.limelightTable.getNumberArray("botpose", [0,0,0,0,0,0,0])
        self.robotPoseTable.putNumber("xPos", self.visionPose[0])
        self.robotPoseTable.putNumber("yPos", self.visionPose[1])
        self.robotPoseTable.putNumber("yaw", self.visionPose[5])
        if not (self.visionPose[0] == 0 and self.visionPose[1] == 0 and self.visionPose[5] == 0):  
            self.visionPose2D = Pose2d(self.visionPose[0], self.visionPose[1], self.visionPose[5])
            self.drive.odometry.addVisionMeasurement(self.visionPose2D, wpilib.Timer.getFPGATimestamp())
        self.robotPose = self.drive.odometry.getEstimatedPosition()
        self.robotX = self.robotPose.X()
        self.robotY = self.robotPose.Y()
        self.robotTheta = self.robotPose.rotation().radians() 
        self.robotPoseTable.putNumber("robotX" , self.robotX)
        self.robotPoseTable.putNumber("robotY" , self.robotY)
        self.robotPoseTable.putNumber("robotXTest" , self.robotPose.x)
        self.robotPoseTable.putNumber("robotYTest" , self.robotPose.y)
        self.robotPoseTable.putNumber("robotTheta" , self.robotTheta)
        
        self.hal.publish(self.telemetryTable)
        self.hal.publish(self.limelightTable)
        self.drive.updateOdometry(self.hal)

    def teleopInit(self) -> None:
        self.shooterStateMachine.state = 0

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
        speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), -self.input.turning * turnScalar)
        self.drive.update(self.time.dt, self.hal, speed)
        profiler.end("drive updates")

        #pose = self.drive.odometry.getPose()
        # self.table.putNumber("odomX", pose.x)
        # self.table.putNumber("odomY", pose.y)

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
