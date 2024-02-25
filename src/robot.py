import auto
import profiler
import robotHAL
import stages
import wpilib
from intakeStateMachine import IntakeStateMachine
from ntcore import NetworkTableInstance
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from PIDController import PIDController, PIDControllerForArm
from real import lerp
from shooterStateMachine import ShooterTarget, StateMachine
from simHAL import RobotSimHAL
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

        self.aim: ShooterTarget = ShooterTarget.NONE
        self.rev: bool = False
        self.shoot: bool = False

        self.camTemp: float = 0.0

        self.overideShooterStateMachine: bool = False
        self.overideIntakeStateMachine: bool = False

        self.manualAimJoystickY: float = 0
        self.aimEncoderReset: bool = False
        self.manualFeedMotor: bool = False


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
        self.intakeReverse = self.armCtrlr.getBButton()

        self.shooterAimManual = -self.armCtrlr.getLeftY()

        #POV is also known as the Dpad, 0 is centered on top, angles go clockwise
        self.aim = ShooterTarget.NONE
        if(self.armCtrlr.getPOV() != -1):
            if self.armCtrlr.getPOV() < 10 or self.armCtrlr.getPOV() > 350: # up
                self.aim = ShooterTarget.SUBWOOFER
            elif self.armCtrlr.getPOV() < 280  and self.armCtrlr.getPOV() > 260: # left
                self.aim = ShooterTarget.PODIUM
            elif self.armCtrlr.getPOV() < 190 and self.armCtrlr.getPOV() > 170: # down
                self.aim = ShooterTarget.AMP

        self.rev = self.armCtrlr.getLeftTriggerAxis() > 0.2
        self.shoot = self.armCtrlr.getLeftBumper()

        self.camTemp = -self.armCtrlr.getRightY()


        if(self.armCtrlr.getYButtonPressed()):
            self.overideShooterStateMachine = not self.overideShooterStateMachine
            self.overideIntakeStateMachine = self.overideShooterStateMachine

        self.manualFeedMotor = self.armCtrlr.getRightTriggerAxis() > 0.2
        self.manualFeedReverseMotor = self.armCtrlr.getRightBumper()
        self.manualAimJoystickY = self.armCtrlr.getLeftY()
        self.aimEncoderReset = self.armCtrlr.getLeftStickButtonPressed()
        self.camEncoderReset = self.armCtrlr.getRightStickButtonPressed()

AUTO_SIDE_RED = "red"
AUTO_SIDE_BLUE = "blue"
AUTO_SIDE_FMS = "FMS side"

AUTO_NONE = "none"
AUTO_INTAKE_CENTER_RING = "grab center ring"
AUTO_EXIT = "exit"
AUTO_GET_ALL = "grab all"
AUTO_SIDE_UPPER = 'go from speaker side to upper ring'
AUTO_SIDE_LOWER = 'go from side of speaker and get lower ring'
AUTO_SHOOT_PRELOADED = 'shoot preloaded ring'

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()

        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.input = RobotInputs()

        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), wheelPositions)

        self.abs = True
        self.driveGyroYawOffset = 0.0 # the last angle that drivers reset the field oriented drive to zero at

        self.intakeStateMachine = IntakeStateMachine()
        self.shooterStateMachine = StateMachine()

        self.autoSideChooser = wpilib.SendableChooser()
        self.autoSideChooser.setDefaultOption(AUTO_SIDE_FMS, AUTO_SIDE_FMS)
        self.autoSideChooser.addOption(AUTO_SIDE_RED, AUTO_SIDE_RED)
        self.autoSideChooser.addOption(AUTO_SIDE_BLUE, AUTO_SIDE_BLUE)
        wpilib.SmartDashboard.putData('auto side chooser', self.autoSideChooser)
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption(AUTO_NONE, AUTO_NONE)
        self.autoChooser.addOption(AUTO_INTAKE_CENTER_RING, AUTO_INTAKE_CENTER_RING)
        self.autoChooser.addOption(AUTO_EXIT, AUTO_EXIT)
        self.autoChooser.addOption(AUTO_GET_ALL, AUTO_GET_ALL)
        self.autoChooser.addOption(AUTO_SIDE_UPPER, AUTO_SIDE_UPPER)
        self.autoChooser.addOption(AUTO_SIDE_LOWER, AUTO_SIDE_LOWER)
        self.autoChooser.addOption(AUTO_SHOOT_PRELOADED, AUTO_SHOOT_PRELOADED)
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

        self.odomField = wpilib.Field2d()
        wpilib.SmartDashboard.putData("odom", self.odomField)

    def robotPeriodic(self) -> None:
        profiler.start()

        self.time = TimeData(self.time)

        self.hal.publish(self.table)
        self.shooterStateMachine.publishInfo()

        self.drive.updateOdometry(self.hal)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)
        self.odomField.setRobotPose(pose)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putNumber("ctrl/absOffset", self.driveGyroYawOffset)
        self.table.putNumber("ctrl/driveX", self.input.driveX)
        self.table.putNumber("ctrl/driveY", self.input.driveY)

        self.table.putNumber("timesinceinit", self.time.timeSinceInit)

        profiler.end("robotPeriodic")



    def teleopInit(self) -> None:
        self.shooterStateMachine.state = 0
        self.manualAimPID = PIDControllerForArm(0, 0, 0, 0, 0.04, 0)
        self.manualShooterPID = PIDController(0, 0, 0, 0.2)
        self.PIDspeedSetpoint = 0


    def teleopPeriodic(self) -> None:
        frameStart = wpilib.getTime()
        self.input.update()
        self.hal.stopMotors()

        if self.input.gyroReset:
            self.driveGyroYawOffset = self.hal.yaw

        if self.input.absToggle:
            self.abs = not self.abs


        profiler.start()
        speedControlEdited = lerp(1, 5.0, self.input.speedCtrl)
        turnScalar = 3.6
        driveVector = Translation2d(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited)
        if self.abs:
            driveVector = driveVector.rotateBy(Rotation2d(-self.hal.yaw + self.driveGyroYawOffset))
        speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), -self.input.turning * turnScalar)
        self.drive.update(self.time.dt, self.hal, speed)
        profiler.end("drive updates")

        self.table.putNumber("POV", self.input.armCtrlr.getPOV())

        profiler.start()

        if(not self.input.overideIntakeStateMachine):
            self.intakeStateMachine.update(self.hal, self.input.intake)
        else:
            if(self.input.intake):
                self.hal.intakeSpeeds = [0.4, 0.4]
            if(self.input.intakeReverse):
                self.hal.intakeSpeeds = [-0.4, -0.4]
            self.intakeStateMachine.state = 0

        profiler.end("intake state machine")

        profiler.start()

        if(self.input.aimEncoderReset):
            self.hardware.resetAimEncoderPos(0)

        if(self.input.camEncoderReset):
            self.hardware.resetCamEncoderPos(0)

        if(not self.input.overideShooterStateMachine):
            self.shooterStateMachine.aim(self.input.aim)
            self.shooterStateMachine.rev(self.input.rev)
            self.shooterStateMachine.shoot(self.input.shoot)
            self.shooterStateMachine.update(self.hal, self.time.timeSinceInit, self.time.dt)
        else:
            self.shooterStateMachine.state = 0
            self.hal.shooterAimSpeed = self.manualAimPID.tick(0, self.hal.shooterAimPos, self.time.dt)

            if(self.input.manualAimJoystickY > 0.2):
                self.hal.shooterAimSpeed += -0.2
            if(self.input.manualAimJoystickY < -0.2):
                self.hal.shooterAimSpeed += 0.2

            if(self.input.manualFeedMotor):
                self.hal.intakeSpeeds[1] += 0.4
                self.hal.shooterIntakeSpeed += 0.4
            if(self.input.manualFeedReverseMotor):
                self.hal.intakeSpeeds[1] -= 0.4
                self.hal.shooterIntakeSpeed -= 0.4

            # TODO: this is moving to fast
            speedTarget = 0
            if(self.input.rev):
                speedTarget = 100
            self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * 0.1 + self.PIDspeedSetpoint
            self.hal.shooterSpeed = self.manualShooterPID.tick(self.PIDspeedSetpoint, self.hal.shooterAngVelocityMeasured, self.time.dt)
            if(self.input.shoot):
                self.hal.shooterIntakeSpeed = 0.4

        self.table.putBoolean("ShooterStateMachineOveride", self.input.overideShooterStateMachine)
        self.table.putBoolean("IntakeStateMachineOveride", self.input.overideIntakeStateMachine)
        self.table.putNumber("LeftStickY", self.input.manualAimJoystickY)

        profiler.end("shooter state machine")

        self.hal.camSpeed = self.input.camTemp * 0.2

        profiler.start()
        self.hardware.update(self.hal, self.time)
        profiler.end("hardware update")
        self.table.putNumber("frame time", wpilib.getTime() - frameStart)

    # NOTE: filename is *just* the title of the file, with no extension and no path
    # filename is directly passed to pathplanner.loadPath
    def loadTrajectory(self, fileName: str, flipped: bool) -> PathPlannerTrajectory:
        p = PathPlannerPath.fromPathFile(fileName)
        if flipped:
            p = p.flipPath()
        t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
        return t

    def autonomousInit(self) -> None:
        # when simulating, initalize sim to have a preloaded ring
        if isinstance(self.hardware, RobotSimHAL):
            self.hardware.ringPos = 1
            self.hardware.ringTransitionStart = -1

        self.holonomicController = PPHolonomicDriveController(
            PIDConstants(1, 0, 0),
            PIDConstants(3, 0, 0),
            5.0,
            self.drive.modulePositions[0].distance(Translation2d()))

        flipToRed = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        if self.autoSideChooser.getSelected() == AUTO_SIDE_FMS:
            if NetworkTableInstance.getDefault().getTable("FMSInfo").getBoolean("IsRedAlliance", False):
               flipToRed = True
            else:
                flipToRed = False

        b = stages.StageBuilder()
        # shootRoutine = stages.StageBuilder() \
        #     .addShooterPrepStage(ShooterTarget.SUBWOOFER, True).setTimeout(4).addAbortLog("cancelled shooter prep because of timeout") \
        #     .addShooterFireStage()
        traj = self.loadTrajectory("middle", flipToRed)
        centerRing = stages.StageBuilder() \
            .addIntakeStage().triggerAlongPath(0.6, traj) \
            .addIntakeStage() \
            .addStageSet(stages.StageBuilder() \
                          .addPathStage(self.loadTrajectory("middleBack", flipToRed)) \
                          .addShooterPrepStage(ShooterTarget.SUBWOOFER, True)) \
            .addShooterFireStage()

        initialPose: Pose2d = Pose2d()

        if self.autoChooser.getSelected() == AUTO_NONE:
            pass

        elif self.autoChooser.getSelected() == AUTO_INTAKE_CENTER_RING:
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            b.addTelemetryStage(AUTO_INTAKE_CENTER_RING)
            b.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            b.addShooterFireStage()
            b.addStageBuiltStage(centerRing)

        elif self.autoChooser.getSelected() == AUTO_GET_ALL:
            traj = self.loadTrajectory("middle", flipToRed)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            b.addTelemetryStage(AUTO_GET_ALL)
            b.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            b.addShooterFireStage()
            b.addStageBuiltStage(centerRing)

            # UPPER RING
            b.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("upper", flipToRed))
            b.addIntakeStage()
            b.addStageSet(stages.StageBuilder() \
                          .addPathStage(self.loadTrajectory("upperBack", flipToRed)) \
                          .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            b.addShooterFireStage()

            # LOWER RING
            b.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("lower", flipToRed))
            b.addIntakeStage()
            b.addStageSet(stages.StageBuilder() \
                          .addPathStage(self.loadTrajectory("lowerBack", flipToRed)) \
                          .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            b.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_EXIT:
            traj = self.loadTrajectory("exit", flipToRed)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            b.addTelemetryStage(AUTO_EXIT)
            b.addPathStage(traj)

        elif self.autoChooser.getSelected() == AUTO_SHOOT_PRELOADED:
            initialPose = Pose2d()
            b.addTelemetryStage(AUTO_SHOOT_PRELOADED)
            b.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            b.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDE_UPPER:
            traj = self.loadTrajectory("side-upper", flipToRed)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            b.addTelemetryStage(AUTO_SIDE_UPPER)
            b.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            b.addShooterFireStage()
            b.addIntakeStage().triggerAlongPath(0.5, traj)
            b.addIntakeStage()
            b.addStageSet(stages.StageBuilder() \
                          .addPathStage(self.loadTrajectory("upperBack", flipToRed)) \
                          .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            b.addShooterFireStage

        elif self.autoChooser.getSelected() == AUTO_SIDE_LOWER:
            traj = self.loadTrajectory('side-lower', flipToRed)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            b.addTelemetryStage(AUTO_SIDE_LOWER)
            b.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            b.addShooterFireStage()
            b.addIntakeStage().triggerAlongPath(0.5, traj)
            b.addIntakeStage()
            b.addStageSet(stages.StageBuilder() \
                          .addPathStage(self.loadTrajectory('lowerBack', flipToRed)) \
                          .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            b.addShooterFireStage

        else:
            assert(False)
        self.auto = auto.Auto(self.time.timeSinceInit, b.firstStage)

        self.driveGyroYawOffset = initialPose.rotation().radians()
        self.hardware.resetGyroToAngle(initialPose.rotation().radians())
        self.hardware.update(self.hal, self.time)
        self.drive.resetOdometry(initialPose, self.hal)
        self.holonomicController.reset(initialPose, ChassisSpeeds())

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.auto.update(self)
        self.shooterStateMachine.update(self.hal, self.time.timeSinceInit, self.time.dt)
        self.hardware.update(self.hal, self.time)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)

if __name__ == "__main__":
    wpilib.run(Robot)

    # r = Robot()
    # r.robotInit()
    # r.autonomousInit()
    # while(True):
    #     r.robotPeriodic()
    #     r.autonomousPeriodic()
