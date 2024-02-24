import math

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
from real import angleWrap, lerp
from shooterStateMachine import ShooterTarget, StateMachine
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import CircularScalar, Scalar
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition


class RobotInputs():
    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.driveScalar = CircularScalar(.05, 1)
        self.rotScalar = Scalar(deadZone = .1, exponent = 1)

        self.driveX: float = 0.0
        self.driveY: float = 0.0
        self.turning: float = 0.0
        self.speedCtrl: float = 0.0
        self.gyroReset: bool = False
        self.brakeButton: bool = False
        self.absToggle: bool = False

        self.turningPIDButton: bool = False
        self.targetAngle: float = 0.0

        self.intake: bool = False

        self.aim: ShooterTarget = ShooterTarget.NONE
        self.rev: bool = False
        self.shoot: bool = False

        self.camTemp: float = 0.0

        self.overideShooterStateMachine: bool = False
        self.overideIntakeStateMachine: bool = False

        self.shooterAimManual: float = 0
        self.aimEncoderReset: bool = False
        self.manualFeed: bool = False
        self.manualFeedReverse: bool = False

        self.climb: float = 0.0 # + is trigger in, - is reverse pressed, range goes -1 to 1

    def update(self) -> None:
        ##flipped x and y inputs so they are relative to bot
        self.driveX, self.driveY = self.driveScalar.Scale(-self.driveCtrlr.getLeftX(), -self.driveCtrlr.getLeftY())
        self.turning = self.rotScalar(self.driveCtrlr.getRightX())
       
        self.turningPIDButton = self.driveCtrlr.getLeftBumper()


        self.speedCtrl = self.driveCtrlr.getRightTriggerAxis()

        self.gyroReset = self.driveCtrlr.getYButtonPressed()
        self.brakeButton = self.driveCtrlr.getBButtonPressed()
        self.absToggle = self.driveCtrlr.getXButtonPressed()

    
        if self.driveCtrlr.getPOV() < 190 and self.driveCtrlr.getPOV() > 170: #down
            # if NetworkTableInstance.getDefault().getTable("FMSInfo").getBoolean("isBlueAlliance", False):
               # self.targetAngle = math.radians(90)
            # else:
            self.targetAngle = math.radians(0)
        elif self.driveCtrlr.getPOV() > 80  and self.driveCtrlr.getPOV() < 100: #right
            self.targetAngle = math.radians(-90)
        elif (self.driveCtrlr.getPOV() < 10 and self.driveCtrlr.getPOV() > -0.9) or self.driveCtrlr.getPOV() > 350: #up
            self.targetAngle = math.radians(60)
        elif self.driveCtrlr.getPOV() > 260 and self.driveCtrlr.getPOV() < 280: #left
            self.targetAngle = math.radians(90)
        
        """        #angle snapping with ABXY
        if self.driveCtrlr.getAButton():    #AMP SNAP
            if NetworkTableInstance.getDefault().getTable("FMSInfo").getBoolean("isBlueAlliance", False): 
                self.targetAngle = math.radians(90)
            else:
                self.targetAngle = math.radians(-90)
        
        elif self.driveCtrlr.getBButton(): #SOURCE SNAP
            if NetworkTableInstance.getDefault().getTable("FMSInfo").getBoolean("isBlueAlliance", False): 
                self.targetAngle = math.radians(60)
            else:
                self.targetAngle = math.radians(-60)
        
        elif self.driveCtrlr.getYButton: #snap to face driver station
                self.targetAngle = 0"""
        
        # arm controller
        self.intake = self.armCtrlr.getAButton()

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

        self.climb = self.armCtrlr.getRightTriggerAxis() - float(self.armCtrlr.getRightBumper())
        # manual mode controls

        if(self.armCtrlr.getYButtonPressed()):
            self.overideShooterStateMachine = not self.overideShooterStateMachine
            self.overideIntakeStateMachine = self.overideShooterStateMachine

        self.shooterAimManual = -self.armCtrlr.getLeftY()
        self.intakeReverse = self.armCtrlr.getBButton()
        self.manualFeed = self.intake
        self.manualFeedReverse = self.intakeReverse
        self.aimEncoderReset = self.armCtrlr.getLeftStickButtonPressed()
        self.camEncoderReset = self.armCtrlr.getRightStickButtonPressed()

AUTO_SIDE_RED = "red"
AUTO_SIDE_BLUE = "blue"
AUTO_SIDE_FMS = "FMS side"

AUTO_NONE = "none"
AUTO_INTAKE_CENTER_RING = "grab center ring"
AUTO_EXIT = "exit"
AUTO_GET_ALL = "grab all"

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
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

        self.turnPID = PIDController(2.4, 0, 0)
        self.turnPID.kp = 2.4
        self.table.putNumber("turnPID kp", self.turnPID.kp)
    def robotPeriodic(self) -> None:
        profiler.start()

        self.time = TimeData(self.time)

        self.hal.publish(self.table)
        self.shooterStateMachine.publishInfo()

        self.drive.updateOdometry(self.hal)

        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putNumber("ctrl/absOffset", self.driveGyroYawOffset)
        self.table.putNumber("ctrl/driveX", self.input.driveX)
        self.table.putNumber("ctrl/driveY", self.input.driveY)
        self.table.putBoolean("ctrl/manualMode", self.input.overideIntakeStateMachine)

        self.table.putNumber("target angle", math.degrees(self.input.targetAngle))

        self.turnPID.kp = self.table.getNumber("turnPID kp", 0.3)

        self.table.putNumber("drive pov", self.input.driveCtrlr.getPOV())

        profiler.end("robotPeriodic")

    def teleopInit(self) -> None:
        self.shooterStateMachine.state = 0
        self.manualAimPID = PIDControllerForArm(0, 0, 0, 0, 0.04, 0)
        self.manualShooterPID = PIDController(0, 0, 0, 0.2)
        self.PIDspeedSetpoint = 0


        self.turnPID = PIDController(0.3, 0, 0)

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


        if self.input.turningPIDButton:
            speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), self.turnPID.tickErr(angleWrap(-self.input.targetAngle + (-self.hal.yaw + self.driveGyroYawOffset)), self.input.targetAngle, self.time.dt))
        else:
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
            self.hardware.shooterAimEncoder.setPosition(0)

        if(self.input.camEncoderReset):
            self.hardware.camEncoder.setPosition(0)

        if(not self.input.overideShooterStateMachine):
            self.shooterStateMachine.aim(self.input.aim)
            self.shooterStateMachine.rev(self.input.rev)
            self.shooterStateMachine.shoot(self.input.shoot)
            self.shooterStateMachine.update(self.hal, self.time.timeSinceInit, self.time.dt)
        else:
            self.shooterStateMachine.state = 0
            self.hal.shooterAimSpeed = self.manualAimPID.tick(0, self.hal.shooterAimPos, self.time.dt)

            if(self.input.shooterAimManual > 0.2):
                self.hal.shooterAimSpeed += -0.1
            if(self.input.shooterAimManual < -0.2):
                self.hal.shooterAimSpeed += 0.1

            if(self.input.manualFeed):
                self.hal.intakeSpeeds[1] += 0.4
                self.hal.shooterIntakeSpeed += 0.4
            if(self.input.manualFeedReverse):
                self.hal.intakeSpeeds[1] -= 0.4
                self.hal.shooterIntakeSpeed -= 0.4

            # TODO: this is moving to fast
            speedTarget = 0
            if(self.input.rev):
                speedTarget = 100
            self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * 0.1 + self.PIDspeedSetpoint
            self.hal.shooterSpeed = self.manualShooterPID.tick(self.PIDspeedSetpoint, self.hal.shooterAngVelocityMeasured, self.time.dt)
            if(self.input.shoot):
                self.hal.shooterIntakeSpeed = 0.

            # TODO: manual cam drive
            # camTarget = self.shooterStateMachine.table.getNumber('targetCam', 0)
            # self.shooterStateMachine.camPID.kp = self.shooterStateMachine.table.getNumber("cam kp", 0)
            # self.hal.camSpeed = self.shooterStateMachine.camPID.tick(camTarget, self.hal.camPos, self.time.dt)

        self.table.putBoolean("ShooterStateMachineOveride", self.input.overideShooterStateMachine)
        self.table.putBoolean("IntakeStateMachineOveride", self.input.overideIntakeStateMachine)
        self.table.putNumber("ShooterAimManual", self.input.shooterAimManual)

        profiler.end("shooter state machine")

        # self.hal.camSpeed = self.input.camTemp * 0.2
        self.hal.climberSpeed = self.input.climb * 0.05

        profiler.start()
        self.hardware.update(self.hal)
        profiler.end("hardware update")
        self.table.putNumber("frame time", wpilib.getTime() - frameStart)

    def loadTrajectory(self, name: str, flipped: bool) -> PathPlannerTrajectory:
        p = PathPlannerPath.fromPathFile(name)
        if flipped:
            p = p.flipPath()
        t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
        return t

    def autonomousInit(self) -> None:
        self.holonomicController = PPHolonomicDriveController(
            PIDConstants(1, 0, 0),
            PIDConstants(self.turnPID.kp, self.turnPID.ki, self.turnPID.kd,),
            5.0,
            self.drive.modulePositions[0].distance(Translation2d()))

        flipToRed = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        if self.autoSideChooser.getSelected() == AUTO_SIDE_FMS:
            if NetworkTableInstance.getDefault().getTable("FMSInfo").getBoolean("IsRedAlliance", False):
               flipToRed = True
            else:
                flipToRed = False

        stageList: list[auto.Stage] = []
        initialPose: Pose2d = Pose2d()

        if self.autoChooser.getSelected() == AUTO_NONE:
            stageList = []
        elif self.autoChooser.getSelected() == AUTO_INTAKE_CENTER_RING:
            traj = self.loadTrajectory("middle", flipToRed)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            stageList = [
                stages.makeTelemetryStage(AUTO_INTAKE_CENTER_RING),
                stages.makeShooterPrepStage(ShooterTarget.SUBWOOFER, True),
                stages.makeShooterFireStage(),
                stages.makePathStageWithTriggerAtPercent(traj, 0.6, stages.makeIntakeStage()),
                stages.makeIntakeStage(),
                stages.makeStageSet([
                    stages.makePathStage(self.loadTrajectory("middleBack", flipToRed)),
                    stages.makeShooterPrepStage(ShooterTarget.SUBWOOFER, True),
                ]),
                stages.makeShooterFireStage()
            ]
        elif self.autoChooser.getSelected() == AUTO_GET_ALL:
            traj = self.loadTrajectory("middle", flipToRed)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            stageList = [
                stages.makeTelemetryStage(AUTO_GET_ALL),
                stages.makeShooterPrepStage(ShooterTarget.SUBWOOFER, True),
                stages.makeShooterFireStage(),

                # CENTER RING
                stages.makePathStageWithTriggerAtPercent(traj, 0.6, stages.makeIntakeStage()),
                stages.makeIntakeStage(),
                stages.makeStageSet([
                    stages.makePathStage(self.loadTrajectory("middleBack", flipToRed)),
                    stages.makeShooterPrepStage(ShooterTarget.SUBWOOFER, True),
                ]),
                stages.makeShooterFireStage(),

                # UPPER RING
                stages.makePathStageWithTriggerAtPercent(self.loadTrajectory("upper", flipToRed), 0.6, stages.makeIntakeStage()),
                stages.makeIntakeStage(),
                stages.makeStageSet([
                    stages.makePathStage(self.loadTrajectory("upperBack", flipToRed)),
                    stages.makeShooterPrepStage(ShooterTarget.SUBWOOFER, True),
                ]),
                stages.makeShooterFireStage(),

                # LOWER RING
                stages.makePathStageWithTriggerAtPercent(self.loadTrajectory("lower", flipToRed), 0.6, stages.makeIntakeStage()),
                stages.makeIntakeStage(),
                stages.makeStageSet([
                    stages.makePathStage(self.loadTrajectory("lowerBack", flipToRed)),
                    stages.makeShooterPrepStage(ShooterTarget.SUBWOOFER, True),
                ]),
                stages.makeShooterFireStage(),
            ]
        elif self.autoChooser.getSelected() == AUTO_EXIT:
            traj = self.loadTrajectory("exit", flipToRed)
            initialPose = traj.getInitialTargetHolonomicPose()
            stageList = [
                stages.makeTelemetryStage(AUTO_EXIT),
                stages.makePathStage(traj),
            ]
        else:
            assert(False)
        self.auto = auto.Auto(stageList, self.time.timeSinceInit)

        self.driveGyroYawOffset = initialPose.rotation().radians()
        self.hardware.gyro.reset()
        self.hardware.gyro.setAngleAdjustment(-initialPose.rotation().degrees())
        self.hardware.update(self.hal)
        self.drive.resetOdometry(initialPose, self.hal)

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.auto.update(self)
        self.shooterStateMachine.update(self.hal, self.time.timeSinceInit, self.time.dt)
        self.hardware.update(self.hal)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)

if __name__ == "__main__":
    wpilib.run(Robot)

    # r = Robot()
    # r.robotInit()
    # r.autonomousInit()
    # while(True):
    #     r.robotPeriodic()
    #     r.autonomousPeriodic()



