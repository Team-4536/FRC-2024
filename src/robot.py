import math

import profiler
import robotHAL
import wpilib
from autos import AutoBuilder
from ntcore import NetworkTableInstance
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from PIDController import PIDController, PIDControllerForArm, updatePIDsInNT
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from swerveDrive import SwerveDrive
from timing import TimeData
from utils import CircularScalar, Scalar
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from lightControl import LightControl
from noteStateMachine import ShooterTarget, NoteStateMachine

class RobotInputs():
    TARGET_NONE = 0
    TARGET_LEFT = 1
    TARGET_RIGHT = 2
    TARGET_SUBWOOFER = 3
    TARGET_SOURCE = 4

    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.driveScalar = CircularScalar(0.06, 1)
        self.turningScalar = CircularScalar(0.1, 1)
        self.manualAimScalar = Scalar(deadZone=0.1)

        self.driveX: float = 0.0
        self.driveY: float = 0.0
        self.turningX: float = 0.0
        self.turningY: float = 0.0
        self.speedCtrl: float = 0.0
        self.gyroReset: bool = False
        self.brakeButton: bool = False
        self.absToggle: bool = False

        self.turningStickButton: bool = False

        self.angleTarget: int = 0
        self.intake: bool = False

        self.aim: ShooterTarget = ShooterTarget.NONE
        self.rev: bool = False
        self.shoot: bool = False

        self.feed: bool = False

        self.camTemp: float = 0.0

        self.overideNoteStateMachine: bool = False

        self.overrideWasPressed: bool = False # NOTE: (Rob, 3/23/24) this is done completely as a hack, trying the WPI getYButtonPressed was not working as intended (even though other ones weres). 

        self.shooterAimManual: float = 0
        self.aimEncoderReset: bool = False
        self.manualFeed: bool = False
        self.manualFeedReverse: bool = False


        self.climb: float = 0.0 # - is trigger in, + is reverse pressed, range goes -1 to 1
        self.climbEncoderReset: bool = False

        self.lineUpWithSubwoofer: bool = False

    def update(self) -> None:
        ##flipped x and y inputs so they are relative to bot
        self.driveX, self.driveY = self.driveScalar.Scale(-self.driveCtrlr.getLeftY(), -self.driveCtrlr.getLeftX())
        self.turningX, self.turningY = self.turningScalar.Scale(self.driveCtrlr.getRightX(), -self.driveCtrlr.getRightY())

        self.turningPIDButton = self.driveCtrlr.getLeftBumper()
        self.turningStickButton = self.driveCtrlr.getRightStickButton()

        self.speedCtrl = self.driveCtrlr.getRightTriggerAxis()

        self.gyroReset = self.driveCtrlr.getStartButtonPressed()
        self.absToggle = self.driveCtrlr.getBackButtonPressed()

        self.angleTarget = self.TARGET_NONE
        if self.driveCtrlr.getAButton(): #down
            self.angleTarget = self.TARGET_SUBWOOFER
        elif self.driveCtrlr.getBButton(): #right
            self.angleTarget = self.TARGET_RIGHT
        elif self.driveCtrlr.getYButton(): #up
            self.angleTarget = self.TARGET_SOURCE
        elif self.driveCtrlr.getXButton(): #left
            self.angleTarget = self.TARGET_LEFT

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
        self.feed = self.intake


        self.climb = float(self.armCtrlr.getRightBumper()) - self.armCtrlr.getRightTriggerAxis()
        self.climbEncoderReset = self.armCtrlr.getYButtonPressed()

        # manual mode controls
        if(self.armCtrlr.getYButton() and not self.overrideWasPressed):
            self.overideNoteStateMachine = not self.overideNoteStateMachine
        self.overrideWasPressed = self.armCtrlr.getYButton()

        self.shooterAimManual = self.manualAimScalar(-self.armCtrlr.getLeftY())
        self.intakeReverse = self.armCtrlr.getBButton()
        self.manualFeed = self.intake
        self.manualFeedReverse = self.intakeReverse
        self.aimEncoderReset = self.armCtrlr.getLeftStickButtonPressed()
        self.camEncoderReset = self.armCtrlr.getRightStickButtonPressed()

        self.lineUpWithSubwoofer = self.driveCtrlr.getLeftTriggerAxis() > 0.3

AUTO_SIDE_RED = "red"
AUTO_SIDE_BLUE = "blue"
AUTO_SIDE_FMS = "FMS side"

AUTO_NONE = "none"
AUTO_INTAKE_CENTER_RING = "grab center ring"
AUTO_EXIT = "exit"
AUTO_GET_ALL = "three piece chicken nugget happy meal"
AUTO_GET_ALL_PODIUM = 'get all, podium first'
AUTO_SIDE_UPPER = 'go from speaker side to upper ring'
AUTO_SIDE_LOWER = 'go from side of speaker and get lower ring'
AUTO_FAR_MIDDLE = 'go from subwoofer to far middle ring'
AUTO_SHOOT_PRELOADED = 'shoot preloaded ring'
AUTO_SIDEUPPER_V02 = 'Side uper routine version 2'
AUTO_SIDEUPPER_3PC = 'no podium 3 pc chicken McNugget'

#Pipeline definitions
ODOMETRY_RESET_PIPELINE = 0
SUBWOOFER_LINEUP_RED_PIPLINE = 1
SUBWOOFER_LINEUP_BLUE_PIPLINE = 2

# Light animations, unused because they ovveride manual controls of lights
# strobeAnim  = StrobeAnimation(255, 255, 255, 0, 3, 200, 8)
# rainbowAnim = RainbowAnimation(1, .3, 200, False, 8)
# offAnim = FireAnimation(0, 0, 200, 0, 0, False, 8)
# colorFlowAnim = ColorFlowAnimation(255, 0, 255, 0, .2, 54)


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        
        self.limelightLeashOn = True
        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()
        self.lights = LightControl()
        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.input = RobotInputs()

        wheelPositions = [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)]
        self.drive = SwerveDrive(Rotation2d(self.hal.yaw), Pose2d(), wheelPositions)

        self.abs = True
        self.driveGyroYawOffset = 0.0 # the last angle that drivers reset the field oriented drive to zero at

        self.noteStateMachine: NoteStateMachine = NoteStateMachine()

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
        self.autoChooser.addOption(AUTO_FAR_MIDDLE, AUTO_FAR_MIDDLE)
        self.autoChooser.addOption(AUTO_SIDEUPPER_V02, AUTO_SIDEUPPER_V02)
        self.autoChooser.addOption(AUTO_SIDEUPPER_3PC, AUTO_SIDEUPPER_3PC)
        self.autoChooser.addOption(AUTO_GET_ALL_PODIUM, AUTO_GET_ALL_PODIUM)
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)
        
        self.odomField = wpilib.Field2d()
        wpilib.SmartDashboard.putData("odom", self.odomField)

        #kp can be 4 if wanted
        self.turnPID = PIDController("turnPID", 3, 0, 0)
        self.ang = 0

        self.frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front")
        self.robotPoseTable = NetworkTableInstance.getDefault().getTable("robot pose")

        self.subwooferLineupPID = PIDController("Subwoofer Lineup PID", 8, 0, 0, 0)


        self.table.putNumber("ctrl/SWERVE ADDED X", 0.0)
        self.table.putNumber("ctrl/SWERVE ADDED Y", 0.0)
        self.table.putNumber("ctrl/SWERVE ADDED R", 0.0)
        self.table.putNumber("ctrl/SWERVE ADDED DRIVE", 0)
        self.table.putNumber("ctrl/SWERVE ADDED STEER", 0)
        self.table.putNumber("leashedSpeedX", 0)
        self.table.putNumber("leashedSpeedY", 0)
        self.table.putNumber("leashedSpeedTurn", 0)
        

    def robotPeriodic(self) -> None:
        profiler.start()

        self.time = TimeData(self.time)

        self.hal.publish(self.table)
        self.noteStateMachine.publishInfo()

        self.drive.updateOdometry(self.hal)
        
        self.pieceX = self.frontLimelightTable.getNumber("tx", 0)
        self.pieceY = self.frontLimelightTable.getNumber("ty",0)
        pose = self.drive.odometry.getPose()
        self.table.putNumber("odomX", pose.x )
        self.table.putNumber("odomY", pose.y)
        self.odomField.setRobotPose(pose)

        self.table.putBoolean("ctrl/absOn", self.abs)
        self.table.putNumber("ctrl/absOffset", self.driveGyroYawOffset)
        self.table.putNumber("ctrl/driveX", self.input.driveX)
        self.table.putNumber("ctrl/driveY", self.input.driveY)
        self.table.putBoolean("ctrl/noteStateMachineOveride", self.input.overideNoteStateMachine)
        self.table.putNumber("drive pov", self.input.driveCtrlr.getPOV())
        
        self.onRedSide: bool = self.autoSideChooser.getSelected() == AUTO_SIDE_RED
        if self.autoSideChooser.getSelected() == AUTO_SIDE_FMS:
            if NetworkTableInstance.getDefault().getTable("FMSInfo").getBoolean("IsRedAlliance", False):
                self.onRedSide = True
            else:
                self.onRedSide = False
        
        if self.input.absToggle:
            self.abs = not self.abs

        self.lights.updateLED(self.table, self.time, self.hal, self.hardware)
        
        updatePIDsInNT()
        self.table.putNumber("Offset yaw", -self.hal.yaw + self.driveGyroYawOffset)
        profiler.end("robotPeriodic")

    def teleopInit(self) -> None:
        self.noteStateMachine.state = self.noteStateMachine.START
        self.manualAimPID = PIDControllerForArm("ManualAim", 0, 0, 0, 0, 0.04, 0)
        self.manualShooterPID = PIDController("ManualShoot", 0, 0, 0, 0.2)
        self.PIDspeedSetpoint = 0
        self.PIDtoggle = False
        self.rightStickToggle = False

        #TODO make the pipelines an Enum
        #red side
        self.subwooferLineupPipeline: int = 1
        if(not self.onRedSide):
            #blue side
            self.subwooferLineupPipeline = 2

    def teleopPeriodic(self) -> None:
        frameStart = wpilib.getTime()
        self.input.update()
        self.hal.stopMotors()
        
          

        if self.input.gyroReset:
            self.driveGyroYawOffset = self.hal.yaw

        profiler.start()
        speedControlEdited = lerp(1, 5.0, self.input.speedCtrl)
        turnScalar = 6
        driveVector = Translation2d(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited)
        turnVector = Translation2d(self.input.turningY, self.input.turningX) #for pid only
        #absolute drive
        if self.abs:
            driveVector = driveVector.rotateBy(Rotation2d(-self.hal.yaw + self.driveGyroYawOffset))

        #disable pid when stick moved
        if (self.input.turningX != 0 and self.rightStickToggle == False) or self.input.lineUpWithSubwoofer:
            self.PIDtoggle = False

        if self.input.turningX == 0:
            self.rightStickToggle = False

        #turn stick to dpad (kind of)
        if self.input.turningStickButton:
            self.PIDtoggle = True
            self.rightStickToggle = True
            if turnVector.angle().degrees() >= -45 and turnVector.angle().degrees() < 45:
                if self.onRedSide:
                    self.ang = math.radians(60)
                else:
                    self.ang = math.radians(-60)
            elif turnVector.angle().degrees() >= 45 and turnVector.angle().degrees() < 135:
                self.ang = math.radians(90)
            elif turnVector.angle().degrees() >= 135 or turnVector.angle().degrees() < -135:
                self.ang = math.radians(0)
            elif turnVector.angle().degrees() >= -135 and turnVector.angle().degrees() < -45:
                self.ang = math.radians(-90)
        self.table.putNumber("ctrl/turnVectorAngle", turnVector.angle().degrees())

        #assign angle based on button
        if self.input.angleTarget == RobotInputs.TARGET_LEFT:
            self.ang = math.radians(-90)
            self.PIDtoggle = True
        elif self.input.angleTarget == RobotInputs.TARGET_RIGHT:
            self.ang = math.radians(90)
            self.PIDtoggle = True
        elif self.input.angleTarget == RobotInputs.TARGET_SOURCE:
            if self.onRedSide:
                self.ang = math.radians(60)
            else:
                self.ang = math.radians(-60)
            self.PIDtoggle = True
        elif self.input.angleTarget == RobotInputs.TARGET_SUBWOOFER:
            self.ang = 0
            self.PIDtoggle = True

        #assign turning speed based on pid
        if self.PIDtoggle:
            speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), self.turnPID.tickErr(angleWrap(self.ang + (-self.hal.yaw + self.driveGyroYawOffset)), self.ang, self.time.dt))
        #limelight lineup
        elif self.input.lineUpWithSubwoofer:
            if(self.frontLimelightTable.getNumber("getpipe", 0) != self.subwooferLineupPipeline):
                self.frontLimelightTable.putNumber("pipeline", self.subwooferLineupPipeline)
            tx = self.frontLimelightTable.getNumber("tx", 0)
            ty = self.frontLimelightTable.getNumber('ty', 0)

            #speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), self.turnPID.tickErr(angleWrap(-math.radians(tx) + 0), 0, self.time.dt))
            speed = ChassisSpeeds(self.subwooferLineupPID.tickErr(math.radians(ty) + 0, 0, self.time.dt), \
                    driveVector.Y(), \
                    self.turnPID.tickErr(angleWrap(-math.radians(tx) + 0), 0, self.time.dt))
        else:
            #set chassis speed with no abs
            speed = ChassisSpeeds(driveVector.X(), driveVector.Y(), -self.input.turningX * turnScalar)

        self.table.putBoolean("ctrl/anglePIDToggle", self.PIDtoggle)
        self.table.putNumber("ctrl/targetAngle", math.degrees(self.ang))

        time = self.table.getNumber("ctrl/SWERVE TEST TIME", 0.0)
        time -= self.time.dt
        self.table.putNumber("ctrl/SWERVE TEST TIME", time)
        if time > 0:
            s = Translation2d(self.table.getNumber("ctrl/SWERVE ADDED X", 0.0), self.table.getNumber("ctrl/SWERVE ADDED Y", 0.0))
            s = s.rotateBy(Rotation2d((-self.hal.yaw + self.driveGyroYawOffset)))
            speed.vx += s.X()
            speed.vy += s.Y()
            speed.omega += self.table.getNumber("ctrl/SWERVE ADDED R", 0.0)
            # for i in range(4):
            #     self.hal.driveVolts[i] = self.table.getNumber("ctrl/SWERVE ADDED DRIVE", 0)
            #     self.hal.steeringVolts[i] = self.table.getNumber("ctrl/SWERVE ADDED STEER", 0)


        # self.drive.update(self.time.dt, self.hal, speed)
        if (self.pieceX == 0.00 and self.pieceY == 0.00):
            pass
        else:
            if self.pieceX > -12 and self.pieceX < 12:
                speed = ChassisSpeeds(-0.4, 0, 0)
            elif (self.pieceX < -12):
                self.leashError: float = self.pieceX - -12
                self.leashError = self.leashError * -1
                self.leashError = self.leashError * 0.1
                if self.leashError < 0.3:
                    self.leashError = 0.3
                speed = ChassisSpeeds(0,0, self.leashError)
            elif (self.pieceX > 12):
                self.leashError: float = self.pieceX - -1
                self.leashError = self.leashError * 0.1
                if self.leashError < 0.3:
                    self.leashError = 0.3
                speed = ChassisSpeeds(0,0, self.leashError)
        
        self.table.putNumber("leashedSpeedX", speed.vx)
        self.table.putNumber("leashedSpeedY", speed.vy)
        self.table.putNumber("leashedSpeedTurn", speed.omega)
        self.drive.update(self.time.dt, self.hal, speed)
        profiler.end("drive updates")


        self.table.putNumber("POV", self.input.armCtrlr.getPOV())

        profiler.start()

        if not self.input.overideNoteStateMachine:
            self.noteStateMachine.intake(self.input.intake)
            self.noteStateMachine.feed(self.input.feed) #untested
            self.noteStateMachine.aim(self.input.aim)
            self.noteStateMachine.rev(self.input.rev)
            self.noteStateMachine.shoot(self.input.shoot)
            self.noteStateMachine.update(self.hal, self.time.timeSinceInit, self.time.dt)
        else:
            self.noteStateMachine.state = self.noteStateMachine.START
            #overides for intaking
            if(self.input.intake):
                self.hal.intakeSpeeds = [0.4, 0.4]
            if(self.input.intakeReverse):
                self.hal.intakeSpeeds = [-0.4, -0.4]

            #overides for lower shooter motor and upper intake
            self.hal.shooterAimSpeed = self.manualAimPID.tick(0, self.hal.shooterAimPos, self.time.dt)
            self.hal.shooterAimSpeed += self.input.shooterAimManual * 0.2

            if(self.input.manualFeed):
                self.hal.intakeSpeeds[1] += 0.4
                self.hal.shooterIntakeSpeed += 0.4
            if(self.input.manualFeedReverse):
                self.hal.intakeSpeeds[1] -= 0.4
                self.hal.shooterIntakeSpeed -= 0.4

            #overid for shooting
            # TODO: this is moving to fast
            speedTarget = 0
            if(self.input.rev):
                speedTarget = 100
            self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * 0.1 + self.PIDspeedSetpoint
            self.hal.shooterSpeed = self.manualShooterPID.tick(self.PIDspeedSetpoint, self.hal.shooterAngVelocityMeasured, self.time.dt)
            if(self.input.shoot):
                self.hal.shooterIntakeSpeed = 0   

        # TODO: manual cam drive
        # camTarget = self.shooterStateMachine.table.getNumber('targetCam', 0)
        # self.shooterStateMachine.camPID.kp = self.shooterStateMachine.table.getNumber("cam kp", 0)
        # self.hal.camSpeed = self.shooterStateMachine.camPID.tick(camTarget, self.hal.camPos, self.time.dt)
    

        if(self.input.aimEncoderReset):
            self.hardware.resetAimEncoderPos(0)

        if(self.input.camEncoderReset):
            self.hardware.resetCamEncoderPos(0)

        profiler.end("note state machine")

        # self.hal.camSpeed = self.input.camTemp * 0.2
        self.hal.climberSpeed = self.input.climb * 0.6


        
    
        
    
        #speed = ChassisSpeeds(0, 0, 0)
        
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
            PIDConstants(self.turnPID.kp, self.turnPID.ki, self.turnPID.kd,),
            5.0,
            self.drive.modulePositions[0].distance(Translation2d()))


        self.auto = AutoBuilder()
        # shootRoutine = stages.StageBuilder() \
        #     .addShooterPrepStage(ShooterTarget.SUBWOOFER, True).setTimeout(4).addAbortLog("cancelled shooter prep because of timeout") \
        #     .addShooterFireStage()
        traj = self.loadTrajectory("middle", self.onRedSide)
        centerRing = AutoBuilder() \
            .addIntakeStage().triggerAlongPath(0.6, traj) \
            .addIntakeStage() \
            .addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("middleBack", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True)) \
            .addShooterFireStage()


        initialPose: Pose2d = Pose2d()

        if self.autoChooser.getSelected() == AUTO_NONE:
            pass

        elif self.autoChooser.getSelected() == AUTO_INTAKE_CENTER_RING:
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_INTAKE_CENTER_RING)
            self.table.putNumber("stage#", 1)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.table.putNumber("stage#", 2)
            self.auto.addShooterFireStage()
            self.table.putNumber("stage#", 3)
            self.auto.addSequence(centerRing)
            self.table.putNumber("stage#", 4)

        elif self.autoChooser.getSelected() == AUTO_GET_ALL:
            traj = self.loadTrajectory("middle", self.onRedSide)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_GET_ALL)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addSequence(centerRing)

            self.auto.addOdometryResetWithLimelightStage(self, ODOMETRY_RESET_PIPELINE)

            # UPPER RING

            self.auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("upper", self.onRedSide))
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("upperBack", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

            # LOWER RING
            self.auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("lower", self.onRedSide))
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("lowerBack", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_GET_ALL_PODIUM:
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            traj = self.loadTrajectory("lower", self.onRedSide)
            self.auto.addIntakeStage().triggerAlongPath(0.6, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("lowerBack", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

            traj = self.loadTrajectory("middle", self.onRedSide)
            
            self.auto.addTelemetryStage(AUTO_GET_ALL)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addSequence(centerRing)

            self.auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("upper", self.onRedSide))
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("upperBack", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_EXIT:
            traj = self.loadTrajectory("exit", self.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_EXIT)
            self.auto.addPathStage(traj)

        elif self.autoChooser.getSelected() == AUTO_SHOOT_PRELOADED:
            initialPose = Pose2d()
            self.auto.addTelemetryStage(AUTO_SHOOT_PRELOADED)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_FAR_MIDDLE:
            traj = self.loadTrajectory("far-middle", self.onRedSide)
            
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_FAR_MIDDLE)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addIntakeStage().triggerAlongPath(0.7, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("far-middle-back", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDE_UPPER:
            traj = self.loadTrajectory("side-upper", self.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_SIDE_UPPER)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("side-upper-back", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDEUPPER_V02:
            traj = self.loadTrajectory("side-upper-v02", self.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_SIDE_UPPER)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("side-upper-back-v02", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDEUPPER_3PC:
            traj = self.loadTrajectory("side-upper-v02", self.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_SIDE_UPPER)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("side-upper-back-v02", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()
            traj = self.loadTrajectory("sideFar-upper-v02", self.onRedSide)
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("sideFar-upper-back-v02", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()



        elif self.autoChooser.getSelected() == AUTO_SIDEUPPER_3PC:
            traj = self.loadTrajectory("side-upper-v02", self.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_SIDE_UPPER)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("side-upper-back-v02", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()
            traj = self.loadTrajectory("sideFar-upper-v02", self.onRedSide)
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("sideFar-upper-back-v02", self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()


        elif self.autoChooser.getSelected() == AUTO_SIDE_LOWER:
            traj = self.loadTrajectory('side-lower', self.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            self.auto.addTelemetryStage(AUTO_SIDE_LOWER)
            self.auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            self.auto.addShooterFireStage()
            self.auto.addIntakeStage().triggerAlongPath(0.5, traj)
            self.auto.addIntakeStage()
            self.auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory('side-lower-back', self.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            self.auto.addShooterFireStage()

        else:
            assert(False)

        self.driveGyroYawOffset = initialPose.rotation().radians()
        self.hardware.resetGyroToAngle(initialPose.rotation().radians())
        self.hardware.update(self.hal, self.time)
        self.drive.resetOdometry(initialPose, self.hal)
        self.holonomicController.reset(initialPose, ChassisSpeeds())

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.auto.run(self)
        self.noteStateMachine.update(self.hal, self.time.timeSinceInit, self.time.dt)
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



