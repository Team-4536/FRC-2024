import robot
import wpilib
from autos import AutoBuilder, Stage
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from intakeStateMachine import IntakeStateMachine
from shooterStateMachine import StateMachine
from ntcore import NetworkTable




AUTO_NONE = "none"
AUTO_INTAKE_CENTER_RING = "grab center ring"
AUTO_EXIT = "exit"
AUTO_GET_ALL = "three piece chicken nugget happy meal"
AUTO_GET_ALL_PODIUM = 'get all, podium first'
AUTO_SIDE_UPPER = 'speaker side to upper ring'
AUTO_SIDE_LOWER = 'speaker side to lower ring'
AUTO_FAR_MIDDLE = 'speaker center to far middle ring'
AUTO_SHOOT_PRELOADED = 'shoot preloaded ring'
AUTO_SIDEUPPER_V02 = 'Side upper routine version 2'
AUTO_SIDEUPPER_3PC = 'side upper 3pc, no podium'
AUTO_TROLL = 'mess up middle rings'
AUTO_SYSTEM_CHECK = 'system check'

"""
A small class to remove the functionality of auto choosers and construction from robot.py.
This was done to prevent large conflicts, and make it easy to change chooser options without jumping up and down 300 lines of other code.
This class is only intended to be used once in robotInit in robot.py, not anywhere else.
Class construction publishes choosers to the dashboard
"""
#systemCheckDriveStage = NetworkTable.getInstance(self)
class RobotAutos():
    def __init__(self) -> None:
        self.systemCheckStopClimbing = False
        self.turn = 0
        
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
        self.autoChooser.addOption(AUTO_TROLL, AUTO_TROLL)
        self.autoChooser.addOption(AUTO_SYSTEM_CHECK, AUTO_SYSTEM_CHECK)
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

        self.systemCheckStage = 0
        self.systemCheckRunIntake = False
        self.systemCheckStopIntaking = False
        self.systemCheckStopClimbing = False

    # NOTE: filename is *just* the title of the file, with no extension and no path
    # filename is directly passed to pathplanner.loadPath
    def loadTrajectory(self, fileName: str, flipped: bool) -> PathPlannerTrajectory:
        p = PathPlannerPath.fromPathFile(fileName)
        if flipped:
            p = p.flipPath()
        t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
        return t

    # creates and returns the currently selected auto on the dashboard, along with the initial pose
    # call in autoInit in robot.py
    def autoInit(self, r: 'robot.Robot') -> tuple[AutoBuilder, Pose2d]:
        auto = AutoBuilder()
        initialPose: Pose2d = Pose2d()

        traj = self.loadTrajectory("middle", r.onRedSide)
        centerRing = AutoBuilder() \
            .addIntakeStage().triggerAlongPath(0.6, traj) \
            .addIntakeStage() \
            .addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("middleBack", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True)) \
            .addShooterFireStage()

        if self.autoChooser.getSelected() == AUTO_NONE:
            pass

        elif self.autoChooser.getSelected() == AUTO_SYSTEM_CHECK:
            #auto.add(Stage(self.systemCheckTest, "test"))
            auto.add(Stage(self.systemCheckDrive, "drive"))
            auto.add(Stage(self.systemCheckIntake, "intake"))
            auto.add(Stage(self.systemCheckAmp, "amp"))
            auto.add(Stage(self.systemCheckIntake, "intake"))
            auto.add(Stage(self.systemCheckShooter, "shooter"))
            auto.add(Stage(self.systemCheckClimbUp, "climb up"))
            auto.add(Stage(self.systemCheckClimbDown, "climb down"))


        elif self.autoChooser.getSelected() == AUTO_INTAKE_CENTER_RING:
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_INTAKE_CENTER_RING)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addSequence(centerRing)

        elif self.autoChooser.getSelected() == AUTO_TROLL:
            traj = self.loadTrajectory("troll", r.onRedSide)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_TROLL)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addPathStage(traj)

        elif self.autoChooser.getSelected() == AUTO_GET_ALL:
            traj = self.loadTrajectory("middle", r.onRedSide)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_GET_ALL)
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addSequence(centerRing)
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)

            # UPPER RING
            auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("upper", r.onRedSide))
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("upperBack", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)

            # LOWER RING
            auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("lower", r.onRedSide))
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("lowerBack", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)

        elif self.autoChooser.getSelected() == AUTO_GET_ALL_PODIUM:
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            traj = self.loadTrajectory("lower", r.onRedSide)
            auto.addIntakeStage().triggerAlongPath(0.6, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("lowerBack", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

            traj = self.loadTrajectory("middle", r.onRedSide)

            auto.addTelemetryStage(AUTO_GET_ALL)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addSequence(centerRing)

            auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("upper", r.onRedSide))
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("upperBack", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_EXIT:
            traj = self.loadTrajectory("exit", r.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_EXIT)
            auto.addPathStage(traj)

        elif self.autoChooser.getSelected() == AUTO_SHOOT_PRELOADED:
            initialPose = Pose2d()
            auto.addTelemetryStage(AUTO_SHOOT_PRELOADED)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_FAR_MIDDLE:
            traj = self.loadTrajectory("far-middle", r.onRedSide)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_FAR_MIDDLE)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addIntakeStage().triggerAlongPath(0.7, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("far-middle-back", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDE_UPPER:
            traj = self.loadTrajectory("side-upper", r.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_SIDE_UPPER)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("side-upper-back", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDEUPPER_V02:
            traj = self.loadTrajectory("side-upper-v02", r.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_SIDE_UPPER)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("side-upper-back-v02", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDEUPPER_3PC:
            traj = self.loadTrajectory("side-upper-v02", r.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_SIDE_UPPER)
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("side-upper-back-v02", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)
            traj = self.loadTrajectory("sideFar-upper-v02", r.onRedSide)
            auto.addPathStage(traj)
            #traj = self.loadTrajectory('sideFar-upper-v02B', r.onRedSide)
            #auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \

                        .addPathStage(self.loadTrajectory("sideFar-upper-back-v02", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()
            auto.addOdometryResetWithLimelightStage(r, robot.ODOMETRY_RESET_PIPELINE)

        elif self.autoChooser.getSelected() == AUTO_SIDE_LOWER:
            traj = self.loadTrajectory('side-lower', r.onRedSide)

            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_SIDE_LOWER)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory('side-lower-back', r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

        else:
            assert(False)

        return auto, initialPose


    def systemCheckTest(self,r: 'robot.Robot'):
        speed = ChassisSpeeds(0.5, 0, self.turn)
        r.drive.update(r.time.dt, r.hal, speed)

    def systemCheckDrive(self,r: 'robot.Robot'):
        if r.input.armCtrlr.getBButtonPressed():
            self.turn = 0
        if r.input.armCtrlr.getXButtonPressed():
            self.turn = 0.05
        if r.input.armCtrlr.getYButtonPressed():
            self.turn = -0.05
        speed = ChassisSpeeds(0.5, 0, self.turn)
        r.drive.update(r.time.dt, r.hal, speed)
        return r.input.armCtrlr.getAButtonPressed()
    def systemCheckIntake(self, r: 'robot.Robot') -> bool | None: 
        IntakeStateMachine.update(r.intakeStateMachine,r.hal, True)
        if r.intakeSensor == True:
                StateMachine.feed(r.shooterStateMachine, True)
        return r.input.armCtrlr.getAButtonPressed()
    def systemCheckAmp(self, r: 'robot.Robot') -> bool | None: 
        if r.hal.shooterSensor == True:
            StateMachine.rev(r.shooterStateMachine, True)
            StateMachine.aim(r.shooterStateMachine, ShooterTarget.AMP)
        if r.shooterStateMachine.onTarget == True and r.input.armCtrlr.getXButton and r.hal.shooterSensor == True:
            StateMachine.shoot(r.shooterStateMachine, True)
        if r.hal.shooterSensor == False:
            StateMachine.aim(r.shooterStateMachine, ShooterTarget.NONE)
        return r.input.armCtrlr.getAButtonPressed()
    def systemCheckShooter(self, r: 'robot.Robot') -> bool | None:
        if r.hal.shooterSensor == True:
            StateMachine.rev(r.shooterStateMachine, True)
            StateMachine.aim(r.shooterStateMachine, ShooterTarget.SUBWOOFER)
        if r.shooterStateMachine.onTarget == True and r.input.armCtrlr.getXButton and r.hal.shooterSensor == True:
            StateMachine.shoot(r.shooterStateMachine, True)
        return r.input.armCtrlr.getAButtonPressed()
    def systemCheckClimbUp(self, r: 'robot.Robot') -> bool | None: 
        if self.systemCheckStopClimbing == False:
            r.hal.climberSpeed = 0.1
        if not hasattr(self, "climbTimer"):
            self.climbTimer = r.time.timeSinceInit
        self.currentTime = r.time.timeSinceInit
        if self.currentTime - 0.9 > self.climbTimer:
            self.systemCheckStopClimbing = True
            r.hal.climberSpeed = 0
        return r.input.armCtrlr.getAButtonPressed()
    def systemCheckClimbDown(self, r: 'robot.Robot') -> bool | None:
        r.hal.climberSpeed = -0.1
        if r.hal.climberLimitPressed == True:
            r.hal.climberSpeed = 0
            self.systemCheckStage = 0
        













    # def systemCheck(self, r: 'robot.Robot') -> bool | None:
    #     if r.input.armCtrlr.getAButtonPressed():
    #         self.systemCheckStage += 1
    #     IntakeStateMachine.update(r.intakeStateMachine,r.hal, self.systemCheckRunIntake)
        
    #     if self.systemCheckStage == 4 and self.systemCheckStopIntaking == True or self.systemCheckStage == 5:
    #         self.systemCheckRunIntake = True
    #     else:
    #         self.systemCheckRunIntake = False   

    #     if self.systemCheckStage == 1:
    #         speed = ChassisSpeeds(0.05, 0, 0)
    #         r.drive.update(r.time.dt, r.hal, speed)
    #     elif self.systemCheckStage == 2:  
    #         speed = ChassisSpeeds(0.05, 0, -0.5)
    #         r.drive.update(r.time.dt, r.hal, speed)
    #     elif self.systemCheckStage == 3:
    #         speed = ChassisSpeeds(0.05, 0, 0.5)
    #         r.drive.update(r.time.dt, r.hal, speed)
    #     elif self.systemCheckStage == 4:
    #         if r.intakeSensor == True:
    #             self.systemCheckStopIntaking = False
    #             StateMachine.feed(r.shooterStateMachine, True)
    #         if r.hal.shooterSensor == True:
    #             StateMachine.rev(r.shooterStateMachine, True)
    #             StateMachine.aim(r.shooterStateMachine, ShooterTarget.AMP)
    #         if r.shooterStateMachine.onTarget == True and r.input.armCtrlr.getXButton and r.hal.shooterSensor == True:
    #             StateMachine.shoot(r.shooterStateMachine, True)
    #         if r.hal.shooterSensor == False:
    #             StateMachine.aim(r.shooterStateMachine, ShooterTarget.NONE)
    #             if not hasattr(self, "resetTimer"):
    #                 self.resetTimer = r.time.timeSinceInit
    #             self.currentTime = r.time.timeSinceInit
    #             if self.currentTime - 0.9 > self.resetTimer:
    #                 self.systemCheckStage +=1
    #     elif self.systemCheckStage == 5: 
    #         if r.intakeSensor == True:
    #             StateMachine.feed(r.shooterStateMachine, True)
    #         if r.hal.shooterSensor == True:
    #             StateMachine.rev(r.shooterStateMachine, True)
    #             StateMachine.aim(r.shooterStateMachine, ShooterTarget.SUBWOOFER)
    #         if r.shooterStateMachine.onTarget == True and r.input.armCtrlr.getXButton and r.hal.shooterSensor == True:
    #             StateMachine.shoot(r.shooterStateMachine, True)
    #         if r.hal.shooterSensor == False:
    #             self.systemCheckStage += 1
    #     elif self.systemCheckStage == 6:
    #         if self.systemCheckStopClimbing == True:
    #             r.hal.climberSpeed = 0.1
    #         if not hasattr(self, "climbTimer"):
    #             self.climbTimer = r.time.timeSinceInit
    #         self.currentTime = r.time.timeSinceInit
    #         if self.currentTime - 0.9 > self.climbTimer:
    #             self.systemCheckStopClimbing = True
    #             r.hal.climberSpeed = 0
    #     elif self.systemCheckStage == 7:
    #         r.hal.climberSpeed = -0.1
    #         if r.hal.climberLimitPressed == True:
    #             r.hal.climberSpeed = 0
    #             self.systemCheckStage = 0





                
        



