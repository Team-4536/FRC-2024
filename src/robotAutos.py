import wpilib
from autos import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from robot import Robot
from shooterStateMachine import ShooterTarget
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

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


class RobotAutos():
    def robotInit(self) -> None:
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

    # NOTE: filename is *just* the title of the file, with no extension and no path
    # filename is directly passed to pathplanner.loadPath
    def loadTrajectory(self, fileName: str, flipped: bool) -> PathPlannerTrajectory:
        p = PathPlannerPath.fromPathFile(fileName)
        if flipped:
            p = p.flipPath()
        t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
        return t

    def autoInit(self, r: 'Robot') -> tuple[AutoBuilder, Pose2d]:
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

        elif self.autoChooser.getSelected() == AUTO_INTAKE_CENTER_RING:
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_INTAKE_CENTER_RING)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addSequence(centerRing)

        elif self.autoChooser.getSelected() == AUTO_GET_ALL:
            traj = self.loadTrajectory("middle", r.onRedSide)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_GET_ALL)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addSequence(centerRing)
            auto.addOdometryResetWithLimelightStage(r, ODOMETRY_RESET_PIPELINE)

            # UPPER RING

            auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("upper", r.onRedSide))
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                   .addPathStage(self.loadTrajectory("upperBack", r.onRedSide)) \
                   .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

            # LOWER RING
            auto.addIntakeStage().triggerAlongPath(0.6, self.loadTrajectory("lower", r.onRedSide))
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                   .addPathStage(self.loadTrajectory("lowerBack", r.onRedSide)) \
                   .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

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
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                   .addPathStage(self.loadTrajectory("side-upper-back-v02", r.onRedSide)) \
                   .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()
            traj = self.loadTrajectory("sideFar-upper-v02", r.onRedSide)
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                   .addPathStage(self.loadTrajectory("sideFar-upper-back-v02", r.onRedSide)) \
                   .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()

        elif self.autoChooser.getSelected() == AUTO_SIDEUPPER_3PC:
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
            traj = self.loadTrajectory("sideFar-upper-v02", r.onRedSide)
            auto.addIntakeStage().triggerAlongPath(0.5, traj)
            auto.addIntakeStage()
            auto.addStageSet(AutoBuilder() \
                        .addPathStage(self.loadTrajectory("sideFar-upper-back-v02", r.onRedSide)) \
                        .addShooterPrepStage(ShooterTarget.SUBWOOFER, True))
            auto.addShooterFireStage()


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

        return auto, initialPose
