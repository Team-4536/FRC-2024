import robot
import wpilib
from autos import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

AUTO_TROLL = 'mess up middle rings'



"""
A small class to remove the functionality of auto choosers and construction from robot.py.
This was done to prevent large conflicts, and make it easy to change chooser options without jumping up and down 300 lines of other code.
This class is only intended to be used once in robotInit in robot.py, not anywhere else.
Class construction publishes choosers to the dashboard
"""
class RobotAutos():
    def __init__(self) -> None:
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.setDefaultOption(AUTO_TROLL, AUTO_TROLL)
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

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

        if self.autoChooser.getSelected() == AUTO_TROLL:
            traj = self.loadTrajectory("troll", r.onRedSide)
            initialPose = traj.getInitialState().getTargetHolonomicPose()
            auto.addTelemetryStage(AUTO_TROLL)
            auto.addShooterPrepStage(ShooterTarget.SUBWOOFER, True)
            auto.addShooterFireStage()
            auto.addPathStage(traj)

        else:
            assert(False)

        return auto, initialPose
