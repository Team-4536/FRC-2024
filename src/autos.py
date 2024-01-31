from typing import TYPE_CHECKING

from autoStaging import Stage
from ntcore import NetworkTableInstance
from wpimath._controls._controls.trajectory import Trajectory

if TYPE_CHECKING:
    from robot import Robot

def makePathStage(t: Trajectory) -> Stage:
    def stage(r: 'Robot') -> bool:
        currentPose = r.drive.odometry.getPose()
        goal = t.sample(r.time.timeSinceInit - r.auto.stagestart)

        adjustedSpeeds = r.holonomicController.calculate(
            currentPose, goal, goal.pose.rotation())

        r.drive.update(r.time.dt, r.hal, adjustedSpeeds)
        return (r.time.timeSinceInit - r.auto.stagestart) > r.trajectory.totalTime()
    return stage

def makeTelemetryStage(s: str) -> Stage:
    def log(r: 'Robot') -> bool:
        NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
        return True
    return log