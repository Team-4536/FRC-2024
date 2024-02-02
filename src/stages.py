from typing import TYPE_CHECKING

from auto import Stage
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
        return (r.time.timeSinceInit - r.auto.stagestart) > t.totalTime()
    return stage
#speed is percentage 
def makeIntake(time: float, speed: float):
    def stage(r: 'Robot') -> bool:
        r.hal.driveSpeeds[0] = speed
        if (r.time.timeSinceInit - r.auto.stagestart) > time:
            return True
        else:
            r.hal.driveSpeeds[0] = speed
        return False
    return stage
        
def makeTelemetryStage(s: str) -> Stage:
    def log(r: 'Robot') -> bool:
        NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
        return True
    return log