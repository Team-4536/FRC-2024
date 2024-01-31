import autoStaging
import robot
from ntcore import NetworkTableInstance
from wpimath._controls._controls.trajectory import Trajectory


def makePathStage(t: Trajectory) -> autoStaging.Stage:
    def stage(r: robot.Robot) -> bool:
        currentPose = r.drive.odometry.getPose()
        goal = r.trajectory.sample(r.time.timeSinceInit - r.autoStartTime)

        adjustedSpeeds = r.holonomicController.calculate(
            currentPose, goal, goal.pose.rotation())

        r.drive.update(r.time.dt, r.hal, adjustedSpeeds)
        return (r.time.timeSinceInit - r.auto.stagestart) > r.trajectory.totalTime()
    return stage

def makeTelemetryStage(s: str) -> autoStaging.Stage:
    def log(r: robot.Robot) -> bool:
        NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
        return True
    return log