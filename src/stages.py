from typing import TYPE_CHECKING

import stages
from auto import Stage
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory

if TYPE_CHECKING:
    from robot import Robot

def makePathStage(t: PathPlannerTrajectory) -> Stage:
    def stage(r: 'Robot') -> bool:
        goal = t.sample(r.time.timeSinceInit - r.auto.stagestart)

        table = NetworkTableInstance.getDefault().getTable("autos")
        table.putNumber("pathGoalX", goal.getTargetHolonomicPose().X())
        table.putNumber("pathGoalY", goal.getTargetHolonomicPose().Y())
        table.putNumber("pathGoalR", goal.getTargetHolonomicPose().rotation().radians())

        adjustedSpeeds = r.holonomicController.calculateRobotRelativeSpeeds(r.drive.odometry.getPose(), goal)
        table.putNumber("pathVelX", adjustedSpeeds.vx)
        table.putNumber("pathVelY", adjustedSpeeds.vy)
        table.putNumber("pathVelR", adjustedSpeeds.omega)

        r.drive.update(r.time.dt, r.hal, adjustedSpeeds)
        return (r.time.timeSinceInit - r.auto.stagestart) > t.getTotalTimeSeconds()
    return stage

def makeIntakeStage() -> Stage:
    def stage(r: 'Robot') -> bool:
        r.intakeStateMachine.update(r.hal, True)
        return r.intakeStateMachine.state == r.intakeStateMachine.STORING
    return stage

def makePathStageWithTriggerAtPercent(t: PathPlannerTrajectory, percent: float, triggered: Stage):
    stagePath = stages.makePathStage(t)
    def stage(r: 'Robot') -> bool:
        isOver = stagePath(r)
        if ((r.time.timeSinceInit - r.auto.stagestart) > (t.getTotalTimeSeconds() * percent)):
            triggered(r)
        return isOver
    return stage

# targets go: 0 - amp, 1 - podium, 2 - subwoofer
def makeShooterAimStage(target: int, rev: bool) -> Stage:
    flags = [False, False, False, False, False]
    assert(target >= 0 and target <= 2)
    flags[target] = True

    def stage(r: 'Robot') -> bool:
        r.shooterStateMachine.update(r.hal, False, False, False, rev, False, 0, r.time.timeSinceInit, r.time.dt)
        return True
    return stage

def makeShooterFireStage() -> Stage:
    def stage(r: 'Robot') -> bool:
        r.shooterStateMachine.update(r.hal, False, False, False, False, True, 0, r.time.timeSinceInit, r.time.dt)
        return True
    return stage

def makeTelemetryStage(s: str) -> Stage:
    def log(r: 'Robot') -> bool:
        NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
        return True
    return log