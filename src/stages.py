from typing import TYPE_CHECKING

import stages
from auto import Stage
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget

if TYPE_CHECKING:
    from robot import Robot

def makePathStage(t: PathPlannerTrajectory) -> Stage:
    def func(r: 'Robot') -> bool | None:
        goal = t.sample(r.time.timeSinceInit - r.auto.stageStart)

        table = NetworkTableInstance.getDefault().getTable("autos")
        table.putNumber("pathGoalX", goal.getTargetHolonomicPose().X())
        table.putNumber("pathGoalY", goal.getTargetHolonomicPose().Y())
        table.putNumber("pathGoalR", goal.getTargetHolonomicPose().rotation().radians())

        table.putNumber("odomR", r.drive.odometry.getPose().rotation().radians())
        adjustedSpeeds = r.holonomicController.calculateRobotRelativeSpeeds(r.drive.odometry.getPose(), goal)
        table.putNumber("pathVelX", adjustedSpeeds.vx)
        table.putNumber("pathVelY", adjustedSpeeds.vy)
        table.putNumber("pathVelR", adjustedSpeeds.omega)

        r.drive.update(r.time.dt, r.hal, adjustedSpeeds)

        return (r.time.timeSinceInit - r.auto.stageStart) > t.getTotalTimeSeconds()
    return Stage(func, "")

def makeWaitStage(t: float) -> Stage:
    def func(r : 'Robot') -> bool | None:
        return (r.time.timeSinceInit - r.auto.stageStart) > t
    return Stage(func, f"wait for {t}s")

def makeIntakeStage() -> Stage:
    def func(r: 'Robot') -> bool | None:
        r.intakeStateMachine.update(r.hal, True)
        return (r.intakeStateMachine.state == r.intakeStateMachine.STORING)
    return Stage(func, "intake ring")

# return status is dictated by the path stage, the triggered stages return is ignored
def makePathStageWithTriggerAtPercent(t: PathPlannerTrajectory, percent: float, triggered: Stage) -> Stage:
    stagePath = stages.makePathStage(t)
    def func(r: 'Robot') -> bool|None:
        isOver = stagePath.func(r)
        if ((r.time.timeSinceInit - r.auto.stageStart) > (t.getTotalTimeSeconds() * percent)):
            triggered.func(r)
        return isOver
    return Stage(func, f"path with {triggered.name} trigger")

# ends when state is on target
def makeShooterPrepStage(target: ShooterTarget, rev: bool) -> Stage:
    def func(r: 'Robot') -> bool | None:
        r.shooterStateMachine.aim(target)
        r.shooterStateMachine.rev(rev)
        return r.shooterStateMachine.onTarget
    return Stage(func, f"shooter aim at {target.name}, revved: {rev}")

def makeShooterFireStage() -> Stage:
    def func(r: 'Robot') -> bool | None:
        r.shooterStateMachine.shoot(True)
        return r.shooterStateMachine.state == r.shooterStateMachine.READY_FOR_RING
    return Stage(func, "fire shooter")

def makeTelemetryStage(s: str) -> Stage:
    def func(r: 'Robot') -> bool | None:
        NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
        return True
    return Stage(func, f"logging {s}")

def makeStageSet(stages: list[Stage]) -> Stage:
    def func(r: 'Robot') -> bool | None:
        aborted = False
        incomplete = False
        for s in stages:
            status = s.func(r)
            if status is False:
                incomplete = True
            elif status is None:
                aborted = True

        if aborted:
            return None
        return not incomplete
    return Stage(func, f"set [{', '.join(s.name for s in stages)}]")

# NOTE: triggers abort when the timeout is hit, moves to nextStage and abortStage of the given stage
# passes through aborts from the inner stage
def makeStageWithTimeout(s: Stage, timeout: float) -> Stage:
    def func(r: 'Robot') -> bool | None:
        status = s.func(r)
        if status is None:
            return None
        elif (r.time.timeSinceInit - r.auto.stageStart) > timeout:
            return None
        else:
            return status
    return Stage(func, f"{s.name} with timeout")