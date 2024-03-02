from typing import TYPE_CHECKING
import wpimath
from wpimath import kinematics
from wpimath.kinematics import ChassisSpeeds
import shooterStateMachine
from shooterStateMachine import StateMachine

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
    flags = [False, False, False]
    assert(target >= 0 and target <= 2)
    flags[target] = True

    def stage(r: 'Robot') -> bool:
        r.shooterStateMachine.update(r.hal, flags[0], flags[1], flags[2], rev, False, 0, r.time.timeSinceInit, r.time.dt)
        return r.shooterStateMachine.onTarget
    return stage

def makeShooterFireStage() -> Stage:
    def stage(r: 'Robot') -> bool:
        r.shooterStateMachine.update(r.hal, False, False, False, False, True, 0, r.time.timeSinceInit, r.time.dt)
        return r.shooterStateMachine.state == r.shooterStateMachine.READY_FOR_RING
    return stage

def makeTelemetryStage(s: str) -> Stage:
    def log(r: 'Robot') -> bool:
        NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
        return True
    return log

def makeStageSet(stages: list[Stage]) -> Stage:
    def stage(r: 'Robot') -> bool:
        broken: bool = False
        for s in stages:
            if not s(r):
                broken = True
        return not broken
    return stage

def dontGiveUp(r: 'Robot') -> Stage:
    def stage(r: 'Robot') -> bool:
        r.table.putNumber("forwardSpeedForStgae", 0.1)
        forwardSpeed = r.table.getNumber("forawrdSpeedForStage", 0.1)
        speed = ChassisSpeeds(forwardSpeed, 0, 0)
        s = StateMachine()
        if s.state == s.READY_FOR_RING:    
            r.drive.update(r.time.dt, r.hal, speed)
        else:
            haveRing = True
        return haveRing
    
    return stage
    
    