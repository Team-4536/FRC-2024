from typing import TYPE_CHECKING, Self
import wpimath
from wpimath import controller
from wpimath import trajectory, geometry, controller
from wpimath.trajectory import TrapezoidProfile, Trajectory, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import ProfiledPIDController, HolonomicDriveController, ProfiledPIDControllerRadians, PIDController
import stages
from auto import Stage
from ntcore import NetworkTable, NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.kinematics import ChassisSpeeds
import math


if TYPE_CHECKING:
    from robot import Robot

def makePathStage(t: PathPlannerTrajectory) -> Stage:
    def stage(r: 'Robot') -> bool:
        goal = t.sample(r.time.timeSinceInit - r.auto.stagestart)

        table = NetworkTableInstance.getDefault().getTable("autos")
        table.putNumber("pathGoalX", goal.getTargetHolonomicPose().X())
        table.putNumber("pathGoalY", goal.getTargetHolonomicPose().Y())
        table.putNumber("pathGoalR", goal.getTargetHolonomicPose().rotation().radians())

        table.putNumber("odomR", r.drive.odometry.getEstimatedPosition().rotation().radians())
        adjustedSpeeds = r.holonomicController.calculateRobotRelativeSpeeds(r.drive.odometry.getEstimatedPosition(), goal)
        table.putNumber("pathVelX", adjustedSpeeds.vx)
        table.putNumber("pathVelY", adjustedSpeeds.vy)
        table.putNumber("pathVelR", adjustedSpeeds.omega)

        r.drive.update(r.time.dt, r.hal, adjustedSpeeds)
        return (r.time.timeSinceInit - r.auto.stagestart) > t.getTotalTimeSeconds()
    return stage

def makeWaitStage(t: float) -> Stage:
    def stage(r : 'Robot'):
        return (r.time.timeSinceInit - r.auto.stagestart) > t
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

# ends when state is on target
def makeShooterPrepStage(target: ShooterTarget, rev: bool) -> Stage:
    def stage(r: 'Robot') -> bool:
        r.shooterStateMachine.aim(target)
        r.shooterStateMachine.rev(rev)
        return r.shooterStateMachine.onTarget
    return stage

def makeShooterFireStage() -> Stage:
    def stage(r: 'Robot') -> bool:
        r.shooterStateMachine.shoot(True)
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


def odometryResetWithLimelight(r: 'Robot', pipeline: int) -> Stage:
    limelightTable = r.FrontLimelightTable
    robotPoseTable = r.robotPoseTable

    def stage(r: 'Robot') -> bool:
        #gets the pos from limelight
        visionPose = limelightTable.getNumberArray("botpose_wpiblue", [0,0,0,0,0,0,0])
        #debug values
        robotPoseTable.putNumber("limeXPos", visionPose[0])
        robotPoseTable.putNumber("limeYPos", visionPose[1])
        robotPoseTable.putNumber("limeYaw", visionPose[5])
        #make sure there is a value present and has input(R3) to update
        if (not (visionPose[0] == 0 and visionPose[1] == 0 and visionPose[5] == 0)):  
            visionPose2D = Pose2d(visionPose[0], visionPose[1], math.radians(visionPose[5]))
            #X, Y, & Yaw are updated correctly
            r.drive.resetOdometry(visionPose2D, r.hal)


        return True
    
    return stage