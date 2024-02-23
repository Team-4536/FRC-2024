from typing import TYPE_CHECKING, Self
import wpimath
from wpimath import controller
from wpimath import trajectory, geometry, controller
from wpimath.trajectory import TrapezoidProfile, Trajectory, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import ProfiledPIDController, HolonomicDriveController, ProfiledPIDControllerRadians, PIDController
import stages
from auto import Stage
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget

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

def goToAprilTag(self,tx, ty, tr) -> Stage:
        self.hal.yaw = 0
        self.drive.resetOdometry(Pose2d(1.166,5.522,Rotation2d(0)), self.hal)
        self.autoStartTime = self.time.timeSinceInit
       
        self.table.putNumber("path/Xp", 1)
        
        self.table.putNumber("path/Yp", 1)
        RControllerP = 7
        RControllerI = 0
        RControllerD = 0
        self.table.putNumber('path/Rp', 7)
        T_PConstraintsVolocityMax = 6.28
        T_PConstraintsRotaionAccelerationMax = 1
        self.XController = PIDController(
            self.table.getNumber('path/Xp'), 0, 0)
        self.YController = PIDController(
            self.table.getNumber('path/Yp'), 0, 0)
        self.RotationController = ProfiledPIDController(
            self.table.getNumber('path/Rp'), 0, 0, TrapezoidProfile.Constraints(T_PConstraintsVolocityMax, T_PConstraintsRotaionAccelerationMax))
       
        
        currentPose = self.drive.odometry.getPose()
        # goal = self.trajectory.sample(self.time.timeSinceInit - self.autoStartTime)
        goal = Trajectory.State()
       
        xSpeed = self.XController.calculate(currentPose.X(), tx)
        ySpeed = self.YController.calculate(currentPose.Y(), ty)
        rSpeed = self.RotationController.calculate(currentPose.rotation().radians(), tr)
        
        adjustedSpeeds = [xSpeed, ySpeed, rSpeed]
        
        self.drive.adjustSpeeds
        
        
    
        return Self