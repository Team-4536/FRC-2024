import math

import robot
import wpilib
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.geometry import Pose2d

#TODO: ABORTS
# TODO: TIMEOUTS

def runPath(r: 'robot.Robot', t: PathPlannerTrajectory, timeIntoTraj: float):
    goal = t.sample(timeIntoTraj)

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

def runPathUntilDone(r: 'robot.Robot', t: PathPlannerTrajectory):
    timer = wpilib.Timer()
    timer.start()
    while(timer.get() < t.getTotalTimeSeconds()):
        runPath(r, t, timer.get())
        yield 0

def intakeUntilRingGot(r: 'robot.Robot'):
    while(r.intakeStateMachine.state != r.intakeStateMachine.STORING):
        r.intakeStateMachine.update(r.hal, True)
        yield 0

def wait(duration: float):
    t = wpilib.Timer()
    t.start()
    while(t.get() < duration):
        yield 0

def prepShooter(r: 'robot.Robot', target: ShooterTarget, rev: bool) -> bool:
    r.shooterStateMachine.feed(True)
    r.shooterStateMachine.aim(target)
    r.shooterStateMachine.rev(rev)
    return r.shooterStateMachine.onTarget

# TODO: make this not frame dependant
def fireShooterUntilDone(r: 'robot.Robot'):
    while r.shooterStateMachine.state != r.shooterStateMachine.READY_FOR_RING:
        r.shooterStateMachine.shoot(True)
        yield 0

def tryResetOdomWithLimelight(r: 'robot.Robot', pipeline: int) -> bool:
    limelightTable = r.frontLimelightTable
    robotPoseTable = r.robotPoseTable

    if(limelightTable.getNumber("getPipe", -1) != pipeline):
        limelightTable.putNumber("pipeline", pipeline)

    #gets the pos from limelight
    visionPose = limelightTable.getNumberArray("botpose_wpiblue", [0,0,0,0,0,0,0])
    #debug values
    robotPoseTable.putNumber("limeXPos", visionPose[0])
    robotPoseTable.putNumber("limeYPos", visionPose[1])
    robotPoseTable.putNumber("limeYaw", visionPose[5])
    if (not (visionPose[0] == 0 and visionPose[1] == 0 and visionPose[5] == 0)):
        visionPose2D:Pose2d = Pose2d(visionPose[0], visionPose[1], math.radians(visionPose[5]))

        #X, Y, & Yaw are updated correctly
        r.drive.resetOdometry(visionPose2D, r.hal)
        return True
    return False
