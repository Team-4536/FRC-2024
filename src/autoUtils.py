import math

import robot
import wpilib
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

# TODO: ABORTS
# TODO: TIMEOUTS


# NOTE: filename is *just* the title of the file, with no extension and no path
# filename is directly passed to pathplanner.loadPath
def loadTrajectory(fileName: str, flipped: bool) -> PathPlannerTrajectory:
    p = PathPlannerPath.fromPathFile(fileName)
    if flipped:
        p = p.flipPath()
    t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
    return t

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
        yield "running path until finished"

def intakeUntilRingGot(r: 'robot.Robot'):
    while(r.intakeStateMachine.state != r.intakeStateMachine.STORING):
        r.intakeStateMachine.update(r.hal, True)
        yield "waiting on ring intake"

def wait(duration: float):
    t = wpilib.Timer()
    t.start()
    while(t.get() < duration):
        yield f"waiting for {duration}s"

def prepShooter(r: 'robot.Robot', target: ShooterTarget, rev: bool) -> bool:
    r.shooterStateMachine.feed(True)
    r.shooterStateMachine.aim(target)
    r.shooterStateMachine.rev(rev)
    return r.shooterStateMachine.onTarget

# TODO: make this not frame dependant
def fireShooterUntilDone(r: 'robot.Robot'):
    while r.shooterStateMachine.state != r.shooterStateMachine.READY_FOR_RING:
        r.shooterStateMachine.shoot(True)
        yield "waiting on shooter to return to ready state"

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

def scoreRing(r: 'robot.Robot', outTraj: PathPlannerTrajectory, returnTraj: PathPlannerTrajectory):
    t = wpilib.Timer()
    t.start()
    while(t.get() < outTraj.getTotalTimeSeconds()):
        r.intakeStateMachine.update(r.hal, True)
        runPath(r, outTraj, t.get())
        yield "waiting on running out path"

    yield from intakeUntilRingGot(r)

    # run back
    t.restart()
    while(t.get() < returnTraj.getTotalTimeSeconds()):
        prepShooter(r, ShooterTarget.SUBWOOFER, True)
        runPath(r, returnTraj, t.get())
        yield "waiting for return path"

    while not prepShooter(r, ShooterTarget.SUBWOOFER, True):
        yield "waiting on shooter prep"

    yield from fireShooterUntilDone(r)
