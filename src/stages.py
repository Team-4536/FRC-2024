from copy import copy, deepcopy

from typing import TYPE_CHECKING


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
from real import angleWrap
from shooterStateMachine import ShooterTarget

from wpimath import angleModulus

from wpimath.kinematics import ChassisSpeeds
import math



if TYPE_CHECKING:
    from robot import Robot


class StageBuilder:
    def __init__(self) -> None:
        self.firstStage: Stage | None = None
        self.currentStage: Stage = None # type: ignore

    def add(self, new: Stage | None) -> 'StageBuilder':
        if new is None:
            return self

        if self.currentStage is not None:
            self.currentStage.nextStage = new
        self.currentStage = new

        if self.firstStage is None:
            self.firstStage = new
        return self

    def addAbortLog(self, log: str) -> 'StageBuilder':
        if self.currentStage is not None:
            self.currentStage.abortStage = StageBuilder().addTelemetryStage(log).currentStage
        return self

    def addAbort(self, new: 'StageBuilder') -> 'StageBuilder':
        if self.currentStage is not None:
            self.currentStage.abortStage = new.firstStage
        return self

    def _newPathStage(self, t: PathPlannerTrajectory, trajName: str) -> Stage:
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
        s = Stage(func, f"path '{trajName}'")
        return s

    # NOTE: filename is *just* the title of the file, with no extension and no path
    # filename is directly passed to pathplanner.loadPath
    def addPathStage(self, t: PathPlannerTrajectory, trajName: str = "unnamed") -> 'StageBuilder':
        self.add(self._newPathStage(t, trajName))
        return self

    def addWaitStage(self, t: float) -> 'StageBuilder':
        def func(r : 'Robot') -> bool | None:
            return (r.time.timeSinceInit - r.auto.stageStart) > t

        self.add(Stage(func, f"wait for {t}s"))
        return self

    def addIntakeStage(self) -> 'StageBuilder':
        def func(r: 'Robot') -> bool | None:
            r.intakeStateMachine.update(r.hal, True)
            return (r.intakeStateMachine.state == r.intakeStateMachine.STORING)
        self.add(Stage(func, "intake ring"))
        return self

    # return status is dictated by the path stage, the triggered stages return is ignored (including aborts)
    # starts up the triggered stage when the path has covered [percent] of its total time
    def triggerAlongPath(self, percent: float, t: PathPlannerTrajectory, trajName: str = "unnamed") -> 'StageBuilder':
        stg = copy(self.currentStage)
        pathStage = self._newPathStage(t, trajName)
        def func(r: 'Robot') -> bool|None:
            isOver = pathStage.func(r)
            if ((r.time.timeSinceInit - r.auto.stageStart) > (t.getTotalTimeSeconds() * percent)):
                stg.func(r)
            return isOver

        self.currentStage.func = func
        self.currentStage.name = f"path with {stg.name} trigger"
        self.currentStage.abortStage = None
        return self

    # ends when state is on target
    def addShooterPrepStage(self, target: ShooterTarget, rev: bool) -> 'StageBuilder':
        def func(r: 'Robot') -> bool | None:
            r.shooterStateMachine.aim(target)
            r.shooterStateMachine.rev(rev)
            return r.shooterStateMachine.onTarget
        self.add(Stage(func, f"shooter aim at {target.name}, revved: {rev}"))
        return self

    def addShooterFireStage(self) -> 'StageBuilder':
        def func(r: 'Robot') -> bool | None:
            r.shooterStateMachine.shoot(True)
            return r.shooterStateMachine.state == r.shooterStateMachine.READY_FOR_RING
        self.add(Stage(func, "fire shooter"))
        return self

    def addTelemetryStage(self, s: str) -> 'StageBuilder':
        def func(r: 'Robot') -> bool | None:
            NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
            return True
        self.add(Stage(func, f"logging {s}"))
        return self

    # NOTE: when making the stage set, this iterates through the stages in the passed list
    # it goes by the nextStage of each, none of the aborts
    # aborts from the inner stages are reported, but the sets abort is what get moved to

    # Stage set executes all and then ends when all are done
    def addStageSet(self, stages: 'StageBuilder') -> 'StageBuilder':
        stageList: list[Stage] = []
        s = stages.firstStage
        while s is not None:
            stageList.append(s)
            s = s.nextStage

        def func(r: 'Robot') -> bool | None:
            aborted = False
            incomplete = False
            for s in stageList:
                status = s.func(r)
                if status is False:
                    incomplete = True
                elif status is None:
                    aborted = True
                s = s.nextStage

            if aborted:
                return None
            return not incomplete
        self.add(Stage(func, f"set [{', '.join(s.name for s in stageList)}]"))
        return self

    # NOTE: doesn't make a copy of the new stages - so watch out
    def addStageBuiltStage(self, s: 'StageBuilder') -> 'StageBuilder':
        self.add(s.firstStage)
        while self.currentStage.nextStage is not None:
            self.currentStage = self.currentStage.nextStage
        return self

    # NOTE: triggers abort when the timeout is hit, moves to nextStage and abortStage of the given stage
    # passes through aborts from the inner stage
    def setTimeout(self, duration: float) -> 'StageBuilder':
        stg = copy(self.currentStage)
        def func(r: 'Robot') -> bool | None:
            status = stg.func(r)
            if status is None:
                return None
            elif (r.time.timeSinceInit - r.auto.stageStart) > duration:
                return None
            else:
                return status

        self.currentStage.name = f"{stg.name} with timeout"
        self.currentStage.func = func
        self.currentStage.abortStage = None
        return self

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
