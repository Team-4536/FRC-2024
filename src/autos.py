
from copy import copy
from typing import TYPE_CHECKING, Callable

from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory
from noteStateMachine import ShooterTarget

import math
from wpimath.geometry import Pose2d

if TYPE_CHECKING:
    from robot import Robot

StageFunc = Callable[['Robot'], bool | None]
class Stage:
    def __init__(self, f: StageFunc, name: str) -> None:
        self.func = f
        self.name = name
        self.nextStage: 'Stage | None' = None
        self.abortStage: 'Stage | None' = None

    def add(self, s: 'Stage') -> tuple['Stage', 'Stage']:
        self.nextStage = s
        return self, s

class AutoBuilder:
    def __init__(self) -> None:
        self.firstStage: Stage | None = None
        self.currentBuildStage: Stage = None # type: ignore
        self.currentRunStage: Stage | None = None

        self.stageStart = 0 # This is the trigger for restarting a seq. when 0, run will start from firstRunStage
        self.table = NetworkTableInstance.getDefault().getTable("autos")

    # Resets and makes sure StageBuilder will start up from the beginning of the sequence next run call
    def reset(self, time: float) -> None:
        self.stageStart = 0

    # runs one tick, returns if still running or finished
    def run(self, r: 'Robot') -> bool:
        if self.stageStart == 0:
            self.stageStart = r.time.timeSinceInit
            self.currentRunStage = self.firstStage

        self.table.putNumber("stageTime", self.stageStart)
        if self.currentRunStage is not None:
            self.table.putString("stage", f"{self.currentRunStage.name}")
            done = self.currentRunStage.func(r)
            if done is True:
                self.currentRunStage = self.currentRunStage.nextStage
                self.stageStart = r.time.timeSinceInit
            elif done is None:
                self.currentRunStage = self.currentRunStage.abortStage
                self.stageStart = r.time.timeSinceInit

        return self.currentRunStage is None

    def add(self, new: Stage | None) -> 'AutoBuilder':
        if new is None:
            return self

        if self.currentBuildStage is not None:
            self.currentBuildStage.nextStage = new
        self.currentBuildStage = new

        if self.firstStage is None:
            self.firstStage = new
        return self

    def addAbort(self, new: 'AutoBuilder') -> 'AutoBuilder':
        if self.currentBuildStage is not None:
            self.currentBuildStage.abortStage = new.firstStage
        return self

    def addAbortLog(self, log: str) -> 'AutoBuilder':
        if self.currentBuildStage is not None:
            self.currentBuildStage.abortStage = AutoBuilder().addTelemetryStage(log).currentBuildStage
        return self

    # NOTE: TODO: doesn't make a copy of the new stages - so watch out
    def addSequence(self, s: 'AutoBuilder') -> 'AutoBuilder':
        self.add(s.firstStage)
        while self.currentBuildStage.nextStage is not None:
            self.currentBuildStage = self.currentBuildStage.nextStage
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
    def addPathStage(self, t: PathPlannerTrajectory, trajName: str = "unnamed") -> 'AutoBuilder':
        self.add(self._newPathStage(t, trajName))
        return self

    def addWaitStage(self, t: float) -> 'AutoBuilder':
        def func(r : 'Robot') -> bool | None:
            return (r.time.timeSinceInit - r.auto.stageStart) > t

        self.add(Stage(func, f"wait for {t}s"))
        return self

    def addIntakeStage(self) -> 'AutoBuilder':
        def func(r: 'Robot') -> bool | None:
            r.noteStateMachine.intake(True)
            return (r.noteStateMachine.state == r.noteStateMachine.STORED_IN_SHOOTER)
        self.add(Stage(func, "intake ring"))
        return self

    # return status is dictated by the path stage, the triggered stages return is ignored (including aborts)
    # starts up the triggered stage when the path has covered [percent] of its total time
    def triggerAlongPath(self, percent: float, t: PathPlannerTrajectory, trajName: str = "unnamed") -> 'AutoBuilder':
        stg = copy(self.currentBuildStage)
        pathStage = self._newPathStage(t, trajName)
        def func(r: 'Robot') -> bool|None:
            isOver = pathStage.func(r)
            if ((r.time.timeSinceInit - r.auto.stageStart) > (t.getTotalTimeSeconds() * percent)):
                stg.func(r)
            return isOver

        self.currentBuildStage.func = func
        self.currentBuildStage.name = f"path with {stg.name} trigger"
        self.currentBuildStage.abortStage = None
        return self

    # ends when state is on target
    def addShooterPrepStage(self, target: ShooterTarget, rev: bool) -> 'AutoBuilder':
        def func(r: 'Robot') -> bool | None:
            r.noteStateMachine.feed(True)
            r.noteStateMachine.aim(target)
            r.noteStateMachine.rev(rev)
            return r.noteStateMachine.onTarget
        self.add(Stage(func, f"shooter aim at {target.name}, revved: {rev}"))
        return self

    def addShooterFireStage(self) -> 'AutoBuilder':
        def func(r: 'Robot') -> bool | None:
            r.noteStateMachine.shoot(True)
            return r.noteStateMachine.state == r.noteStateMachine.START
        self.add(Stage(func, "fire shooter"))
        return self

    def addTelemetryStage(self, s: str) -> 'AutoBuilder':
        def func(r: 'Robot') -> bool | None:
            NetworkTableInstance.getDefault().getTable("autos").putString("telemStageLog", s)
            return True
        self.add(Stage(func, f"logging {s}"))
        return self

    # NOTE: when making the stage set, this iterates through the stages in the passed list
    # it goes by the nextStage of each, none of the aborts
    # aborts from the inner stages are reported, but the sets abort is what get moved to

    # Stage set executes all and then ends when all are done
    def addStageSet(self, stages: 'AutoBuilder') -> 'AutoBuilder':
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

    # NOTE: triggers abort when the timeout is hit, moves to nextStage and abortStage of the given stage
    # passes through aborts from the inner stage
    def setTimeout(self, duration: float) -> 'AutoBuilder':
        stg = copy(self.currentBuildStage)
        def func(r: 'Robot') -> bool | None:
            status = stg.func(r)
            if status is None:
                return None
            elif (r.time.timeSinceInit - r.auto.stageStart) > duration:
                return None
            else:
                return status

        self.currentBuildStage.name = f"{stg.name} with timeout"
        self.currentBuildStage.func = func
        self.currentBuildStage.abortStage = None
        return self
    
    def addOdometryResetWithLimelightStage(self, r: 'Robot', pipeline: int) -> 'AutoBuilder':
        limelightTable = r.frontLimelightTable
        robotPoseTable = r.robotPoseTable
        def func(r: 'Robot') -> bool:
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
            elif (r.time.timeSinceInit - r.auto.stageStart) > 0.25:
                return True
            return False
        
        self.add(Stage(func, "reset odom with limelight"))
        return self