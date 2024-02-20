from typing import TYPE_CHECKING

from auto import Stage
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget

if TYPE_CHECKING:
    from robot import Robot

class StageBuilder:
    def __init__(self) -> None:
        self.firstStage: Stage | None = None
        self.currentStage: Stage = None # type: ignore

    def add(self, new: Stage) -> 'StageBuilder':
        if self.firstStage is None:
            self.firstStage = new
            self.currentStage = new

        if self.currentStage is not None:
            self.currentStage.nextStage = new
            self.currentStage = new
        return self

    def addAbortLog(self, log: str) -> 'StageBuilder':
        if self.currentStage is not None:
            self.currentStage.abortStage = StageBuilder().addTelemetryStage(log).currentStage
        return self

    def addAbort(self, new: 'StageBuilder') -> 'StageBuilder':
        if self.currentStage is not None:
            self.currentStage.abortStage = new.firstStage
        return self

    def addPathStage(self, t: PathPlannerTrajectory) -> 'StageBuilder':
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

        if self.currentStage is not None:
            self.add(Stage(func, ""))
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
    def triggerAlongPath(self, t: PathPlannerTrajectory, percent: float) -> 'StageBuilder':
        curr = self.currentStage
        stagePath = StageBuilder().addPathStage(t).currentStage # TODO: this line is just awful
        def func(r: 'Robot') -> bool|None:
            isOver = stagePath.func(r)
            if ((r.time.timeSinceInit - r.auto.stageStart) > (t.getTotalTimeSeconds() * percent)):
                curr.func(r)
            return isOver
        self.add(Stage(func, f"path with {curr.name} trigger"))
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
    def addStageSet(self, stages: 'StageBuilder') -> 'StageBuilder':
        stageList: list[Stage] = []
        s = stages.firstStage
        while s is not None:
            stageList.append(s)

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
    def setTimeout(self, duration: float) -> 'StageBuilder':
        curr = self.currentStage
        def func(r: 'Robot') -> bool | None:
            status = curr.func(r)
            if status is None:
                return None
            elif (r.time.timeSinceInit - r.auto.stageStart) > duration:
                return None
            else:
                return status
        self.currentStage = Stage(func, f"{curr.name} with timeout")
        return self