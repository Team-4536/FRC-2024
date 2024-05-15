from ast import Yield
from enum import auto
from inspect import signature
from pathlib import Path
from typing import Generator

import autos
import robot
import wpilib
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


# NOTE: filename is *just* the title of the file, with no extension and no path
# filename is directly passed to pathplanner.loadPath
def loadTrajectory(fileName: str, flipped: bool) -> PathPlannerTrajectory:
    p = PathPlannerPath.fromPathFile(fileName)
    if flipped:
        p = p.flipPath()
    t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
    return t

"""
A small class to remove the functionality of auto choosers and construction from robot.py.
This was done to prevent large conflicts, and make it easy to change chooser options without jumping up and down 300 lines of other code.
This class is only intended to be used once in robotInit in robot.py, not anywhere else.
Class construction publishes choosers to the dashboard
"""
class RobotAutos():
    def __init__(self) -> None:
        self.autoChooser = wpilib.SendableChooser()
        self.auto: Generator | None = None

        for key in self.__dict__:
            obj = self.__dict__[key]
            if callable(obj) and (len(signature(obj).parameters) == 1):
                print(f"we'd be adding {key}")

        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

    def initOrRunAuto(self, r: 'robot.Robot'):
        self.auto = self.__dict__[self.autoChooser.getSelected()]()
        assert(self.auto is not None)
        self.auto.__next__()

    @staticmethod
    def doNothing(r: 'robot.Robot'):
        yield 0

    @staticmethod
    def shootStartingRing(r: 'robot.Robot'):
        yield from autos.intakeUntilRingGot(r)
        while not autos.prepShooter(r, ShooterTarget.SUBWOOFER, True):
            yield 0
        yield from autos.fireShooterUntilDone(r)

    @staticmethod
    def scoreRing(r: 'robot.Robot', outTraj: PathPlannerTrajectory, returnTraj: PathPlannerTrajectory):
        t = wpilib.Timer()
        t.start()
        while(t.get() < outTraj.getTotalTimeSeconds()):
            r.intakeStateMachine.update(r.hal, True)
            autos.runPath(r, outTraj, t.get())
            yield 0

        while(r.intakeStateMachine.state != r.intakeStateMachine.STORING):
            r.intakeStateMachine.update(r.hal, True)
            yield 0

        # run back
        t.restart()
        while(t.get() < returnTraj.getTotalTimeSeconds()):
            autos.prepShooter(r, ShooterTarget.SUBWOOFER, True)
            autos.runPath(r, returnTraj, t.get())
            yield 0

        autos.prepShooter(r, ShooterTarget.PODIUM, True)
        yield from autos.fireShooterUntilDone(r)

    @staticmethod
    def shootThenIntakeCenterRing(r: 'robot.Robot'):
        initialPos: Pose2d = Pose2d()
        middleTraj = loadTrajectory("middle", r.onRedSide)
        returnTraj = loadTrajectory("middleBack", r.onRedSide)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, middleTraj, returnTraj)

    @staticmethod
    def troll(r: 'robot.Robot'):
        initialPos: Pose2d = Pose2d()
        traj = loadTrajectory("troll", r.onRedSide)
        yield from RobotAutos.shootStartingRing(r)
        yield from autos.runPathUntilDone(r, traj)

    @staticmethod
    def getAllMidUpLow(r: 'robot.Robot'):
        middleOut = loadTrajectory("middle", r.onRedSide)
        middleBack = loadTrajectory("middleBack", r.onRedSide)
        upperOut = loadTrajectory("upper", r.onRedSide)
        upperBack = loadTrajectory("upperBack", r.onRedSide)
        lowerOut = loadTrajectory("lower", r.onRedSide)
        lowerBack = loadTrajectory("lowerBack", r.onRedSide)

        initialPose = middleOut.getInitialTargetHolonomicPose()

        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, middleOut, middleBack)
        yield from RobotAutos.scoreRing(r, upperOut, upperBack)
        yield from RobotAutos.scoreRing(r, lowerOut, lowerBack)

    @staticmethod
    def getAllMidLowUp(r: 'robot.Robot'):
        middleOut = loadTrajectory("middle", r.onRedSide)
        middleBack = loadTrajectory("middleBack", r.onRedSide)
        upperOut = loadTrajectory("upper", r.onRedSide)
        upperBack = loadTrajectory("upperBack", r.onRedSide)
        lowerOut = loadTrajectory("lower", r.onRedSide)
        lowerBack = loadTrajectory("lowerBack", r.onRedSide)

        initialPose = middleOut.getInitialTargetHolonomicPose()

        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, middleOut, middleBack)
        yield from RobotAutos.scoreRing(r, lowerOut, lowerBack)
        yield from RobotAutos.scoreRing(r, upperOut, upperBack)

    @staticmethod
    def exitBackwards(r: 'robot.Robot'):
        traj = loadTrajectory("exit", r.onRedSide)
        initialPose = Pose2d()
        yield from autos.runPathUntilDone(r, traj)

    @staticmethod
    def ShootAndGetFarMiddle(r: 'robot.Robot'):
        outTraj = loadTrajectory("far-middle", r.onRedSide)
        returnTraj = loadTrajectory("far-middle", r.onRedSide)
        initialPose = outTraj.getInitialState().getTargetHolonomicPose()
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)

    @staticmethod
    def ShootFromUpperSpeakerAndScoreUpperNote(r: 'robot.Robot'):
        outTraj = loadTrajectory("side-upper", r.onRedSide)
        returnTraj = loadTrajectory("side-upper-back", r.onRedSide)
        initialPose = outTraj.getInitialState().getTargetHolonomicPose()
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)

    @staticmethod
    def ShootFromUpperSpeakerAndScoreTwo(r: 'robot.Robot'):
        outTraj = loadTrajectory("side-upper", r.onRedSide)
        returnTraj = loadTrajectory("side-upper-back", r.onRedSide)
        farOutTraj = loadTrajectory("sideFar-upper-v02", r.onRedSide)
        farReturnTraj = loadTrajectory("sideFar-upper-back-v02", r.onRedSide)
        initialPose = outTraj.getInitialState().getTargetHolonomicPose()
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)
        yield from RobotAutos.scoreRing(r, farOutTraj, farReturnTraj)

    @staticmethod
    def ShootFromLowerSpeakerAndScoreLowerNote(r: 'robot.Robot'):
        outTraj = loadTrajectory("side-lower", r.onRedSide)
        returnTraj = loadTrajectory("side-lower-back", r.onRedSide)
        initialPose = outTraj.getInitialState().getTargetHolonomicPose()
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)
