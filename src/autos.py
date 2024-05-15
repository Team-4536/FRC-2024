from inspect import signature

import autoUtils
import robot
import wpilib
from ntcore import NetworkTableInstance
from shooterStateMachine import ShooterTarget

"""
DOCS FOR HOW AUTOS WORK
    (info for this file as well as autoUtils.py)
    (as of first implementation, may 14 2024, by rob)

RIP Stages, you will abolutely not be missed

- ****Autos are just functions****, plain an simple.
    normal things about composition and parameterization are still on, DRY to a good degree

- The key piece that makes this work is the yield keyword (AKA coroutines AKA generators)
    They essentially allow your function to exit in the middle and retirn to the location it left the next time
    it's called. It means autos can be structured as a sequence of normal code, with while loops and all and
    they won't explode the robot becuase the function is still being called and returning every frame. Any state
    created in the auto function stays valid when it yields and then is called again. The trajectories you see
    in the autos below really are only loaded once, but they stay valid between frames.

- 'yield' and 'yield from' both expect a value, which for this system should be a string. It's put to NT under
    the key 'autoMessage', and should be a description of why the auto yielded at that point.

- 'yield from' will yield the value from the inner function, only when that function yields. If it doesn't, the
    statement just moves on without exiting.

- All functions defined in RobotAutos with the signature "def fun(r: 'robot.Robot'):" are automagically
put into the chooser that appears on the dash, see RobotAutos.__init__() for the (terrible) implementation
    // I'm very unhappy that the functions aren't typed, if someone wants to take a crack at getting the right
        signature go for it, but the generators mess with it a lot.
    // note that the exact signiture isn't actually checked for, just evey fn with one arg
    // incorrect sigs with one arg will crash this, except for some basic ones that were excluded, again, see the impl.

- This class is only meant to be used from robot.py, once

- anything that you're gonna reuse that shouldn't be on the dash, put into the autoUtils file.

- check autoUtils and this file for good examples of how to create a decent set of autos

"""

class RobotAutos():
    # collects all of the functions in this class and puts them into a chooser on the dash.
    def __init__(self) -> None:
        self.autoChooser = wpilib.SendableChooser()
        self.auto = None
        outputTopic = NetworkTableInstance.getDefault().getStringTopic("autoMessage")
        self.outputPub = outputTopic.publish()

        self.autoChooser.setDefaultOption('doNothing', 'doNothing')
        for key in RobotAutos.__dict__:
            if key == '__init__':
                continue
            elif key == 'runAuto':
                continue
            elif key == 'doNothing':
                continue

            obj = RobotAutos.__dict__[key]
            if callable(obj) and (len(signature(obj).parameters) == 1):
                self.autoChooser.addOption(key, key)

        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

    # takes what auto has been selected on the dash and creates the generator for it
    # if the key is invalid, self.auto = None
    def initAuto(self, r: 'robot.Robot'):
        auto = RobotAutos.__dict__[self.autoChooser.getSelected()]
        if auto is not None:
            self.auto = auto(r)
        else:
            self.auto = None

    def runAuto(self):
        if(self.auto is not None):
            try:
                self.outputPub.set(self.auto.__next__())
            except StopIteration:
                self.outputPub.set("[finished executing]")

    @staticmethod
    def doNothing(r: 'robot.Robot'):
        pass

    @staticmethod
    def shootStartingRing(r: 'robot.Robot'):
        yield from autoUtils.intakeUntilRingGot(r)
        while not autoUtils.prepShooter(r, ShooterTarget.SUBWOOFER, True):
            yield "waiting on shooter prep"
        yield from autoUtils.fireShooterUntilDone(r)

    @staticmethod
    def shootThenIntakeCenterRing(r: 'robot.Robot'):
        middleTraj = autoUtils.loadTrajectory("middle", r.onRedSide)
        returnTraj = autoUtils.loadTrajectory("middleBack", r.onRedSide)
        r.resetGyroAndOdomToPose(middleTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, middleTraj, returnTraj)

    @staticmethod
    def troll(r: 'robot.Robot'):
        traj = autoUtils.loadTrajectory("troll", r.onRedSide)
        r.resetGyroAndOdomToPose(traj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.runPathUntilDone(r, traj)

    @staticmethod
    def getAllMidUpLow(r: 'robot.Robot'):
        middleOut = autoUtils.loadTrajectory("middle", r.onRedSide)
        middleBack = autoUtils.loadTrajectory("middleBack", r.onRedSide)
        upperOut = autoUtils.loadTrajectory("upper", r.onRedSide)
        upperBack = autoUtils.loadTrajectory("upperBack", r.onRedSide)
        lowerOut = autoUtils.loadTrajectory("lower", r.onRedSide)
        lowerBack = autoUtils.loadTrajectory("lowerBack", r.onRedSide)
        r.resetGyroAndOdomToPose(middleOut.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, middleOut, middleBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from autoUtils.scoreRing(r, upperOut, upperBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from autoUtils.scoreRing(r, lowerOut, lowerBack)

    @staticmethod
    def getAllMidLowUp(r: 'robot.Robot'):
        middleOut = autoUtils.loadTrajectory("middle", r.onRedSide)
        middleBack = autoUtils.loadTrajectory("middleBack", r.onRedSide)
        upperOut = autoUtils.loadTrajectory("upper", r.onRedSide)
        upperBack = autoUtils.loadTrajectory("upperBack", r.onRedSide)
        lowerOut = autoUtils.loadTrajectory("lower", r.onRedSide)
        lowerBack = autoUtils.loadTrajectory("lowerBack", r.onRedSide)

        r.resetGyroAndOdomToPose(middleOut.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, middleOut, middleBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from autoUtils.scoreRing(r, lowerOut, lowerBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from autoUtils.scoreRing(r, upperOut, upperBack)

    @staticmethod
    def exitBackwards(r: 'robot.Robot'):
        traj = autoUtils.loadTrajectory("exit", r.onRedSide)
        r.resetGyroAndOdomToPose(traj.getInitialTargetHolonomicPose())
        yield from autoUtils.runPathUntilDone(r, traj)

    @staticmethod
    def shootAndGetFarMiddle(r: 'robot.Robot'):
        outTraj = autoUtils.loadTrajectory("far-middle", r.onRedSide)
        returnTraj = autoUtils.loadTrajectory("far-middle", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, outTraj, returnTraj)

    @staticmethod
    def shootFromUpperSpeakerAndScoreUpperNote(r: 'robot.Robot'):
        outTraj = autoUtils.loadTrajectory("side-upper", r.onRedSide)
        returnTraj = autoUtils.loadTrajectory("side-upper-back", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, outTraj, returnTraj)

    @staticmethod
    def shootFromUpperSpeakerAndScoreTwo(r: 'robot.Robot'):
        outTraj = autoUtils.loadTrajectory("side-upper", r.onRedSide)
        returnTraj = autoUtils.loadTrajectory("side-upper-back", r.onRedSide)
        farOutTraj = autoUtils.loadTrajectory("sideFar-upper-v02", r.onRedSide)
        farReturnTraj = autoUtils.loadTrajectory("sideFar-upper-back-v02", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, outTraj, returnTraj)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from autoUtils.scoreRing(r, farOutTraj, farReturnTraj)

    @staticmethod
    def shootFromLowerSpeakerAndScoreLowerNote(r: 'robot.Robot'):
        outTraj = autoUtils.loadTrajectory("side-lower", r.onRedSide)
        returnTraj = autoUtils.loadTrajectory("side-lower-back", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.scoreRing(r, outTraj, returnTraj)
