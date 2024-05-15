from inspect import signature

import autoUtils
import robot
import wpilib
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from shooterStateMachine import ShooterTarget
from wpimath.kinematics import ChassisSpeeds


# NOTE: filename is *just* the title of the file, with no extension and no path
# filename is directly passed to pathplanner.loadPath
def loadTrajectory(fileName: str, flipped: bool) -> PathPlannerTrajectory:
    p = PathPlannerPath.fromPathFile(fileName)
    if flipped:
        p = p.flipPath()
    t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
    return t

# TODO: document this fucking mess

class RobotAutos():
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
    def scoreRing(r: 'robot.Robot', outTraj: PathPlannerTrajectory, returnTraj: PathPlannerTrajectory):
        t = wpilib.Timer()
        t.start()
        while(t.get() < outTraj.getTotalTimeSeconds()):
            r.intakeStateMachine.update(r.hal, True)
            autoUtils.runPath(r, outTraj, t.get())
            yield "waiting on running out path"

        yield from autoUtils.intakeUntilRingGot(r)

        # run back
        t.restart()
        while(t.get() < returnTraj.getTotalTimeSeconds()):
            autoUtils.prepShooter(r, ShooterTarget.SUBWOOFER, True)
            autoUtils.runPath(r, returnTraj, t.get())
            yield "waiting for return path"

        while not autoUtils.prepShooter(r, ShooterTarget.SUBWOOFER, True):
            yield "waiting on shooter prep"

        yield from autoUtils.fireShooterUntilDone(r)

    @staticmethod
    def shootThenIntakeCenterRing(r: 'robot.Robot'):
        middleTraj = loadTrajectory("middle", r.onRedSide)
        returnTraj = loadTrajectory("middleBack", r.onRedSide)
        r.resetGyroAndOdomToPose(middleTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, middleTraj, returnTraj)

    @staticmethod
    def troll(r: 'robot.Robot'):
        traj = loadTrajectory("troll", r.onRedSide)
        r.resetGyroAndOdomToPose(traj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from autoUtils.runPathUntilDone(r, traj)

    @staticmethod
    def getAllMidUpLow(r: 'robot.Robot'):
        middleOut = loadTrajectory("middle", r.onRedSide)
        middleBack = loadTrajectory("middleBack", r.onRedSide)
        upperOut = loadTrajectory("upper", r.onRedSide)
        upperBack = loadTrajectory("upperBack", r.onRedSide)
        lowerOut = loadTrajectory("lower", r.onRedSide)
        lowerBack = loadTrajectory("lowerBack", r.onRedSide)
        r.resetGyroAndOdomToPose(middleOut.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, middleOut, middleBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.scoreRing(r, upperOut, upperBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.scoreRing(r, lowerOut, lowerBack)

    @staticmethod
    def getAllMidLowUp(r: 'robot.Robot'):
        middleOut = loadTrajectory("middle", r.onRedSide)
        middleBack = loadTrajectory("middleBack", r.onRedSide)
        upperOut = loadTrajectory("upper", r.onRedSide)
        upperBack = loadTrajectory("upperBack", r.onRedSide)
        lowerOut = loadTrajectory("lower", r.onRedSide)
        lowerBack = loadTrajectory("lowerBack", r.onRedSide)

        r.resetGyroAndOdomToPose(middleOut.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, middleOut, middleBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.scoreRing(r, lowerOut, lowerBack)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.scoreRing(r, upperOut, upperBack)

    @staticmethod
    def exitBackwards(r: 'robot.Robot'):
        traj = loadTrajectory("exit", r.onRedSide)
        r.resetGyroAndOdomToPose(traj.getInitialTargetHolonomicPose())
        yield from autoUtils.runPathUntilDone(r, traj)

    @staticmethod
    def shootAndGetFarMiddle(r: 'robot.Robot'):
        outTraj = loadTrajectory("far-middle", r.onRedSide)
        returnTraj = loadTrajectory("far-middle", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)

    @staticmethod
    def shootFromUpperSpeakerAndScoreUpperNote(r: 'robot.Robot'):
        outTraj = loadTrajectory("side-upper", r.onRedSide)
        returnTraj = loadTrajectory("side-upper-back", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)

    @staticmethod
    def shootFromUpperSpeakerAndScoreTwo(r: 'robot.Robot'):
        outTraj = loadTrajectory("side-upper", r.onRedSide)
        returnTraj = loadTrajectory("side-upper-back", r.onRedSide)
        farOutTraj = loadTrajectory("sideFar-upper-v02", r.onRedSide)
        farReturnTraj = loadTrajectory("sideFar-upper-back-v02", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)
        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.scoreRing(r, farOutTraj, farReturnTraj)

    @staticmethod
    def shootFromLowerSpeakerAndScoreLowerNote(r: 'robot.Robot'):
        outTraj = loadTrajectory("side-lower", r.onRedSide)
        returnTraj = loadTrajectory("side-lower-back", r.onRedSide)
        r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

        autoUtils.tryResetOdomWithLimelight(r, 0)
        yield from RobotAutos.shootStartingRing(r)
        yield from RobotAutos.scoreRing(r, outTraj, returnTraj)
