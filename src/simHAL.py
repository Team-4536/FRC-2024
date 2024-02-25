import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
from wpimath.geometry import Rotation2d, Translation2d


class RobotSimHAL():
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.drivePositions = [0.0, 0.0, 0.0, 0.0]
        self.driveVels = [0.0, 0.0, 0.0, 0.0]
        self.steerEncoderPositions = [0.0, 0.0, 0.0, 0.0]
        self.steerVels = [0.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0

        self.table = NetworkTableInstance.getDefault().getTable("sim")

    def update(self, buf: RobotHALBuffer, dt: float) -> None:
        # prev = self.prev
        self.prev = copy.deepcopy(buf)

        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            self.driveVels[i] = lerp(self.driveVels[i], buf.driveSpeeds[i] * 1/0.2, 0.2)
            self.steerVels[i] = lerp(self.steerVels[i], buf.steeringSpeeds[i] * 20, 0.3)
            self.table.putNumber(prefs[i] + "SimSteerVel", self.steerVels[i])
            self.table.putNumber(prefs[i] + "SimDriveVel", self.driveVels[i])

        angleDeltaSum = 0.0
        for i in range(4):
            buf.steeringPositions[i] += self.steerVels[i] * dt
            buf.driveSpeedMeasured[i] = self.driveVels[i]

            driveDist = self.driveVels[i] * dt
            buf.drivePositions[i] += driveDist

            new = Translation2d(driveDist, 0)
            new = new.rotateBy(Rotation2d(buf.steeringPositions[i]))
            new += SwerveDrive.modulePositions[i]
            old = SwerveDrive.modulePositions[i]
            delta = angleWrap(math.atan2(new.y, new.x) - math.atan2(old.y, old.x))
            self.table.putNumber(prefs[i] + "NAngle", -math.degrees(math.atan2(new.y, new.x)))
            self.table.putNumber(prefs[i] + "OAngle", -math.degrees(math.atan2(old.y, old.x)))
            self.table.putNumber(prefs[i] + "AngleDelta", -math.degrees(delta))
            angleDeltaSum += delta

        self.table.putNumber("angleDeltaAvg", angleDeltaSum / 4)
        self.yaw += angleDeltaSum / 4
        buf.yaw = self.yaw

    # TODO: gyro yaw sim
    def resetGyroToAngle(self, ang: float) -> None:
        self.yaw = ang

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def resetAimEncoderPos(self, nPos: float) -> None:
        pass

    # TODO: test to verify that this and the actual hal have the same signature
