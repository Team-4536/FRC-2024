import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from swerveDrive import SwerveDrive
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d


class RobotSimHAL():
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.drivePositions = [0.0, 0.0, 0.0, 0.0]
        self.driveVels = [0.0, 0.0, 0.0, 0.0]
        self.steerEncoderPositions = [0.0, 0.0, 0.0, 0.0]
        self.steerVels = [0.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0

        self.shooterAngVel = 0.0
        self.shooterAimVel = 0.0
        self.shooterAimPos = 0.0

        self.table = NetworkTableInstance.getDefault().getTable("sim")

        self.ringPos = 0
        self.ringTransitionStart = -1

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        # prev = self.prev
        self.prev = copy.deepcopy(buf)

        # UPDATE WHEEL VELOCITIES
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            self.driveVels[i] = lerp(self.driveVels[i], buf.driveVolts[i] * 1/0.2, 0.2)
            self.steerVels[i] = lerp(self.steerVels[i], buf.steeringVolts[i] * 20, 0.3)
            self.table.putNumber(prefs[i] + "SimSteerVel", self.steerVels[i])
            self.table.putNumber(prefs[i] + "SimDriveVel", self.driveVels[i])

        # UPDATE HAL BUFFER WITH WHEEL DATA, CALCULATE YAW DELTA
        angleDeltaSum = 0.0
        for i in range(4):
            buf.steeringPositions[i] += self.steerVels[i] * time.dt
            buf.driveSpeedMeasured[i] = self.driveVels[i]

            driveDist = self.driveVels[i] * time.dt
            buf.drivePositions[i] += driveDist

            new = Translation2d(driveDist, 0)
            new = new.rotateBy(Rotation2d(buf.steeringPositions[i]))
            new += SwerveDrive.modulePositions[i]
            old = SwerveDrive.modulePositions[i]
            delta = angleWrap(math.atan2(new.y, new.x) - math.atan2(old.y, old.x))
            angleDeltaSum += delta
        self.yaw += angleDeltaSum / 4
        buf.yaw = self.yaw

        # SIMULATE RING POSITION, TRANSITION, AND SENSOR READINGS
        if self.ringPos == 0:
            buf.intakeSensor = False
            buf.shooterSensor = False
            if buf.intakeSpeeds[0] > 0.001:

                if self.ringTransitionStart == -1:

                    self.ringTransitionStart = time.timeSinceInit
                else:
                    if (time.timeSinceInit - self.ringTransitionStart) > 0.4:
                        self.ringPos = 1
                        self.ringTransitionStart = -1
            else:
                self.ringTransitionStart = -1
        elif self.ringPos == 1:
            buf.intakeSensor = True
            buf.shooterSensor = False
            if buf.shooterIntakeSpeed > 0.001:

                if self.ringTransitionStart == -1:

                    self.ringTransitionStart = time.timeSinceInit
                else:
                    if (time.timeSinceInit - self.ringTransitionStart) > 0.4:
                        self.ringPos = 2
                        self.ringTransitionStart = -1
            else:
                self.ringTransitionStart = -1
        elif self.ringPos == 2:
            buf.intakeSensor = False
            buf.shooterSensor = True
            if buf.shooterIntakeSpeed > 0.001:

                if self.ringTransitionStart == -1:

                    self.ringTransitionStart = time.timeSinceInit
                else:
                    if (time.timeSinceInit - self.ringTransitionStart) > 0.2:
                        self.ringPos = 0
                        self.ringTransitionStart = -1
            else:
                self.ringTransitionStart = -1
        self.table.putNumber("ringPos", self.ringPos)
        self.table.putNumber("ringTransitionStart", self.ringTransitionStart)

        # SHOOTER VELOCITY, ARM POSITION
        self.shooterAngVel = lerp(self.shooterAngVel, buf.shooterSpeed * 1/0.00181, 0.4)
        self.shooterAimVel = lerp(self.shooterAimVel, buf.shooterAimSpeed * 1, 0.1)
        self.shooterAimVel -= 0.005 * math.cos(self.shooterAimPos + 0.1)

        self.shooterAimPos += self.shooterAimVel
        self.shooterAimPos = max(0, min(self.shooterAimPos, 1.9))

        buf.shooterAimPos = self.shooterAimPos
        buf.shooterAngVelocityMeasured = self.shooterAngVel

        # TODO: cam sim


    # TODO: gyro yaw sim
    def resetGyroToAngle(self, ang: float) -> None:
        self.yaw = ang

    def resetCamEncoderPos(self, nPos: float) -> None:
        pass

    def resetClimbEncoderPos(self, nPos: float) -> None:
        pass
    
    def resetAimEncoderPos(self, nPos: float) -> None:
        pass

    def setLEDs(self, r: int, g: int, b: int, w: int = 0, startIdx: int = 0, count: int = 0) -> None:
        pass

    # TODO: test to verify that this and the actual hal have the same signature
