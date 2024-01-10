import copy

import navx
import ntcore
import rev
import wpilib
from ctre.sensors import CANCoder


class RobotHALBuffer():
    def __init__(self) -> None:
        self.driveSpeeds: list[float] = [0, 0, 0, 0]
        self.steeringSpeeds: list[float] = [0, 0, 0, 0]
        self.drivePositions: list[float] = [0, 0, 0, 0]
        self.steeringPositions: list[float] = [0, 0, 0, 0]

        self.yaw: float = 0

    def resetEncoders(self) -> None:
        for i in range(4):
            self.drivePositions[i] = 0
            self.steeringPositions[i] = 0

    def stopMotors(self) -> None:
        for i in range(4):
            self.driveSpeeds[i] = 0
            self.steeringSpeeds[i] = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            table.putNumber(prefs[i] + "DriveSpeed", self.driveSpeeds[i])
            table.putNumber(prefs[i] + "SteerSpeed", self.steeringSpeeds[i])
            table.putNumber(prefs[i] + "DrivePos", self.drivePositions[i])
            table.putNumber(prefs[i] + "SteerPos", self.steeringPositions[i])

class RobotHAL():
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.driveMotors = [rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
                           ]
        self.driveEncoders = [x.getEncoder() for x in self.driveMotors]

        self.steerMotors = [rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
                           ]
        self.steerEncoders = [CANCoder(21), CANCoder(22), CANCoder(23), CANCoder(24) ]

        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)
        self.yawCenter: float = 0

    def update(self, buf: RobotHALBuffer) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        for m, s in zip(self.driveMotors, buf.driveSpeeds):
            m.set(s)
        for i in range(0, 4):
            e = self.driveEncoders[i]
            if(buf.drivePositions[i] != prev.drivePositions[i]):
                e.setPosition(buf.drivePositions[i])
            buf.drivePositions[i] = e.getPosition()

        for m, s in zip(self.steerMotors, buf.steeringSpeeds):
            m.set(s)
        for i in range(0, 4):
            e = self.steerEncoders[i]
            if(buf.steeringPositions[i] != prev.steeringPositions[i]):
                e.setPosition(buf.steeringPositions[i])
            buf.steeringPositions[i] = e.getPosition()

        if(buf.yaw != prev.yaw):
            self.yawCenter = buf.yaw
        buf.yaw = self.gyro.getYaw() - self.yawCenter
