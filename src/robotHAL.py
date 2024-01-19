import copy

import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
from phoenix6 import StatusCode
import math

class RobotHALBuffer():
    def __init__(self) -> None:
        self.driveSpeeds: list[float] = [0, 0, 0, 0] # -1 to 1 // volts to motor controller
        self.steeringSpeeds: list[float] = [0, 0, 0, 0] # -1 to 1
        self.drivePositions: list[float] = [0, 0, 0, 0] # in meters
        self.steeringPositions: list[float] = [0, 0, 0, 0] # in rads
        self.driveSpeedMeasured: list[float] = [0, 0, 0, 0] # m/s // output from encoders

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
            table.putNumber(prefs[i] + "DriveSpeedMeasured", self.driveSpeedMeasured[i])

        table.putNumber("yaw", self.yaw)

class RobotHAL():
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

        self.driveMotors = [rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
                           ]
        self.driveEncoders = [x.getEncoder() for x in self.driveMotors]
        self.driveMotors[1].setInverted(True)
        self.driveMotors[3].setInverted(True)

        self.steerMotors = [rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
                           ]
        for m in self.steerMotors:
            m.setInverted(True)

        self.steerEncoders = [CANcoder(21), CANcoder(22), CANcoder(23), CANcoder(24)]

        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)

        self.driveGearing: float = 6.12 # motor to wheel rotations
        self.wheelRadius: float = .05 # in meteres

    def update(self, buf: RobotHALBuffer) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        for m, s in zip(self.driveMotors, buf.driveSpeeds):
            m.set(s)

        for i in range(0, 4):
            e = self.driveEncoders[i]
            buf.drivePositions[i] = math.radians((e.getPosition() / self.driveGearing) * 360) * self.wheelRadius
            buf.driveSpeedMeasured[i] = math.radians((e.getVelocity() / self.driveGearing) * 360) * self.wheelRadius / 60

        for m, s in zip(self.steerMotors, buf.steeringSpeeds):
            m.set(s)

        for i in range(0, 4):
            e = self.steerEncoders[i]
            buf.steeringPositions[i] = math.radians(e.get_position().value_as_double * 360)

        if(buf.yaw != prev.yaw and abs(buf.yaw) < 0.01):
            self.gyro.reset()

        buf.yaw = math.radians(-self.gyro.getYaw())
