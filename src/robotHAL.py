import copy
import math

import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder


class RobotHALBuffer():
    def __init__(self) -> None:
        self.driveSpeeds: list[float] = [0, 0, 0, 0] # -1 to 1 // volts to motor controller
        self.steeringSpeeds: list[float] = [0, 0, 0, 0] # -1 to 1
        self.drivePositions: list[float] = [0, 0, 0, 0] # in meters
        self.steeringPositions: list[float] = [0, 0, 0, 0] # in CCW rads
        self.driveSpeedMeasured: list[float] = [0, 0, 0, 0] # m/s // output from encoders

        self.intakeSpeeds: list[float] = [0, 0] # -1 to 1 // volts to motor controller
        # self.intakePositions: list[float] = [0, 0] # whatever encoders return

        self.shooterSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterAimSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterIntakeSpeed: float = 0 # -1 to 1 // volts to motor controller

        self.shooterAimPos: float = 0 # rads out from resting position

        self.lowerShooterLimitSwitch: bool = False
        self.upperShooterLimitSwitch: bool = False

        self.intakeSensor: bool = False
        self.colorSensorProx: bool = False

        self.yaw: float = 0

    def resetEncoders(self) -> None:
        # swerve encoders
        for i in range(4):
            self.drivePositions[i] = 0
            self.steeringPositions[i] = 0

        self.shooterAimPos = 0

    def stopMotors(self) -> None:
        # swerve motors
        for i in range(4):
            self.driveSpeeds[i] = 0
            self.steeringSpeeds[i] = 0

        self.intakeSpeeds[0] = 0
        self.intakeSpeeds[1] = 0

        # shooter motors
        self.shooterSpeed = 0
        self.shooterAimSpeed = 0
        self.shooterIntakeSpeed = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        # swerve modules
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            table.putNumber(prefs[i] + "DriveSpeed", self.driveSpeeds[i])
            table.putNumber(prefs[i] + "SteerSpeed", self.steeringSpeeds[i])
            table.putNumber(prefs[i] + "DrivePos", self.drivePositions[i])
            table.putNumber(prefs[i] + "SteerPos", self.steeringPositions[i])
            table.putNumber(prefs[i] + "DriveSpeedMeasured", self.driveSpeedMeasured[i])

        table.putNumber("GreenIntakeSpeed", self.intakeSpeeds[0])
        table.putNumber("BlueIntakeSpeed", self.intakeSpeeds[1])
        # table.putNumber("GreenIntakeEncoder", self.intakePositions[0])
        # table.putNumber("BlueIntakeEncoder", self.intakePositions[1])

        # shooter motors
        table.putNumber("ShooterSpeed", self.shooterSpeed)
        table.putNumber("ShooterAimSpeed", self.shooterAimSpeed)
        table.putNumber("ShooterIntakeSpeed", self.shooterIntakeSpeed)

        table.putNumber("ShooterAimPos", self.shooterAimPos)

        table.putBoolean("LowerShooterLimitSwitch", self.lowerShooterLimitSwitch)
        table.putBoolean("UpperShooterLimitSwitch", self.upperShooterLimitSwitch)

        table.putBoolean("IntakeSensor", self.intakeSensor)
        table.putBoolean("ShooterProxSensor", self.colorSensorProx)

        # gyro
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
        for d in self.driveMotors:
            d.setSmartCurrentLimit(40)

        self.steerMotors = [rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
                           ]
        for m in self.steerMotors:
            m.setInverted(True)
            m.setOpenLoopRampRate(50)
            m.setSmartCurrentLimit(40)

        self.steerEncoders = [CANcoder(21), CANcoder(22), CANcoder(23), CANcoder(24)]

        # intake motors and encoders
        self.intakeMotors = [rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless),
                             rev.CANSparkMax(10, rev.CANSparkMax.MotorType.kBrushless)]
        self.intakeMotors[1].setInverted(True)

        self.intakeEncoders = [c.getEncoder() for c in self.intakeMotors]
        for k in self.intakeMotors:
            k.setSmartCurrentLimit(30)

        # shooter motors
        self.shooterTopMotor = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless)
        self.shooterBottomMotor = rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless) # motor on follower
        self.shooterAimMotor = rev.CANSparkMax(14, rev.CANSparkMax.MotorType.kBrushless)
        self.shooterIntakeMotor = rev.CANSparkMax(13, rev.CANSparkMax.MotorType.kBrushless)
        # shooter encoders
        self.shooterTopEncoder = self.shooterTopMotor.getEncoder()
        self.shooterBottomEncoder = self.shooterBottomMotor.getEncoder()
        self.shooterAimEncoder = self.shooterAimMotor.getEncoder()
        self.shooterIntakeEncoder = self.shooterIntakeMotor.getEncoder()

        # other
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)

        self.lowerShooterLimitSwitch = wpilib.DigitalInput(2)
        self.upperShooterLimitSwitch = wpilib.DigitalInput(3)

        self.intakeSensor = wpilib.DigitalInput(0)
        self.I2C = wpilib.I2C.Port.kOnboard
        self.colorSensor = rev.ColorSensorV3(self.I2C)

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

        for m, s in zip(self.intakeMotors, buf.intakeSpeeds):
            m.set(s)

        # for i in range(0, 2):
        #     e = self.intakeEncoders[i]
        #     buf.intakePositions[i] = e.getPosition()

        # shooter motors speeds
        self.shooterTopMotor.set(buf.shooterSpeed) # bottom shooter motor is on follower mode
        self.shooterAimMotor.set(buf.shooterAimSpeed)
        self.shooterIntakeMotor.set(buf.shooterIntakeSpeed)
        # get shooter encoder values
        self.shooterTopAngularVelocityMeasured = (self.shooterTopEncoder.getVelocity()/60)*math.pi*2
        self.shooterBottomAngularVelocityMeasured = (self.shooterBottomEncoder.getVelocity()/60)*math.pi*2
        self.shooterAimPos = self.shooterAimEncoder.getPosition()

        if(buf.yaw != prev.yaw and abs(buf.yaw) < 0.01):
            self.gyro.reset()

        buf.yaw = math.radians(-self.gyro.getYaw())

        buf.lowerShooterLimitSwitch = self.lowerShooterLimitSwitch.get()
        buf.upperShooterLimitSwitch = self.upperShooterLimitSwitch.get()

        if self.colorSensor.getProximity() > 1950:
            buf.colorSensorProx = True
        else:
            buf.colorSensorProx = False
        buf.intakeSensor = self.intakeSensor.get()
