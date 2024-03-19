import copy
import math

import navx
import ntcore
import profiler
import rev
import wpilib
from phoenix5.led import CANdle
from phoenix6.hardware import CANcoder
from timing import TimeData


class RobotHALBuffer():
    def __init__(self) -> None:
        self.driveSpeeds: list[float] = [0, 0, 0, 0] # -1 to 1 // volts to motor controller
        self.steeringSpeeds: list[float] = [0, 0, 0, 0] # -1 to 1
        self.drivePositions: list[float] = [0, 0, 0, 0] # in meters
        self.driveSpeedMeasured: list[float] = [0, 0, 0, 0] # m/s // output from encoders
        self.steeringPositions: list[float] = [0, 0, 0, 0] # in CCW rads

        self.intakeSpeeds: list[float] = [0, 0] # -1 to 1 // volts to motor controller
        # self.intakePositions: list[float] = [0, 0] # whatever encoders return

        self.shooterSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterAimSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterIntakeSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterAimPos: float = 0 # rads out from resting position

        self.shooterAngVelocityMeasured : float = 0

        self.camSpeed: float = 0
        self.camPos: float = 0


        self.climberSpeed: float = 0.0 # -1 to 1 volts, climbing up is -
        self.climberLimitPressed: bool = False
        self.climbCurrent = 0.0
        self.climbTemp = 0.0
        self.climbPos = 0.0

        self.lowerShooterLimitSwitch: bool = False
        self.upperShooterLimitSwitch: bool = False

        self.intakeSensor: bool = False
        self.shooterSensor: bool = False

        self.intakeSpeeds: list[float] = [0, 0] # -1 to 1 // volts to motor controller
        # self.intakePositions: list[float] = [0, 0] # whatever encoders return

        self.shooterSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterAimSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterIntakeSpeed: float = 0 # -1 to 1 // volts to motor controller
        self.shooterAimPos: float = 0 # rads out from resting position

        self.shooterAngVelocityMeasured : float = 0

        self.camSpeed: float = 0
        self.camPos: float = 0

        self.lowerShooterLimitSwitch: bool = False
        self.upperShooterLimitSwitch: bool = False

        self.intakeSensor: bool = False
        self.shooterSensor: bool = False

        self.yaw: float = 0
        self.leds: list[tuple] = [(0, 0, 0)] * 8

        self.debugBool: bool = False

    def resetEncoders(self) -> None:
        # swerve encoders
        for i in range(4):
            self.drivePositions[i] = 0
            self.steeringPositions[i] = 0

        self.shooterAimPos = 0
        self.camPos = 0

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

        self.camSpeed = 0

        self.climberSpeed = 0.0

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

        table.putNumber("ShooterAngVelMeasured", self.shooterAngVelocityMeasured)

        table.putBoolean("LowerShooterLimitSwitch", self.lowerShooterLimitSwitch)
        table.putBoolean("UpperShooterLimitSwitch", self.upperShooterLimitSwitch)

        table.putBoolean("IntakeSensor", self.intakeSensor)
        table.putBoolean("ShooterSensor", self.shooterSensor)

        table.putNumber("camSpeed", self.camSpeed)
        table.putNumber("camPos", self.camPos)

        table.putNumber("climberSpeed", self.climberSpeed)

        table.putBoolean("climberLimit", self.climberLimitPressed)
        table.putNumber("climberCurrent", self.climbCurrent)
        table.putNumber("climberTemp", self.climbTemp)

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

        # self.intakeEncoders = [c.getEncoder() for c in self.intakeMotors]
        # for k in self.intakeMotors:
        #     k.setSmartCurrentLimit(30)

        # shooter motors
        self.shooterTopMotor = rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless)
        self.shooterTopMotor.setInverted(True)
        self.shooterBottomMotor = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless) # motor on follower
        self.shooterAimMotor = rev.CANSparkMax(14, rev.CANSparkMax.MotorType.kBrushless)
        self.shooterAimMotor.setInverted(True)
        self.shooterIntakeMotor = rev.CANSparkMax(13, rev.CANSparkMax.MotorType.kBrushless)
        # shooter encoders
        self.shooterTopEncoder = self.shooterTopMotor.getEncoder()
        self.shooterBottomEncoder = self.shooterBottomMotor.getEncoder()
        self.shooterAimEncoder = self.shooterAimMotor.getEncoder()
        self.shooterAimEncoder.setPosition(0)
        self.shooterIntakeEncoder = self.shooterIntakeMotor.getEncoder()

        self.camMotor = rev.CANSparkMax(15, rev.CANSparkMax.MotorType.kBrushless)
        self.camEncoder = self.camMotor.getEncoder()
        self.camEncoder.setPosition(0)

        self.climbingMotor = rev.CANSparkMax(16, rev.CANSparkMax.MotorType.kBrushless)
        self.climbEncoder = self.climbingMotor.getEncoder()
        self.climbEncoder.setPosition(0)

        self.climbingMotor.setInverted(False)
        self.climbSensor = self.climbingMotor.getReverseLimitSwitch(rev.SparkLimitSwitch.Type.kNormallyOpen)

        # other
        self.gyro = navx.AHRS(wpilib.SPI.Port.kMXP)

        self.lowerShooterLimitSwitch = wpilib.DigitalInput(3)
        self.upperShooterLimitSwitch = wpilib.DigitalInput(1)

        self.intakeSensor = wpilib.DigitalInput(0)
        self.I2C = wpilib.I2C.Port.kOnboard
        self.shooterSensor = wpilib.DigitalInput(2)
        # self.colorSensor = rev.ColorSensorV3(self.I2C)

        self.driveGearing: float = 6.12 # motor to wheel rotations
        self.wheelRadius: float = .05 # in meteres

        self.ledController: CANdle = CANdle(20)

    # angle expected in CCW rads
    def resetGyroToAngle(self, ang: float) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(-math.degrees(ang))

    def resetCamEncoderPos(self, nPos: float) -> None:
        self.camEncoder.setPosition(nPos)

    def resetAimEncoderPos(self, nPos: float) -> None:
        self.shooterAimEncoder.setPosition(nPos)

    def setLEDs(self, r: int, g: int, b: int, w: int = 0, startIdx: int = 0, count: int = 512) -> None:
        self.ledController.setLEDs(r, g, b, w, startIdx, count)

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        profiler.start()
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
        profiler.end("drive updates")

        profiler.start()
        for m, s in zip(self.intakeMotors, buf.intakeSpeeds):
            m.set(s)
        profiler.end("steer updates")

        # for i in range(0, 2):
        #     e = self.intakeEncoders[i]
        #     buf.intakePositions[i] = e.getPosition()

        profiler.start()
        self.shooterTopMotor.set(buf.shooterSpeed) # bottom shooter motor is on follower mode
        self.shooterAimMotor.set(buf.shooterAimSpeed)
        self.shooterIntakeMotor.set(buf.shooterIntakeSpeed)

        buf.shooterAngVelocityMeasured = (self.shooterTopEncoder.getVelocity()/60)*math.pi*2
        buf.shooterAimPos = self.shooterAimEncoder.getPosition() * math.pi * 2 / 45

        self.camMotor.set(buf.camSpeed)
        buf.camPos = self.camEncoder.getPosition() * math.pi * 2 / 4

        self.climbingMotor.set(buf.climberSpeed)
        buf.climbPos = self.climbEncoder.getPosition()

        buf.climberLimitPressed = self.climbSensor.get()
        buf.climbCurrent = self.climbingMotor.getOutputCurrent()
        buf.climbTemp = self.climbingMotor.getMotorTemperature()


        profiler.end("other motor encoder updates")

        profiler.start()
        if(buf.yaw != prev.yaw and abs(buf.yaw) < 0.01):
            self.gyro.reset()
        buf.yaw = math.radians(-self.gyro.getAngle())
        profiler.end("gyro updates")

        profiler.start()
        buf.lowerShooterLimitSwitch = self.lowerShooterLimitSwitch.get()
        buf.upperShooterLimitSwitch = self.upperShooterLimitSwitch.get()
        profiler.end("switch updates")

        profiler.start()

        # ntcore.NetworkTableInstance.getDefault().getTable("telemetry").putNumber("colorProx", self.colorSensor.getProximity())
        # if self.colorSensor.getProximity() >= 400:
        #     buf.shooterSensor = True
        # else:
        #     buf.shooterSensor = False

        buf.intakeSensor = self.intakeSensor.get()
        buf.shooterSensor = self.shooterSensor.get()
        profiler.end("sensor updates")
