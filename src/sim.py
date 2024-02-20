from base64 import encode
from json import encoder
from numpy import emath
import wpilib
from wpilib import simulation
from wpilib.simulation import EncoderSim
import rev
import navx
from ntcore import NetworkTableInstance
import profiler
import navx
import ntcore
import rev
import wpilib
from phoenix6.hardware import CANcoder
import profiler




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

        self.shooterAngVelocityMeasured : float = 0

        self.camSpeed: float = 0
        self.camPos: float = 0

        self.lowerShooterLimitSwitch: bool = False
        self.upperShooterLimitSwitch: bool = False

        self.intakeSensor: bool = False
        self.shooterSensor: bool = False

        self.yaw: float = 0
       
    def resetEncoders(self) -> None:
        # swerve encoders
        for i in range(4):
            self.drivePositions[i] = 0
            self.steeringPositions[i] = 0

        self.shooterAimPos = 0
        self.camPos = 0

class RobotHAL():
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        fl = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        fr = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        bl = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        br = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)



        flEncoder = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless).get.CANEncoder()
        frEncoder = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless).get.CANEncoder()
        blEncoder = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless).get.CANEncoder()
        brEncoder = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless).get.CANEncoder()

        flEncoderSim = flEncoder.AnalogEncoderSim()
        frEncoderSim = frEncoder.AnalogEncoderSim()
        blEncoderSim = blEncoder.AnalogEncoderSim()
        brEncoderSim = brEncoder.AnalogEncoderSim()

        self.table.putNumber("telemetry/flencoder", flEncoderSim)
        self.table.putNumber("frEncoderSim.telemetry", frEncoderSim)
        self.table.putNumber("blEncoderSim.telemetry", blEncoderSim)
        

        self.steerMotors = [rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless),
                            rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
                           ]
        

        self.steerEncoders = [CANcoder(21), CANcoder(22), CANcoder(23), CANcoder(24)]

        flEncoder = self.steerEncoders.
        frEncoder = self.steerEncoders.
        blEncoder = self.steerEncoders.
        brEncoder = self.steerEncoders.

        flEncoderSim = flEncoder.Encoder.encoderSim(CANcoder(21))
        frEncoderSim = frEncoder.AnalogEncoderSim()
        blEncoderSim = blEncoder.AnalogEncoderSim()
        brEncoderSim = brEncoder.AnalogEncoderSim()
        