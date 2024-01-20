from ast import mod

from phoenix6 import rotation
from real import angleWrap
import robotHAL
from ntcore import NetworkTableInstance
from PIDController import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
import math


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive():
    def __init__(self, angle: Rotation2d, pose: Pose2d, wheelStates: list[SwerveModulePosition]) -> None:
        self.maxSpeed = 5.0 # meters per sec // we measured this its not BS
        self.maxSteerSpeed = 1.0 # CCW rads
        # meters, relative to robot center
        oneFtInMeters = 0.305
        self.modulePositions = [
            Translation2d(oneFtInMeters, oneFtInMeters),
            Translation2d(oneFtInMeters, -oneFtInMeters),
            Translation2d(-oneFtInMeters, oneFtInMeters),
            Translation2d(-oneFtInMeters, -oneFtInMeters)
            ]

        self.table = NetworkTableInstance.getDefault().getTable("Swerve settings")
        self.table.putNumber("SteeringKp", .4)
        self.table.putNumber("DriveKp", 0.03)
        self.table.putNumber("DriveKff", 0.2)

        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        # assert(len(wheelStates) == 4)
        # self.odometry = SwerveDrive4Odometry(self.kinematics, angle, tuple(wheelStates), pose) #type: ignore // because of tuple type mismatch, which is assert gaurded
        self.turningPIDs = [PIDController(0, 0, 0) for i in range(4)]
        self.drivePIDs = [PIDController(0, 0, 0) for i in range(4)]

    # speed tuple is x (m/s), y (m/s), anglular speed (CCWR/s)
    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, speed: ChassisSpeeds):
        steerKp = self.table.getNumber("SteeringKp", 0.0)
        driveKp = self.table.getNumber("DriveKp", 0.0)
        driveKff = self.table.getNumber("DriveKff", 0.0)
        for i in range(4):
            self.turningPIDs[i].kp = steerKp
            self.drivePIDs[i].kp = driveKp
            self.drivePIDs[i].kff = driveKff

        wheelPositions = [SwerveModulePosition(hal.drivePositions[i], Rotation2d(hal.steeringPositions[i])) for i in range(4)]
        targetStates = self.kinematics.toSwerveModuleStates(speed)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(targetStates, self.maxSpeed)
        # TODO: Why does the example discretize velocity?

        telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            # state = SwerveModuleState.optimize(targetStates[i], wheelPositions[i].angle)
            state = self.optimizeTarget(targetStates[i], wheelPositions[i].angle)
            telemetryTable.putNumber(prefs[i] + "targetAngle", state.angle.radians())
            telemetryTable.putNumber(prefs[i] + "targetSpeed", state.speed)
            hal.driveSpeeds[i] = self.drivePIDs[i].tick(state.speed, hal.driveSpeedMeasured[i], dt)

            steeringError = angleWrap(state.angle.radians() - wheelPositions[i].angle.radians())

            # hal.steeringSpeeds[i] = self.turningPIDs[i].tick(state.angle.radians(), wheelPositions[i].angle.radians(), dt)
            hal.steeringSpeeds[i] = self.turningPIDs[i].tickErr(steeringError, state.angle.radians(), dt)

        # TODO: fix/test this
        # self.odometry.update(Rotation2d(hal.yaw), (wheelPositions[0], wheelPositions[1], wheelPositions[2], wheelPositions[3]))


    def optimizeTarget(self, target: SwerveModuleState, moduleAngle: Rotation2d) -> SwerveModuleState:

        error = angleWrap(target.angle.radians() - moduleAngle.radians())

        outputSpeed = target.speed
        outputAngle = target.angle.radians()

        # optimize
        if abs(error) > math.pi / 2:
            outputAngle = outputAngle + math.pi
            outputSpeed = -outputSpeed

        # return
        outputAngleRot2d = Rotation2d(angleWrap(outputAngle))
        output = SwerveModuleState(outputSpeed, outputAngleRot2d)

        return output

    # TODO: reset state function