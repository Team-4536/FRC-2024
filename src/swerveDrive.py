import math

import robotHAL
from ntcore import NetworkTableInstance
from PIDController import PIDController
from real import angleWrap
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive():
    # meters, relative to robot center
    oneFtInMeters = 0.305
    modulePositions: list[Translation2d] = [
        Translation2d(oneFtInMeters, oneFtInMeters),
        Translation2d(oneFtInMeters, -oneFtInMeters),
        Translation2d(-oneFtInMeters, oneFtInMeters),
        Translation2d(-oneFtInMeters, -oneFtInMeters)
        ]

    def __init__(self, angle: Rotation2d, pose: Pose2d, wheelStates: list[SwerveModulePosition]) -> None:

        self.maxSpeed = 5.0 # meters per sec // we measured this its not BS
        self.maxSteerSpeed = 1.0 # CCW rads

        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        self.odometry = SwerveDrive4Odometry(self.kinematics, angle, tuple(wheelStates), pose) #type: ignore // because of tuple type mismatch, which is assert gaurded

        prefs = ["FL", "FR", "BL", "BR"]
        self.turningPIDs = [PIDController(prefs[i] + "Turning", 0.3) for i in range(4)]
        self.drivePIDs = [PIDController(prefs[i] + "Drive", 0.03, 0, 0, 0.2) for i in range(4)]

    def resetOdometry(self, pose: Pose2d, hal):
        wheelPositions = [SwerveModulePosition(hal.drivePositions[i], Rotation2d(hal.steeringPositions[i])) for i in range(4)]
        self.odometry.resetPosition(Rotation2d(hal.yaw), (wheelPositions[0], wheelPositions[1], wheelPositions[2], wheelPositions[3]), pose)

    # speed tuple is x (m/s), y (m/s), anglular speed (CCWR/s)
    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, speed: ChassisSpeeds):

        wheelPositions = [SwerveModulePosition(hal.drivePositions[i], Rotation2d(hal.steeringPositions[i])) for i in range(4)]
        targetStates = self.kinematics.toSwerveModuleStates(speed)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(targetStates, self.maxSpeed)
        # TODO: Why does the example discretize velocity?

        telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            state = self.optimizeTarget(targetStates[i], wheelPositions[i].angle)
            hal.driveSpeeds[i] = self.drivePIDs[i].tick(state.speed, hal.driveSpeedMeasured[i], dt)

            telemetryTable.putNumber(prefs[i] + "targetAngle", state.angle.radians())
            telemetryTable.putNumber(prefs[i] + "targetSpeed", state.speed)
            steeringError = angleWrap(state.angle.radians() - wheelPositions[i].angle.radians())
            hal.steeringSpeeds[i] = self.turningPIDs[i].tickErr(steeringError, state.angle.radians(), dt)

    def updateOdometry(self, hal: robotHAL.RobotHALBuffer):
        wheelPositions = [SwerveModulePosition(hal.drivePositions[i], Rotation2d(hal.steeringPositions[i])) for i in range(4)]
        self.odometry.update(Rotation2d(hal.yaw), (wheelPositions[0], wheelPositions[1], wheelPositions[2], wheelPositions[3]))

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