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


# adapted from here: https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/Drivetrain.java
class SwerveDrive():
    def __init__(self, angle: Rotation2d, pose: Pose2d, wheelStates: list[SwerveModulePosition]) -> None:
        # TODO: MAKE THESE ACCURATE BEFORE USING ON BOT
        self.maxSpeed = 1 # meters per sec
        self.maxTurnSpeed = 1 # CCW rads
        self.wheelSize = 1 # metres
        # meters, relative to robot center
        self.modulePositions = [ Translation2d(), Translation2d(), Translation2d(), Translation2d() ]
        self.turningPIDSettings: tuple[float, float, float] = (.001, 0.0, 0.0)

        self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        assert(len(wheelStates) == 4)
        self.odometry = SwerveDrive4Odometry(self.kinematics, angle, tuple(wheelStates), pose) #type: ignore // because of tuple type mismatch, which is assert gaurded
        self.turningPIDs = [PIDController(*self.turningPIDSettings) for i in range(4)]

    # speed tuple is x (m/s), y (m/s), anglular speed (CCWR/s)
    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, targetSpeed: ChassisSpeeds):
        wheelPositions = [SwerveModulePosition(hal.drivePositions[i], Rotation2d(hal.steeringPositions[i])) for i in range(4)]
        targetStates = self.kinematics.toSwerveModuleStates(targetSpeed)
        # TODO: Why does the example discretize velocity?
        SwerveDrive4Kinematics.desaturateWheelSpeeds(targetStates, self.maxSpeed)

        telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(4):
            state = SwerveModuleState.optimize(targetStates[i], wheelPositions[i].angle)
            turnSpeed = self.turningPIDs[i].tick(targetStates[i].angle.radians(), state.angle.radians(), dt) # type: ignore // Rotation2d.radians() has messed up type annotations
            # TODO: currently just does percent of max speed to motors, how to make accurate? is it?
            hal.driveSpeeds[i] = targetStates[i].speed / self.maxSpeed
            hal.steeringSpeeds[i] = turnSpeed / self.maxTurnSpeed
            telemetryTable.putNumber(prefs[i] + "target", targetStates[i].angle.radians()) # type: ignore // radians >:(

        # TODO: fix/test this
        self.odometry.update(Rotation2d(hal.yaw), *wheelPositions)


    # TODO: reset state function