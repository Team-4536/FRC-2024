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
        self.maxSpeed = 1.0 # meters per sec
        self.maxSteerSpeed = 1.0 # CCW rads
        # meters, relative to robot center
        # self.modulePositions = [ Translation2d(), Translation2d(), Translation2d(), Translation2d() ]
        self.turningPIDSettings: tuple[float, float, float] = (.01, 0.0, 0.0)

        # self.kinematics = SwerveDrive4Kinematics(*self.modulePositions)
        # assert(len(wheelStates) == 4)
        # self.odometry = SwerveDrive4Odometry(self.kinematics, angle, tuple(wheelStates), pose) #type: ignore // because of tuple type mismatch, which is assert gaurded
        self.turningPIDs = [PIDController(*self.turningPIDSettings) for i in range(4)]

    # speed tuple is x (m/s), y (m/s), anglular speed (CCWR/s)
    def update(self, dt: float, hal: robotHAL.RobotHALBuffer, targetState: SwerveModuleState):
        wheelPositions = [SwerveModulePosition(hal.drivePositions[i], Rotation2d(hal.steeringPositions[i])) for i in range(4)]
        # targetStates = self.kinematics.toSwerveModuleStates(targetSpeed)
        targetStates = [ targetState for i in range(4) ]
        # TODO: Why does the example discretize velocity?
        # SwerveDrive4Kinematics.desaturateWheelSpeeds(targetStates, self.maxSpeed)

        telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")
        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(1):
            state = SwerveModuleState.optimize(targetStates[i], wheelPositions[i].angle)
            telemetryTable.putNumber(prefs[i] + "targetAngle", state.angle.radians())
            telemetryTable.putNumber(prefs[i] + "currentAngle", wheelPositions[i].angle.radians())
            turnSpeed = self.turningPIDs[i].tick(state.angle.radians(), wheelPositions[i].angle.radians(), dt)
            # TODO: currently just does percent of max speed to motors, how to make accurate? is it?
            hal.driveSpeeds[i] = state.speed / self.maxSpeed
            hal.steeringSpeeds[i] = turnSpeed / self.maxSteerSpeed
            telemetryTable.putNumber(prefs[i] + "speed", turnSpeed)

        # TODO: fix/test this
        # self.odometry.update(Rotation2d(hal.yaw), (wheelPositions[0], wheelPositions[1], wheelPositions[2], wheelPositions[3]))


    # TODO: reset state function