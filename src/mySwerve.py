import math
import wpilib
import numpy as np
from ntcore import NetworkTableInstance
from real import angleWrap
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import feetToMeters, radians


class SwerveDrive:
    def __init__(self) -> None:
        self.MAX_VOLTS = 4
        self.PID_CONSTRAINTS = TrapezoidProfileRadians.Constraints(
            4 * math.pi, 20 * math.pi
        )

        oneftInMeters = feetToMeters(1)

        FL_LOCATION = Translation2d(oneftInMeters, oneftInMeters)
        FR_LOCATION = Translation2d(oneftInMeters, -oneftInMeters)
        BL_LOCATION = Translation2d(-oneftInMeters, oneftInMeters)
        BR_LOCATION = Translation2d(-oneftInMeters, -oneftInMeters)
        self.kinematics = SwerveDrive4Kinematics(
            FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION
        )

    def update(self, ctrlr: wpilib.XboxController) -> None:

        robotVector = ChassisSpeeds(
            ctrlr.getLeftX(), ctrlr.getLeftY(), ctrlr.getRightX()
        )
