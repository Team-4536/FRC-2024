from numpy import newaxis
from inputs import nonLinearScaler
import robotHAL
import swerveDrive
import timing
import wpilib
from wpimath.geometry import Translation2d
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition


class RobotInputs():
    def __init__(self, drive: wpilib.XboxController, arm: wpilib.XboxController) -> None:
        self.driveX = nonLinearScaler(drive.getLeftX())
        self.driveY = nonLinearScaler(drive.getLeftY())
        self.turning = nonLinearScaler(drive.getRightX())
        self.absoluteDriveSwitch = drive.getXButton()
        self.gyroReset = drive.getYButton()

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.time = timing.TimeData(None)

        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.input: RobotInputs = RobotInputs(self.driveCtrlr, self.armCtrlr)

        self.telemetryTable = NetworkTableInstance.getDefault().getTable("telemetry")

        self.hal = robotHAL.RobotHALBuffer()
        self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal)
        self.absoluteDrive = -1
        # (Rob): I would have built the SwerveModulePosition class directly into the HALBuffer, if it weren't for the fact that python can't 'pickle' them. (???)
        self.drive = swerveDrive.SwerveDrive(Rotation2d(0), Pose2d(0, 0, 0),
            [SwerveModulePosition(self.hal.drivePositions[i], Rotation2d(self.hal.steeringPositions[i])) for i in range(4)])

    def robotPeriodic(self) -> None:
        self.time = timing.TimeData(self.time)
        self.hal.publish(self.telemetryTable)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.input = RobotInputs(self.driveCtrlr, self.armCtrlr)
        currentYaw = self.hal.yaw
        
        #newX = X*math.cos(-1*self.currentYaw)
        #newY = 
        if(self.input.absoluteDriveSwitch):
                self.absoluteDrive = self.absoluteDrive*-1
        if(self.input.gyroReset):
            self.hal.yaw = 0
            
        if(self.absoluteDrive == 1):
            originalVector = Translation2d(self.input.driveX,self.input.driveY)
            rotation = Rotation2d(currentYaw)
            newVector = originalVector.rotateBy(rotation)
            speed = ChassisSpeeds(newVector.X() * 0.1, newVector.Y() * 0.1, -self.input.turning * 0.1)
        else:
            speed = ChassisSpeeds(self.input.driveX * 0.1, self.input.driveY * 0.1, -self.input.turning * 0.1)

        self.drive.update(self.time.dt, self.hal, speed)

        self.hardware.update(self.hal)

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal)

if __name__ == "__main__":
    wpilib.run(Robot)