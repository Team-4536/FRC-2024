import copy
import math
import SimEncoders
import navx
import ntcore
from ntcore import NetworkTableInstance
import rev
import robot
import timing
from timing import TimeData
import wpilib

from phoenix6.hardware import CANcoder
import profiler


class RobotInputs():
    def __init__(self) -> None:
        

        self.driveX: float = 0.0
        self.driveY: float = 0.0
        self.turning: float = 0.0
        self.speedCtrl: float = 0.0
        self.gyroReset: bool = False
        self.brakeButton: bool = False
        self.absToggle: bool = False

        self.intake: bool = False

        self.shooterAimManual: float = 0.0
        # self.shooterFeedManual: float = 0.0
        # self.shooterSpeedManual: float = 0.0

        self.ampShot: bool = False
        self.podiumShot: bool = False
        self.subwooferShot: bool = False
        self.rev: bool = False
        self.shoot: bool = False

        self.camTemp: float = 0.0

class RobotSimHALBuffer():
    def __init__(self) -> None:
       

        self.camSpeed: float = 0
        self.camPos: float = 0
        self.prevTime = wpilib.getTime()
       
       
    
    def publish(self, table: ntcore.NetworkTable) -> None:
        # swerve modules
    
        table.putNumber("camSpeed", self.camSpeed)
        table.putNumber("camPos", self.camPos)

class RobotSimHAL():
    def __init__(self) -> None:
        self.prev = RobotSimHALBuffer()
      
        self.hhal = SimEncoders.RobotSimHALBuffer()
        self.input = RobotInputs()
        self.hhal.camSpeed = self.input.camTemp * 0.2
        

    def update(self, buf: RobotSimHALBuffer, dt) -> None:
        
        #self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table = wpilib.SmartDashboard
        
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        madeUpRadiansCam = buf.camSpeed / 3
        timePassed = dt
        MadeUpCamPos = madeUpRadiansCam * timePassed
        self.table.putNumber('CamPoseMadeUp', MadeUpCamPos)

       