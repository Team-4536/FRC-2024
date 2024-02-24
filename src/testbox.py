#from socket import CAN_BCM_RX_DELETE
import wpilib
import rev
from robotHAL import RobotHAL, RobotHALBuffer
from ntcore import NetworkTableInstance
import math
from utils import Scalar


class TestInputs():
    def __init__(self, mainCtrlr: wpilib.XboxController) -> None:
        self.controller = mainCtrlr

        self.motorSwitchDown: float = 0.0
        self.motorSwitchUp: float = 0.0

        self.driveX: float = 0.0
        self.driveY: float = 0.0

        self.testScalar = Scalar(0.1, 1)

    def update(self):
        self.leftMainXScaled = self.testScalar(self.controller.getLeftX())
        self.leftMainYScaled = self.testScalar(-self.controller.getLeftY())

        self.leftMainXRaw = self.controller.getLeftX()
        self.leftMainYRaw = -self.controller.getLeftY()

        self.motorSwitchDown = self.controller.getLeftBumperPressed()
        self.motorSwitchUp = self.controller.getRightBumperPressed()


class TestBox(wpilib.TimedRobot):
    def robotInit(self) -> None:
        # motors======================================================
        # BRUSHLESS = rev.CANSparkMax.MotorType.kBrushless
        BRUSHED = rev.CANSparkMax.MotorType.kBrushed

        # demobot shooter code (untested)=============================
        # self.shooterLeft = rev.CANSparkMax(1,BRUSHED)
        # self.shooterRight = rev.CANSparkMax(2,BRUSHED)

        self.motors: list[rev.CANSparkMax] = []
        for i in range(8):
            self.motors.append(rev.CANSparkMax(i, BRUSHED))

        # controllers=================================================
        self.mainCtrlr = wpilib.XboxController(0)
        self.input = TestInputs(self.mainCtrlr)

        # variables===================================================
        self.motorAccessed = 0
        self.hal = RobotHALBuffer()
        self.hardware = RobotHAL()

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def robotPeriodic(self) -> None:
        self.input.update()

        self.hardware.update(self.hal)

    def teleopPeriodic(self) -> None:
        if (self.input.motorSwitchUp):
            self.motorAccessed += 1
        elif (self.input.motorSwitchDown):
            self.motorAccessed -= 1
        self.motorAccessed = self.motorAccessed % 8

        self.motors[self.motorAccessed].set(self.input.leftMainXScaled * 0.15)
        # demobot code====================================
        # if(self.motorAccessed == 1):
        #     self.shooterLeft.set(self.input.leftMainY * 0.01)
        #     self.shooterRight.set(self.input.leftMainY * -0.01)
        """
        self.hal.leds[0] = 255, 0, 0
        self.hal.leds[4] = 0, 255, 0
        self.hal.leds[5] = 200, 0, 255
        self.hal.leds[7] = 0, 0, 255
        self.hal.leds[1] = 200, 200, 200
        self.hal.leds[2] = 200, 200, 200
        self.hal.leds[5] = 200, 200, 200
        self.hal.leds[6] = 200, 200, 200
        """

        for i in range(8):
            self.hal.leds[i] = 0, 0, 0

        if self.input.leftMainXRaw == 0 and self.input.leftMainYRaw == 0: #no input
            self.hal.leds[i] = 0, 0, 0
        elif self.input.leftMainXRaw > 0 and self.input.leftMainYRaw > 0: #q1
            self.hal.leds[2] = 0, 150, 0
        elif self.input.leftMainXRaw < 0 and self.input.leftMainYRaw > 0: #q2
            self.hal.leds[1] = 0, 150, 0
        elif self.input.leftMainXRaw > 0 and self.input.leftMainYRaw < 0: #q4
            self.hal.leds[5] = 0, 150, 0
        elif self.input.leftMainXRaw < 0 and self.input.leftMainYRaw < 0: #q3
            self.hal.leds[6] = 0, 150, 0






    def disabledPeriodic(self) -> None:

        for i in range(len(self.motors)):
            self.motors[i].set(0)
        
        for i in range(8):
            self.hal.leds[i] = (210, 20, 0)
        

        #self.hardware.ledController.setLEDs(255, 0, 0, 0, 0, 8)

if __name__ == "__main__":
    wpilib.run(TestBox)
