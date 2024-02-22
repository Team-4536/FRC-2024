import wpilib
import rev
from robotHAL import RobotHAL, RobotHALBuffer


class TestInputs():
    def __init__(self, mainCtrlr: wpilib.XboxController) -> None:
        self.motorSwitchDown = mainCtrlr.getLeftBumperPressed()
        self.motorSwitchUp = mainCtrlr.getRightBumperPressed()
        self.leftMainX = mainCtrlr.getLeftX()
        self.leftMainY = mainCtrlr.getLeftY()


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

        # variables===================================================
        self.motorAccessed = 0
        self.hal = RobotHALBuffer()

    def robotPeriodic(self) -> None:
        self.input = TestInputs(self.mainCtrlr)

    def teleopPeriodic(self) -> None:
        if (self.input.motorSwitchUp):
            self.motorAccessed += 1
        elif (self.input.motorSwitchDown):
            self.motorAccessed -= 1
        self.motorAccessed = self.motorAccessed % 8

        self.motors[self.motorAccessed].set(self.input.leftMainX * 0.15)
        # demobot code====================================
        # if(self.motorAccessed == 1):
        #     self.shooterLeft.set(self.input.leftMainY * 0.01)
        #     self.shooterRight.set(self.input.leftMainY * -0.01)
        self.hal.leds[0] = 255, 0, 0
        self.hal.leds[4] = 0, 255, 0
        self.hal.leds[5] = 200, 0, 255 #purple :D
        self.hal.leds[7] = 0, 0, 255
        self.hal.leds[1] = 200, 200, 200
        self.hal.leds[2] = 200, 200, 200
        self.hal.leds[5] = 200, 200, 200
        self.hal.leds[6] = 200, 200, 200




    def disabledPeriodic(self) -> None:

        for i in range(len(self.motors)):
            self.motors[i].set(0)

        for i in range(8):
            self.hal.leds[i] = 150, 110, 0


if __name__ == "__main__":
    wpilib.run(TestBox)
