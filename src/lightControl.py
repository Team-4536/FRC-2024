import math
from time import time
import ntcore
from phoenix5.led import CANdle  #only use for LEDs
from timing import TimeData
import robotHAL
from simHAL import RobotSimHAL
import wpilib

LIGHTS_OFF = "off"
LIGHTS_ON = "on"

class LightControl():
    def __init__(self, hardware) -> None:
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        
        self.hardware = hardware
        
        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware.update(self.hal, self.time)
        
        self.totalLights = 8
        self.numHeaderLights = 1

        self.LEDAnimationFrame = 0
        self.LEDFlashTimer = 0.0
        self.LEDTrigger = False

        self.intakeLEDTrigger = False
        self.climberLEDTrigger = False
        self.lightTriggerBool = False
        self.lightTriggerBool2 = False
        
        self.LEDPrevTrigger = False
        self.intakeLEDPrevTrigger = False
        self.climberLEDPrevTrigger = False
        self.lightTestLEDPrevTrigger = False
        self.lightTestLEDPrevTrigger2 = False
        
        self.lightTestBool = False
        self.funnyTestNumber = 0
        self.funnyTestNumber-=10000
        
        self.lightToggle = wpilib.SendableChooser()
        self.lightToggle.setDefaultOption(LIGHTS_OFF, LIGHTS_OFF)
        self.lightToggle.addOption(LIGHTS_ON, LIGHTS_ON)
        
        self.lightToggle2 = wpilib.SendableChooser()
        self.lightToggle2.setDefaultOption(LIGHTS_OFF, LIGHTS_OFF)
        self.lightToggle2.addOption(LIGHTS_ON, LIGHTS_ON)
        
        self.red = 0
        self.green = 0
        self.blue = 0
        
        self.duration = 0
        
        wpilib.SmartDashboard.putData('light toggle', self.lightToggle)
        wpilib.SmartDashboard.putData('light toggle 2', self.lightToggle2)

    def setLights(self, leds: CANdle, yaw: float):
        #leds.setLEDs(0, 0, 0, 0, 0, totalLights)

        while yaw < 2 * math.pi:
            yaw += 2 * math.pi
        while yaw > 2 * math.pi:
            yaw -= 2 * math.pi

        headerLight: int = (int)((yaw / (2 * math.pi)) * self.totalLights)
        leftHeaderLight: int = headerLight - (int)(self.numHeaderLights/2)
        leds.setLEDs(0, 0, 0, 0, leftHeaderLight + self.numHeaderLights, self.totalLights - self.numHeaderLights)
        leds.setLEDs(0, 0, 0, 0, 0, leftHeaderLight)

        leds.setLEDs(0, 255, 0, 0, leftHeaderLight, self.numHeaderLights)

    def flashLEDs(self, trigger: bool, prevTrigger: bool, red: int, green: int, blue: int, duration: float, ):

            if trigger and not prevTrigger:
                print("hi")
                self.LEDFlashTimer = 2.0
                self.lastLEDTransition = self.time.timeSinceInit
            
            self.funnyTestNumber+=1
            
            self.red = red
            self.green = green
            self.blue = blue
        
            self.duration = duration
            
    def updateLED(self, table: ntcore.NetworkTable, time: TimeData) -> None:
        self.time = time
        table.putNumber("LEDAnimationFrame", self.LEDAnimationFrame)
        table.putBoolean("LEDTrigger", self.LEDTrigger)
        table.putNumber("LEDFlashTimer", self.LEDFlashTimer)
        table.putBoolean("ledPrevTrigger", self.LEDPrevTrigger)
        table.putBoolean("intake ledTrigger", self.intakeLEDTrigger)
        table.putBoolean("climber ledTrigger", self.climberLEDTrigger)
        table.putNumber("funny test number", self.funnyTestNumber)
        if self.lightToggle.getSelected() == LIGHTS_ON:
            self.lightTriggerBool = True
        else:
            self.lightTriggerBool = False
            
        if self.lightToggle2.getSelected() == LIGHTS_ON:
            self.lightTriggerBool2 = True
        else:
            self.lightTriggerBool2 = False
            
        table.putBoolean("light trigger bool", self.lightTriggerBool)
        table.putBoolean("light trigger bool 2", self.lightTriggerBool2)
        if self.hal.climbPos > 20:
            climbTrigger = True
        else:
            climbTrigger = False

        self.flashLEDs(self.hal.intakeSensor, self.intakeLEDPrevTrigger, 194, 54, 201, 0.2)  #barney purple on pickup
        self.intakeLEDPrevTrigger = self.hal.intakeSensor
        self.flashLEDs(climbTrigger, self.climberLEDPrevTrigger, 255, 247, 0, 0.2) #yellow on high climb
        self.climberLEDPrevTrigger = climbTrigger
        self.flashLEDs(self.lightTriggerBool, self.lightTestLEDPrevTrigger, 255, 255, 255, 0.2)
        self.lightTestLEDPrevTrigger = self.lightTriggerBool
        self.flashLEDs(self.lightTriggerBool2, self.lightTestLEDPrevTrigger2, 255, 255, 255, 0.2)
        self.lightTestLEDPrevTrigger2 = self.lightTriggerBool2
        
        if self.LEDFlashTimer > 0:
            
            self.LEDFlashTimer -= self.time.dt
            redBrightnessArray = [0, self.red, 0, self.red]
            greenBrightnessArray = [0, self.green, 0, self.green]
            blueBrightnessArray = [0, self.blue, 0, self.blue]
            
            if (self.time.timeSinceInit - self.lastLEDTransition > self.duration):
                self.lastLEDTransition = self.time.timeSinceInit
                self.hardware.setLEDs(redBrightnessArray[self.LEDAnimationFrame],
                                        greenBrightnessArray[self.LEDAnimationFrame],
                                        blueBrightnessArray[self.LEDAnimationFrame], 0, 0, 200)
                
                self.LEDAnimationFrame += 1
                self.LEDAnimationFrame %= len(redBrightnessArray)
                
                if redBrightnessArray[self.LEDAnimationFrame] > 0 or greenBrightnessArray[self.LEDAnimationFrame] > 0 or blueBrightnessArray[self.LEDAnimationFrame] > 0:
                    self.lightTestBool = True
                else:
                    self.lightTestBool = False

        else:
            self.LEDFlashTimer = 0.0
            self.hardware.setLEDs(0, 0, 0)
            self.lightTestBool = False
            
        table.putBoolean("lights on fr this time", self.lightTestBool)