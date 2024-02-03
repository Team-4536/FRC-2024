from phoenix5.led import CANdle #only use for LEDs
import math
totalLights = 8
numHeaderLights = 1

def setLights(leds: CANdle, yaw: float):
    leds.setLEDs(255, 255, 255, 0, 0, totalLights)
    headerLight: int = (int)((yaw / (2 * math.pi)) * totalLights)
    leftHeaderLight: int = headerLight - (int)(numHeaderLights/2)

    leds.setLEDs(0, 255, 0, 0, leftHeaderLight, numHeaderLights)