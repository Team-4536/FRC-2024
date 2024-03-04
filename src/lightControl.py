import math

from phoenix5.led import CANdle  #only use for LEDs

totalLights = 8
numHeaderLights = 1

def setLights(leds: CANdle, yaw: float):
    #leds.setLEDs(0, 0, 0, 0, 0, totalLights)

    while yaw < 2 * math.pi:
        yaw += 2 * math.pi
    while yaw > 2 * math.pi:
        yaw -= 2 * math.pi

    headerLight: int = (int)((yaw / (2 * math.pi)) * totalLights)
    leftHeaderLight: int = headerLight - (int)(numHeaderLights/2)
    leds.setLEDs(0, 0, 0, 0, leftHeaderLight + numHeaderLights, totalLights - numHeaderLights)
    leds.setLEDs(0, 0, 0, 0, 0, leftHeaderLight)


    leds.setLEDs(0, 255, 0, 0, leftHeaderLight, numHeaderLights)
