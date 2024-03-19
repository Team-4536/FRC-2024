import math

from phoenix5.led import CANdle  #only use for LEDs

totalLights = 8
numHeaderLights = 1

LEDAnimationFrame = 0
LEDLastTransition = 0
LEDFlashTimer = 0.0
LEDPrevTrigger = False
LEDTrigger = False


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

def flashCheck(self, trigger: bool):
        self.LEDTrigger |= trigger
        if self.LEDTrigger and not self.LEDPrevTrigger:
            self.LEDFlashTimer = 2.0
            self.lastLEDTransition = self.time.timeSinceInit
        self.LEDPrevTrigger = self.LEDTrigger
        self.LEDTrigger = False
        
def flashLEDsWhite(self):
        if self.LEDFlashTimer > 0:
            self.LEDFlashTimer -= self.time.dt
            brightnessArray = [0, 255, 0, 255]
            if (self.time.timeSinceInit - self.lastLEDTransition > 0.2):
                self.lastLEDTransition = self.time.timeSinceInit
                self.hardware.setLEDs(brightnessArray[self.LEDAnimationFrame],
                                        brightnessArray[self.LEDAnimationFrame],
                                        brightnessArray[self.LEDAnimationFrame], 0, 0, 200)
                self.LEDAnimationFrame += 1
                self.LEDAnimationFrame %= len(brightnessArray)
        else:
            self.LEDFlashTimer = 0.0
            self.hardware.setLEDs(0, 0, 0)