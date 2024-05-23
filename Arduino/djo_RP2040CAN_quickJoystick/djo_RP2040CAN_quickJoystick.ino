/* Benjamin Aigner, 2022 <beni@asterics-foundation.org> */
/* Public domain / CC 0 */
//Simplification and Comments edits by David Orser

/** example code using all possibilities of the Joystick class
   for the RP2040.
*/

#include <Joystick.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Use BOOTSEL to start the Joystick demo.");
  Joystick.begin();
}

void loop() {
  //Pressing the on-board BOOTSEL button will initiate pushing 
  //all 32 buttons sequentially.
  
  //NOTE: this is not how you normally create custom buttons 
  //attached to an Arduino...its just really easy 
  
  if (!digitalRead(PIN_BUTTON)) { //Check the BOOTSEL button state (pulled-up --> FALSE=pressed)
    Serial.println("Joystick buttons");
    for (uint8_t i = 1; i <= 32; i++) {
      Joystick.button(i, true);
      delay(250);
      Joystick.button(i, false);
      delay(10); // We need a short delay here, sending packets with less than 1ms leads to packet loss!
    }
  }
}
