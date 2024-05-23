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

  pinMode(5,INPUT_PULLUP);
}

void loop() {
  //Pressing the on-board BOOTSEL button will initiate pushing 
  //all 32 buttons sequentially.
  
  //NOTE: this is not how you normally create custom buttons 
  //attached to an Arduino...its just really easy 
  
  if (!digitalRead(PIN_BUTTON)) { //Check the BOOTSEL button state (pulled-up --> FALSE=pressed)
    Serial.println("Joystick buttons");
    for (int i = 1; i <= 32; i++) {
      Serial.print(i);
      Serial.print(",");
      Joystick.button(i, true);
      delay(250);
      Joystick.button(i, false);
      delay(10); // We need a short delay here, sending packets with less than 1ms leads to packet loss!
    }
  }

  // Check manual button (needs to be installed on breadboard.)
  if (!digitalRead(5)) { //Check the Externally Wired Custom Button state (pulled-up --> FALSE=pressed)
   Joystick.button(3, true);
  } else {
   Joystick.button(3, false);
  }
  delay(10); // We need a short delay here, sending packets with less than 1ms leads to packet loss!
}
