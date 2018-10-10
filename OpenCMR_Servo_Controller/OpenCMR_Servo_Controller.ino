//-----------------------------------------------------------------------------
// OpenCM9.04 Servos controller - Which tries to emulate some of the
//     default firmware that R++ manager restores to the board.
//     This version will not emulate the motion code, but does try to emulate
//     many of the other registers and the like
//-----------------------------------------------------------------------------
#include <EEPROM.h>
#include <OLLO.h>
#include "globals.h"

uint8_t print_debug_outputs = 1;  

//-----------------------------------------------------------------------------
// Setup
//-----------------------------------------------------------------------------
void setup() {
#ifdef DEBUG_PINS
  pinMode(0, OUTPUT); digitalWriteFast(0, LOW);
  pinMode(1, OUTPUT); digitalWriteFast(1, LOW);
  pinMode(2, OUTPUT); digitalWriteFast(2, LOW);
  pinMode(3, OUTPUT); digitalWriteFast(3, LOW);
  pinMode(4, OUTPUT); digitalWriteFast(4, LOW);
#endif
#ifdef DBGSerial
  DBGSerial.begin(115200);
  delay(250);
  DBGSerial.println("OpenCM_Servo_Controller start"); DBGSerial.flush();
  while (DBGSerial.available()) DBGSerial.read(); // clear out the input port of any garbage...
#endif
  InitalizeHardwareAndRegisters();

  Serial.begin(BAUDRATE);

  DBGPrintln("*** End Setup ***");
}

//-----------------------------------------------------------------------------
// Loop
//-----------------------------------------------------------------------------
void loop() {
  // Check and process any data coming from the DXL Buss
  DBGDigitalToggle(0);  // let us know if we make it through main loop
  DXL_BUSS.processInput();

  // Then process any data coming from the USB
  ProcessUSBInputData();

  // If we have DBGSerial set, check for input to maybe turn some debug outputs on and off.
#ifdef DBGSerial
  if (DBGSerial.available()) {
    uint32_t start_time = millis();
    while ((millis() - start_time) < 5) DBGSerial.read(); // wait a few milliseconds and clear out anything that comes in
    print_debug_outputs = !print_debug_outputs;  // toggle it on or off
    if (print_debug_outputs) 
      DBGSerial.println("\n\n*** Debug outputs Turned on ***");
    else
      DBGSerial.println("*** Debug outputs Turned off ***");
  }  
#endif
}
