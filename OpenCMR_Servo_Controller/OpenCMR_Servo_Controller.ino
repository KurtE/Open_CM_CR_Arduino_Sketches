//-----------------------------------------------------------------------------
// OpenCM9.04 Servos controller - Which tries to emulate some of the
//     default firmware that R++ manager restores to the board.
//     This version will not emulate the motion code, but does try to emulate
//     many of the other registers and the like
//-----------------------------------------------------------------------------
#include <EEPROM.h>
#include <OLLO.h>
#include "globals.h"


//-----------------------------------------------------------------------------
// Setup
//-----------------------------------------------------------------------------
void setup() {
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
#ifdef DBGSerial
  DBGSerial.begin(115200);
  delay(250);
  DBGSerial.println("OpenCM_Servo_Controller start"); DBGSerial.flush();
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
  DXL_BUSS.processInput();

  // Then process any data coming from the USB
  ProcessUSBInputData();
}
