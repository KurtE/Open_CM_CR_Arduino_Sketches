#include <EEPROM.h>

#include "globals.h"


void setup() {
#ifdef DBGSerial
  DBGSerial.begin(115200);
  delay(250);
  DBGSerial.println("OpenCM_Servo_Controller start");
#endif
  InitalizeHardwareAndRegisters();
  uint32_t st = millis();

  Serial.begin(BAUDRATE);

  DBGPrintln("*** End Setup ***");
}

void loop() {
  // Check and process any data coming from the DXL Buss
  DXL_BUSS.processInput();

  // Then process any data coming from the USB
  ProcessUSBInputData();
}

// Fatal error - show blink pattern of error number...
void signal_abort(uint8_t error) {
  for (;;) {
    for (uint8_t i = 0; i < error; i++) {
      digitalWrite(BOARD_LED_PIN, HIGH);
      delay(100);
      digitalWrite(BOARD_LED_PIN, LOW);
      delay(100);
    }
    delay(500);
  }
}
