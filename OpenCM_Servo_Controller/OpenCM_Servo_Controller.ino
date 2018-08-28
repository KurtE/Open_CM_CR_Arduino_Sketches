#include <DynamixelSDK.h>
#include "globals.h"

dynamixel::PortHandler *portHandler;

extern void signal_abort(uint8_t error);


void setup() {
  InitalizeHardwareAndRegisters();
  uint32_t st = millis();
  // Put in fast blink until we have a serial port
  while(!Serial) {
    if ((millis() - st) > 250) {
      digitalWrite(BOARD_LED_PIN, !digitalRead(BOARD_LED_PIN));
      st = millis();
    }
  }
    
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  if (!portHandler->openPort()) signal_abort(1);
  if (!portHandler->setBaudRate(BAUDRATE)) signal_abort(2);
}

void loop() {
  // Check and process any data coming from the portHandler
  ProcessPortInputData(portHandler);

  // Then process any data coming from the USB
  ProcessUSBInputData();
}

// Fatal error - show blink pattern of error number...
void signal_abort(uint8_t error) {
  for(;;) {
    for (uint8_t i = 0; i < error; i++) {
      digitalWrite(BOARD_LED_PIN, HIGH);
      delay(100);
      digitalWrite(BOARD_LED_PIN, LOW);
      delay(100);
    }
    delay(500);
  }
}

// Process any data that comes from the Port and forward to USB
void ProcessPortInputData(dynamixel::PortHandler *portHandler) {
  // See if any data available from port actually the read takes care of MAX size plus empty
  int cb_port_avail = portHandler->readPort(from_port_buffer, BUFFER_SIZE);
  if (cb_port_avail) {
    // May want to see if we can write that many bytes without blocking...
    Serial.write(from_port_buffer, cb_port_avail);
  }
}
