#include <DynamixelSDK.h>
//====================================================================================================
// Really basic Servo Controller... 
// This version for Robotis OpenCM9.04
//====================================================================================================
//=============================================================================
// Options...
//=============================================================================
#define BAUDRATE                        1000000
#define DEVICENAME                      "3"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
#define BUFFER_SIZE   64
//=============================================================================
// Globals
//=============================================================================
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
uint8_t buffer[BUFFER_SIZE];
int led_state = LOW;

//====================================================================================================
// Setup
//====================================================================================================
void setup() {
  pinMode(BOARD_LED_PIN, OUTPUT);

  Serial.begin(115200); // actual speed does not matter USB

  // Open Port handler
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  if (!portHandler->openPort()) signal_abort(1);
  if (!portHandler->setBaudRate(BAUDRATE)) signal_abort(2);
}

//====================================================================================================
// loop
//====================================================================================================
void loop() {
  bool did_something = false;
  int cb;
  // First forward anything received from DXL
  cb = portHandler->readPort(buffer, BUFFER_SIZE);
  if (cb > 0) {
    Serial.write(buffer, cb);  
    did_something = true;
  }

  // Now forward anything received on USB to DXL
  cb = 0;
  while (Serial.available()) {
    buffer[cb++] = Serial.read();
  }
  if (cb) {
    portHandler->writePort(buffer, cb);  
    did_something = true;
  }
  if (did_something) {
    led_state = led_state? LOW : HIGH;
    digitalWrite(BOARD_LED_PIN, led_state);
  }
  
}


//====================================================================================================
// Signal_Abort - Fatal error,  show blink pattern of error number...
//====================================================================================================
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
