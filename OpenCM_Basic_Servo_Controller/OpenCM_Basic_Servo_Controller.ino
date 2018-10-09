#include <DynamixelSDK.h>
//====================================================================================================
// Really basic Servo Controller...
// This version for Robotis OpenCM9.04
//====================================================================================================
//=============================================================================
// Options...
//=============================================================================
#define BAUDRATE                        1000000
#define DEVICE 1
#if DEVICE || defined(__OPENCR__)
#define DEVICENAME                      "3"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
#define DXLSerial Serial3
#else
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
#define DXLSerial Serial1
#endif

#define BUFFER_SIZE   64
//=============================================================================
// Globals
//=============================================================================
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
uint8_t buffer[BUFFER_SIZE];
int led_state = LOW;
uint32_t g_baud_rate = BAUDRATE;

//extern USBD_CDC_LineCodingTypeDef LineCoding;
//====================================================================================================
// Setup
//====================================================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial2.begin(115200);
  Serial.begin(BAUDRATE); // Init to actual default baud
  Serial2.println("Start Basic Servo Controller");
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
    led_state = led_state ? LOW : HIGH;
    digitalWrite(LED_BUILTIN, led_state);
  }

  // lets see if USB baudrate changed?
  uint32_t new_baud_rate = Serial.getBaudRate();
  //uint32_t new_baud_rate = LineCoding.bitrate;
  if (new_baud_rate != g_baud_rate) {
    DXLSerial.begin(new_baud_rate);
    Serial2.print("New Baud Rate: ");
    Serial2.println(new_baud_rate, DEC);
    g_baud_rate = new_baud_rate;
  }
}


//====================================================================================================
// Signal_Abort - Fatal error,  show blink pattern of error number...
//====================================================================================================
void signal_abort(uint8_t error) {
  for (;;) {
    for (uint8_t i = 0; i < error; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    delay(500);
  }
}
