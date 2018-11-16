#include <DynamixelSDK.h>
//====================================================================================================
// Kurts Quick and dirty test program to find servos on either Open CM or Open CR boards
// This is a test, only a test...
//
//====================================================================================================
//============================================================================
// Global Include files
//=============================================================================

//=============================================================================
// Options...
//=============================================================================
#if defined(__OPENCR__) || defined(__OPENCM904__)
uint8_t port_handler_numbers[] = {1, 3};    // Setup to handle both ports of openCR
#endif
#if defined(TEENSYDUINO)
uint8_t port_handler_numbers[] = {1};     // Default to Serial1
#if defined(__MK66FX1M0__)
// Try with the T3.6 board
#define SERVOBUS Serial1
#define SERVO_RX_PIN           27
#define SERVO_TX_PIN           26
#define SERVO_DIRECTION_PIN 28
#define SERVO_POWER_ENABLE_PIN  29
#else
#define SERVO_POWER_ENABLE_PIN 2
#endif
#endif
#define COUNT_PORTHANDLERS  sizeof(port_handler_numbers)

//=============================================================================
// Define differnt robots..
//=============================================================================

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2                2.0                 // See which protocol version is used in the Dynamixel
#define DEVICENAME                      "3"                 // Check which port is being used on your controller
#define DXL_BAUDRATE 1000000

/** EEPROM AREA **/
#define AX_PRESENT_POSITION_L       36
//=============================================================
// Defines for X series (XL430)
//=============================================================================
#define DXL_X_MODEL_NUMBER         0    // 2 (R)
#define DXL_X_PRESENT_POSITION     132 // 4 (R)



//=============================================================================
// Globals
//=============================================================================
// Global objects
// Handle to port handler and packet handler;
dynamixel::PortHandler *portHandlers[COUNT_PORTHANDLERS];

dynamixel::PacketHandler *packetHandler1;
dynamixel::PacketHandler *packetHandler2;
uint8_t g_servo_protocol[254];
uint32_t g_baud_rate = DXL_BAUDRATE;
//====================================================================================================
// Setup
//====================================================================================================
void setup() {

  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  Serial.println("\nCM9.04 Find Servos program");
#if defined(SERVO_RX_PIN)
  SERVOBUS.setRX(SERVO_RX_PIN);
#endif
#if defined(SERVO_TX_PIN)
  SERVOBUS.setTX(SERVO_TX_PIN);
#endif

#ifdef SERVO_POWER_ENABLE_PIN
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
#endif
  pinMode(4, OUTPUT);

  // Initialize PacketHandler instance
  packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
  // Open port

  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  // Initialize PortHandler instances

  char port_string[5];
  for (uint8_t i = 0; i < COUNT_PORTHANDLERS; i++) {
    itoa(port_handler_numbers[i], port_string, sizeof(port_string));
    portHandlers[i] = dynamixel::PortHandler::getPortHandler(port_string);
    Serial.printf("Get Port Handler %s %x\n", port_string, (uint32_t)portHandlers[i]);
    // Lets init the two different port handlers.
#if defined(SERVO_DIRECTION_PIN)
    portHandlers[i]->setTXEnablePin(SERVO_DIRECTION_PIN);
#endif
    Serial.println("    Call Open Port");
    if (!portHandlers[i]->openPort()) {
      Serial.print("Failed to open port 1 the Dynamixel port: ");
      Serial.println(port_string);
    }
    Serial.println("    Set Baud Rate");
    if (!portHandlers[i]->setBaudRate(g_baud_rate)) {
      Serial.print("Failed to change the Port 1 Dynamixel baudrate: ");
      Serial.println(port_string);
    }
  }
  delay(1000);
  // Lets start of trying to locate all servos.
  FindServos();


}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt

  // lets toss any charcters that are in the input queue
  Serial.println("\nFind Again, enter new baud or just hit enter");
  Serial.flush();  // make sure the complete set of prompts has been output...
  while (Serial.read() != -1) ; // Remove any stuff still in serial buffer.
  // Get a command
  uint32_t new_baud = CheckForNewBaud();
  if (new_baud) {
    SetBaudRate(new_baud);
  }
  FindServos();
}

//=======================================================================================

void FindServos(void) {

  uint16_t w;
  Serial.print("\nSearch for all servos at baud rate: ");
  Serial.println(g_baud_rate, DEC);

  // Initialize to no servos...
  for (int i = 0; i < 254; i++) {
    g_servo_protocol[i] = 0; // not found
  }
  for (uint8_t port_index = 0; port_index < COUNT_PORTHANDLERS; port_index++) {

    dynamixel::PortHandler *portHandler = portHandlers[port_index];
    Serial.print("Begin Searching on Port: ");
    Serial.println(port_handler_numbers[port_index], DEC);

    Serial.println("  Begin Protocol 1: ");
    digitalWrite(4, !digitalRead(4));
    for (int i = 0; i < 254; i++) {
      if (packetHandler1->read2ByteTxRx(portHandler, i, AX_PRESENT_POSITION_L, &w) == COMM_SUCCESS) {
        if (g_servo_protocol[i]) {
          Serial.println("Multiple servos found with same ID");
        }
        g_servo_protocol[i] = 1;
        Serial.print("    ");
        Serial.print(i, DEC);
        Serial.print(" - ");
        Serial.println(w, DEC);
      }
    }

    Serial.println("  Done");
    Serial.println("  Begin Protocol 2: ");
    digitalWrite(4, !digitalRead(4));
    for (int i = 0; i < 254; i++) {
      uint16_t model_number;
      uint32_t position;
      if (packetHandler2->ping(portHandler, i, &model_number) == COMM_SUCCESS) {
        if (g_servo_protocol[i]) {
          Serial.println("Multiple servos found with same ID");
        }
        g_servo_protocol[i] = 2;
        Serial.print("    ");
        Serial.print(i, DEC);
        Serial.print(", Model:");
        Serial.print((int)model_number, HEX);
        packetHandler2->read4ByteTxRx(portHandler, i, DXL_X_PRESENT_POSITION, &position);
        Serial.print(" - ");
        Serial.println(position, DEC);
      }
    }
    Serial.println("  Try Protocol 2 - broadcast ping: ");
    digitalWrite(4, !digitalRead(4));
    
    std::vector<uint8_t> vec;                       // Dynamixel data storages
    if (packetHandler2->broadcastPing(portHandler, vec) == COMM_SUCCESS) {
      Serial.print("Detected Dynamixel : \n");
      for (int i = 0; i < (int)vec.size(); i++)
      {
        int id = vec.at(i);
        Serial.print("    ");
        Serial.print(id, DEC);
        uint16_t model_number = 0;
        packetHandler2->read2ByteTxRx(portHandler, id, DXL_X_MODEL_NUMBER, &model_number);
        Serial.print(", Model:");
        Serial.println((int)model_number, HEX);
        
        
        g_servo_protocol[i] = 2;
      }
    } else Serial.println("Broadcast ping failed");
    
    Serial.println("  Done");
  }
}

//====================================================================================================
// Process command line, optionally return new baud rate
uint32_t CheckForNewBaud(void) {
  int ch;
  uint32_t new_baud = 0;

  for (;;) {
    // throw away any thing less than CR character...
    ch = Serial.read();
    if (ch != -1) {
      if ((ch >= '0') && (ch <= '9')) {
        new_baud = new_baud * 10 + (uint32_t)(ch - '0');
      } else {
        break;
      }
    }
  }
  while (Serial.read() != -1) ; // remove any trailing stuff.
  return new_baud;
}


//=======================================================================================
void SetBaudRate(uint32_t new_baud)
{
  Serial.print("Setting Baud to: ");
  Serial.println(new_baud);

  for (uint8_t i = 0; i < COUNT_PORTHANDLERS; i++) {
    if (!portHandlers[i]->setBaudRate(new_baud)) {
      Serial.print("Failed to change the Port ");
      Serial.print(port_handler_numbers[i], DEC);
      Serial.print("Baud to: ");
      Serial.println(new_baud, DEC);
    }
  }
  g_baud_rate = new_baud;
}
