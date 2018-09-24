#include <DynamixelSDK.h>


//====================================================================================================
// Kurts Test program to try out different ways to manipulate the AX12 servos on the PhantomX
// This is a test, only a test...
//
// This version for Robotis OpenCM9.04
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
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23
/** RAM AREA **/
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49
//=============================================================
// Defines for X series (XL430)
//=============================================================================
#define DXL_PING                    1
#define DXL_READ_DATA               2
#define DXL_WRITE_DATA              3
#define DXL_REG_WRITE               4
#define DXL_ACTION                  5
#define DXL_RESET                   6
#define DXL_SYNC_READ               0x82
#define DXL_SYNC_WRITE              0x83


/** EEPROM AREA **/
#define DXL_X_MODEL_NUMBER         0 //2 1060(xl430-250)
#define DXL_X_MODEL_INFORMATION    2 //4
#define DXL_X_VERSION              6
#define DXL_X_ID                   7   // 1 1
#define DXL_X_BAUD_RATE            8   // 1 1
#define DXL_X_RETURN_DELAY_TIME    9   // 1 250
#define DXL_X_DRIVE_MODE           10  // 1 0
#define DXL_X_OPERATING_MODE       11  // 1 3
#define DXL_X_SECONDARY_ID         12  // 1 255
#define DXL_X_PROTOCOL_VERSION     13  // 1 - 2
#define DXL_X_HOMING_OFFSET        20  // 4 - 0
#define DXL_X_MOVING_THRESHOLD     24  // 4 - 10
#define DXL_X_TEMPERATURE_LIMIT    31  //1 - 72
#define DXL_X_MAX_VOLTAGE_LIMIT    32  // 2 140
#define DXL_X_MIN_VOLTAGE_LIMIT    34  // 2 60
#define DXL_X_PWM_LIMIT            36  // 2 885
#define DXL_X_ACCELERATION_LIMIT   40  // 4 32767
#define DXL_X_VElOCITY_LIMIT       44  // 4 415
#define DXL_X_MAX_POSITION_LIMIT   48  // 4 4095
#define DXL_X_MIN_POSITION_LIMIT   52  // 4 0
#define DXL_X_SHUTDOWN             63  // 1 52

/** RAM AREA **/
#define DXL_X_TORQUE_ENABLE        64  // 1 0
#define DXL_X_LED                  65  // 1 0
#define DXL_X_STATUS_RETURN_LEVEL  68  // 1 2
#define DXL_X_REGISTERED_INSTRUCTION 69 //1 (R)
#define DXL_X_HARDWARE_ERROR_STATUS  70 // 1 (R)
#define DXL_X_VELOCITY_I_GAIN      76  // 2 1000 
#define DXL_X_VELOCITY_P_GAIN      78  // 2 100 
#define DXL_X_POSITION_D_GAIN      80  // 2 4000 
#define DXL_X_POSITION_I_GAIN      82  // 2 0
#define DXL_X_POSITION_P_GAIN      84  // 2 640
#define DXL_X_FEEDFORWARD_2_GAIN   88  // 2 0
#define DXL_X_FEEDFORWARD_1_GAIN   90  // 2 0
#define DXL_X_BUS_WATCHDOG         98  // 1 -
#define DXL_X_GOAL_PWM             100 // 2 
#define DXL_X_GOAL_VELOCITY        104 // 4
#define DXL_X_PROFILE_ACCELERATION 108 // 4
#define DXL_X_PROFILE_VELOCITY     112 // 4
#define DXL_X_GOAL_POSITION        116 // 4
#define DXL_X_REALTIME_TICK        120 // 2 (R)
#define DXL_X_MOVING               122 // 1 (R)
#define DXL_X_MOVING_STATUS        123 // 1 (R)
#define DXL_X_PRESENT_PWN          124 // 2 (R)
#define DXL_X_PRESENT_LOAD         126 // 2 (R)
#define DXL_X_PRESENT_VELOCITY     128 // 4 (R)
#define DXL_X_PRESENT_POSITION     132 // 4 (R)
#define DXL_X_VELOCITY_TRAJECTORY  136 // 4 (R)
#define DXL_X_POSITION_TRAJECTORY  140 // 4 (R)
#define DXL_X_PRESENT_INPUT_VOLTAGE  144 // 2 (R)
#define DXL_X_PRESENT_TEMPERATURE  146 // 1 (R)



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
