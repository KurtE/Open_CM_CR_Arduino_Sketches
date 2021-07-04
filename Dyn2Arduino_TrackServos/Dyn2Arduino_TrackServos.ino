#include <Dynamixel2Arduino.h>
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
typedef struct {
  DYNAMIXEL::SerialPortHandler *port;
  uint8_t port_number;
  uint8_t protocol_index;
} PortList;

#if defined(__OPENCM904__)
#define DXL_SERIAL0 Serial1
#define DXL_DIR_PIN0 28
DYNAMIXEL::SerialPortHandler ph1(Serial1, 28);
DYNAMIXEL::SerialPortHandler ph3(Serial3, 22);
PortList portlist[] = {
  {&ph1, 1, 1},
  {&ph1, 1, 2},
  {&ph3, 3, 1},
  {&ph3, 3, 2},
};   // Setup to handle both ports of openCR
#elif defined(__OPENCR__)
#define DXL_SERIAL0 Serial1
#define DXL_DIR_PIN0 -1
DYNAMIXEL::SerialPortHandler dxlSerialPortHandlers[] =
{DYNAMIXEL::SerialPortHandler(DXL_SERIAL0, DXL_DIR_PIN0)};
#endif
#if defined(TEENSYDUINO)
#define DXL_SERIAL0 Serial2
#define DXL_DIR_PIN0 6
DYNAMIXEL::SerialPortHandler ph1(DXL_SERIAL0, DXL_DIR_PIN0);
PortList portlist[] = {
  {&ph1, 1, 1},
  {&ph1, 1, 2}
};   // Setup to handle both ports of openCR
#define SERVO_DIRECTION_PIN 2
//#define SERVO_POWER_ENABLE_PIN  3
#else
#define SERVO_POWER_ENABLE_PIN 2
#endif

#define DEBUG_TOGGLE_PIN 13
#ifdef DEBUG_TOGGLE_PIN
#define DBGTogglePin(pin) digitalWrite(pin, !digitalRead(pin));
#else
#define DBGTogglePin(pin)
#endif
#define COUNT_PORT_LIST  (sizeof(portlist)/sizeof(portlist[0]))
Dynamixel2Arduino dxl;

//=============================================================================
// Define different robots..
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
uint8_t g_servo_protocol[254];
uint32_t g_baud_rate = DXL_BAUDRATE;

void wait_keyboard() {
  Serial.println("Hit any key to continue"); Serial.flush();
  while (!Serial.available()) ;
  while (Serial.read() != -1) ;
}
//====================================================================================================
// Setup
//====================================================================================================
void setup() {

  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  Serial.println("\nCM9.04 Find Servos program");
  //  wait_keyboard();
#ifdef SERVO_POWER_ENABLE_PIN
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
#endif
#if defined(BOARD_OpenCM904) && (DXL_SERIAL == Serial1)
  Serial.println("*** OpenCM9.04 setDxlMode ***");
  Serial1.setDxlMode(true);
#endif

  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  // Initialize PortHandler instances
#ifdef DEBUG_TOGGLE_PIN
  pinMode(DEBUG_TOGGLE_PIN, OUTPUT);
#endif
  // Lets start of trying to locate all servos.
  SetBaudRate(DXL_BAUDRATE);

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
XelInfoFromPing_t ping_info[32];
void FindServos(void) {

  Serial.print("\nSearch for all servos at baud rate: ");
  Serial.println(g_baud_rate, DEC);

  // Initialize to no servos...
  for (int i = 0; i < 254; i++) {
    g_servo_protocol[i] = 0; // not found
  }
  for (uint8_t port_index = 0; port_index < COUNT_PORT_LIST; port_index++) {
    DBGTogglePin(DEBUG_TOGGLE_PIN);
    dxl.setPort(portlist[port_index].port);
    Serial.print("Begin Searching on Port: ");
    Serial.println(portlist[port_index].port_number, DEC);

    dxl.setPortProtocolVersionUsingIndex(portlist[port_index].protocol_index);
    Serial.println(dxl.getPortProtocolVersion(), 2);
    //  wait_keyboard();
    if (portlist[port_index].protocol_index == 1) {
      Serial.println("  Begin Protocol 1: ");
      for (int i = 0; i < 254; i++) {
        //Serial.print(".");
        if (dxl.ping(i)) {
          if (g_servo_protocol[i]) {
            Serial.println("Multiple servos found with same ID");
          }
          g_servo_protocol[i] = 1;
          Serial.printf("  %d Type:%d Position:%d\n", i,
                        dxl.getModelNumber(i), static_cast<int>(dxl.getPresentPosition(i))) ;
        }
      }

      Serial.println("  Done");
    } else {
      Serial.println("  Begin Protocol 2 Simple Ping: ");
      for (uint8_t i = 0; i < 254; i++) {
        //Serial.print("-");
        if (dxl.ping(i)) {
          if (g_servo_protocol[i] == 1) {
            Serial.println("Multiple servos found with same ID");
          }
          g_servo_protocol[i] = 2;
          Serial.printf("  %d Type:%d Position:%d\n", i,
                        dxl.getModelNumber(i), static_cast<int>(dxl.getPresentPosition(i))) ;
        }
      }
      Serial.println("  Begin Protocol 2 ping data: ");
      for (uint8_t i = 0; i < 254; i++) {
        if (dxl.ping(i, ping_info, 1)) {
          //        if (((DYNAMIXEL::Master)dxl).ping(i, ping_info, 1)) {
          if (g_servo_protocol[i] == 1) {
            Serial.println("Multiple servos found with same ID");
          }
          g_servo_protocol[i] = 2;
          Serial.printf("  %d Type:%x Ver:%x Position:%d \n", i,
                        ping_info[0].model_number, ping_info[0].firmware_version,
                        static_cast<int>(dxl.getPresentPosition(i))) ;
        }
      }
      Serial.println("  Try Protocol 2 - broadcast ping: ");
      Serial.flush(); // flush it as ping may take awhile...

      if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ping_info,
                                          sizeof(ping_info) / sizeof(ping_info[0]))) {
        Serial.print("Detected Dynamixel : \n");
        for (int i = 0; i < count_pinged; i++)
        {
          Serial.print("    ");
          Serial.print(ping_info[i].id, DEC);
          Serial.print(", Model:");
          Serial.print(ping_info[i].model_number, HEX);
          Serial.print(", Ver:");
          Serial.println(ping_info[i].firmware_version, HEX);
          g_servo_protocol[i] = 2;
        }
      } else Serial.printf("Broadcast returned no items(%x)\n", dxl.getLastLibErrCode());

      Serial.println("  Done");
    }
  }
}
//=======================================================================================
//=======================================================================================


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

  for (uint8_t i = 0; i < COUNT_PORT_LIST; i++) {
    DBGTogglePin(DEBUG_TOGGLE_PIN);
    Serial.printf("  Index: %d\n", i);
    dxl.setPort(portlist[i].port);
    dxl.begin(new_baud);
    delay(100);
    for (uint8_t id = 0; id < 254; id++) dxl.reboot(id);
  }
  g_baud_rate = new_baud;
}
