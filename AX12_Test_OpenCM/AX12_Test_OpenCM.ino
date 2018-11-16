//====================================================================================================
// Kurts Test program to try out different ways to manipulate the AX12 servos on the PhantomX
// This is a test, only a test...
//
// This version for Robotis OpenCM9.04
//====================================================================================================
//============================================================================
// Global Include files
//=============================================================================
#include <DynamixelSDK.h>
#include <syscalls.h>
//=============================================================================
// Options...
//=============================================================================
#if defined(__OPENCM904__)
uint8_t port_handler_numbers[] = {1, 3};    // Setup to handle both ports of openCM
#endif
#if defined(__OPENCR__)
uint8_t port_handler_numbers[] = {3};    // Setup to handle both ports of openCR
#define SERVO_POWER_ENABLE_PIN BDPIN_DXL_PWR_EN
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
// Uncomment the next line if building for a Quad instead of a Hexapod.
//#define QUAD_MODE
//#define TURRET
//#define DEBUG_IO_PINS

//#define VOLTAGE_ANALOG_PIN 0
//#define SOUND_PIN 1
#define SERVO1_SPECIAL  19     // We wish to reserve servo 1 so we can see servo reset

//=============================================================================
// Define differnt robots..
//=============================================================================

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2                2.0                 // See which protocol version is used in the Dynamixel
#define DEVICENAME                      "3"                 // Check which port is being used on your controller
#define DXL_BAUDRATE 1000000

// Constants
/* Servo IDs */
#define     RF_COXA       2
#define     RF_FEMUR      4
#define     RF_TIBIA      6

#define     RM_COXA      14
#define     RM_FEMUR     16
#define     RM_TIBIA     18

#define     RR_COXA       8
#define     RR_FEMUR     10
#define     RR_TIBIA     12

#ifdef SERVO1_SPECIAL
#define     LF_COXA       19
#else
#define     LF_COXA       1
#endif
#define     LF_FEMUR      3
#define     LF_TIBIA      5

#define     LM_COXA      13
#define     LM_FEMUR     15
#define     LM_TIBIA     17

#define     LR_COXA       7
#define     LR_FEMUR      9
#define     LR_TIBIA     11

#ifdef TURRET
#define     TURRET_ROT    20
#define     TURRET_TILT   21
#endif

static const byte pgm_axdIDs[] = {
  LF_COXA, LF_FEMUR, LF_TIBIA,
#ifndef QUAD_MODE
  LM_COXA, LM_FEMUR, LM_TIBIA,
#endif
  LR_COXA, LR_FEMUR, LR_TIBIA,
  RF_COXA, RF_FEMUR, RF_TIBIA,
#ifndef QUAD_MODE
  RM_COXA, RM_FEMUR, RM_TIBIA,
#endif
  RR_COXA, RR_FEMUR, RR_TIBIA
#ifdef TURRET
  , TURRET_ROT, TURRET_TILT
#endif
};

#define NUM_SERVOS ((int)(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0])))
const char* IKPinsNames[] = {
  "LFC", "LFF", "LFT",
#ifndef QUAD_MODE
  "LMC", "LMF", "LMT",
#endif
  "LRC", "LRF", "LRT",
  "RFC", "RFF", "RFT",
#ifndef QUAD_MODE
  "RMC", "RMF", "RMT",
#endif
  "RRC", "RRF", "RRT",
#ifdef TURRET
  "T-ROT", "T-TILT"
#endif
};


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

word           g_wVoltage;
uint8_t        g_servo_index_voltage = 0;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

// g_servo_protocol
typedef union {
  struct {
    uint8_t protocol: 2; // define which protocol to use
    uint8_t port: 6;    // Define which port it is on
  } b;
  uint8_t val;
} SERVO_PROT_PORT_t;

enum           {SERVO_NOT_FOUND = 0, SERVO_PROTOCOL1 = 1, SERVO_PROTOCOL2};

SERVO_PROT_PORT_t        g_servo_protocol[255] = {SERVO_NOT_FOUND};  // What type of servos do we have????

uint8_t        g_count_servos_found = 0;

// Values to use for servo position...
byte          g_bServoID;
word          g_wServoGoalPos;
word          g_wServoGoalSpeed;

//====================================================================================================
// forward reference
//====================================================================================================
extern bool IsValidServo(uint8_t servo_id);
//====================================================================================================
// Setup
//====================================================================================================
void setup() {
  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  Serial.println("\nCM9.04 Servo Test program");
  initMemoryUsageTest();

  pinMode(0, OUTPUT);
#if defined(SERVO_RX_PIN)
  SERVOBUS.setRX(SERVO_RX_PIN);
#endif
#if defined(SERVO_TX_PIN)
  SERVOBUS.setTX(SERVO_TX_PIN);
#endif

#ifdef SERVO_POWER_ENABLE_PIN
  Serial.printf("Enable Servo power: %d\n", SERVO_POWER_ENABLE_PIN);
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
#endif
  pinMode(4, OUTPUT);

  // Initialize PacketHandler instance
  Serial.println("Get Packet Handlers");
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
    Serial.printf("    Set Baud Rate: %d\n", DXL_BAUDRATE);
    if (!portHandlers[i]->setBaudRate(DXL_BAUDRATE)) {
      Serial.print("Failed to change the Port 1 Dynamixel baudrate: ");
      Serial.println(port_string);
    }
  }
  delay(1000);
  // Lets start of trying to locate all servos.
  FindServos();

  PrintServoVoltage();

}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt
  PrintServoVoltage();
  printMemoryUsage();

  // lets toss any charcters that are in the input queue
  while (Serial.read() != -1)
    ;

  Serial.println("0 - All Servos off");
  Serial.println("1 - All Servos center");
  Serial.println("2 - Set Servo position [<Servo>] <Position> [<Speed>]");
  Serial.println("3 - Set Servo Angle");
  Serial.println("4 - Get Servo Positions");
  Serial.println("5 - Find All Servos");
  Serial.println("6 - Set Servo return delay time");
  Serial.println("8 - Set ID: <old> <new>");
  Serial.println("9 - Print Servo Values");
  Serial.println("b - Baud <new baud>");
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]");
  Serial.println("r - Reboot [<sn>]");
  Serial.println("w - write <servo> <reg> <val> (<val2>...)\n\r");

  Serial.print(":");
  Serial.flush();  // make sure the complete set of prompts has been output...
  // Get a command
  if (GetCommandLine()) {
    Serial.println("");
    Serial.print("Cmd: ");
    Serial.println(g_aszCmdLine);
    g_iszCmdLine = 1;  // skip over first byte...
    switch (g_aszCmdLine[0]) {
      case '0':
        AllServosOff();
        break;
      case '1':
        AllServosCenter();
        break;
      case '2':
        SetServoPosition();
        break;
      case '3':
        break;
      case '4':
        GetServoPositions();
        break;
      case '5':
        FindServos();
        break;
      case '6':
        SetServoReturnDelayTime();
        break;
      case '8':
        SetServoID();
        break;
      case '9':
        PrintServoValues();
        break;
      case 'b':
      case 'B':
        SetBaudRate();
        break;
      case 'f':
      case 'F':
        HoldOrFreeServos(0);
        break;
      case 'h':
      case 'H':
        HoldOrFreeServos(1);
        break;
      case 'r':
      case 'R':
        RebootServos();
        break;
      case 't':
      case 'T':
        g_fTrackServos = !g_fTrackServos;
        if (g_fTrackServos) {
          Serial.println("Tracking On");
          TrackServos(true);  // call to initialize all of the positions.
        }
        else
          Serial.println("Tracking Off");
        TrackPrintMinsMaxs();
        break;
      case 'w':
      case 'W':
        WriteServoValues();
        break;
    }
  }
}

//====================================================================================================
void PrintServoVoltage() {
  // Lets try reading in the current voltage for the next servo we found...
  if (g_count_servos_found == 0) return; // no servos found
  Serial.println("PrintServo Voltage called");
  g_servo_index_voltage++;    // will wrap around...
  uint8_t sanity_test_count = 0;
  while (!g_servo_protocol[g_servo_index_voltage].val) {
    g_servo_index_voltage++;
    if (g_servo_index_voltage >= 254) g_servo_index_voltage = 0;
    sanity_test_count--;
    if (sanity_test_count == 0) {
      Serial.println("PSV Sanity Test fail");
      return;
    }
  }
  uint16_t wNewVoltage;
  dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[g_servo_index_voltage].b.port];
  if (g_servo_protocol[g_servo_index_voltage].b.protocol == SERVO_PROTOCOL1) {
    uint8_t bVoltage;
    packetHandler1->read1ByteTxRx(portHandler, g_servo_index_voltage, AX_PRESENT_VOLTAGE, &bVoltage);
    wNewVoltage = bVoltage;
  } else {
    packetHandler2->read2ByteTxRx(portHandler, g_servo_index_voltage, DXL_X_PRESENT_INPUT_VOLTAGE, &wNewVoltage);
  }
  if (wNewVoltage != g_wVoltage) {
    g_wVoltage = wNewVoltage;
    Serial.print("Servo: ");
    Serial.print(g_servo_index_voltage, DEC);
    Serial.print(" Voltage in 10ths: ");
    Serial.println(g_wVoltage, DEC);
  }
}


//====================================================================================================
// Helper function to read in a command line
uint8_t GetCommandLine(void) {
  int ch;
  uint8_t ich = 0;
  g_iszCmdLine = 0;

  for (;;) {
    // throw away any thing less than CR character...
    ch = Serial.read();
    if ((ch >= 10) && (ch <= 15)) {
      g_aszCmdLine[ich] = 0;
      return ich;
    }
    if (ch != -1)
      g_aszCmdLine[ich++] = ch;

    if (g_fTrackServos)
      TrackServos(false);
  }
}

//=======================================================================================
boolean FGetNextCmdNum(word * pw ) {
  // Skip all leading num number characters...
  while ((g_aszCmdLine[g_iszCmdLine] < '0') || (g_aszCmdLine[g_iszCmdLine] > '9')) {
    if (g_aszCmdLine[g_iszCmdLine] == 0)
      return false;  // end of the line...
    g_iszCmdLine++;
  }
  *pw = 0;
  while ((g_aszCmdLine[g_iszCmdLine] >= '0') && (g_aszCmdLine[g_iszCmdLine] <= '9')) {
    *pw = *pw * 10 + (g_aszCmdLine[g_iszCmdLine] - '0');
    g_iszCmdLine++;
  }
  return true;
}


//=======================================================================================
void AllServosOff(void) {
  // Quick and dirty way to do it by broadcast...
  for (uint8_t i = 0; i < COUNT_PORTHANDLERS; i++) {
    packetHandler1->write1ByteTxRx(portHandlers[i], 0xfe, AX_TORQUE_ENABLE, 0x0);
    packetHandler2->write1ByteTxRx(portHandlers[i], 0xfe, DXL_X_TORQUE_ENABLE, 0x0);
  }
}

//=======================================================================================
bool ReportAnyErrors(const char *psz, uint8_t servo_id, int retval, uint8_t error) {

  if ((retval == COMM_SUCCESS) && (error == 0)) return false; // no error
  Serial.print(psz);
  Serial.print(":");
  Serial.print(servo_id, DEC);
  Serial.print("(");
  switch (retval) {
    case COMM_PORT_BUSY : Serial.print("BUSY"); break;
    case COMM_TX_FAIL   : Serial.print("TX FAIL"); break;
    case COMM_RX_FAIL   : Serial.print("RX FAIL"); break;
    case COMM_TX_ERROR  : Serial.print("TX ERROR"); break;
    case COMM_RX_WAITING: Serial.print("RX WAIT"); break;
    case COMM_RX_TIMEOUT: Serial.print("TIMEOUT"); break;
    case COMM_RX_CORRUPT: Serial.print("CORRUPT"); break;
    default: Serial.print(retval, DEC);
  }
  Serial.print(",");
  Serial.print(error, HEX);
  switch (error) {
    case 1: Serial.print(" Result"); break;
    case 2: Serial.print(" Instruction"); break;
    case 3: Serial.print(" CRC"); break;
    case 4: Serial.print(" Range"); break;
    case 5: Serial.print(" Length"); break;
    case 6: Serial.print(" Limit"); break;
    case 7:  Serial.print(" Access"); break;
  }
  Serial.print(") ");
  return true;
}

//=======================================================================================
void AllServosCenter(void) {
  bool any_errors = false;
  uint8_t error;
  int retval;
  // First make sure all of the motors are turned on.
  for (int i = 0; i < 255; i++) {
    dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[i].b.port];

    if (g_servo_protocol[i].b.protocol == SERVO_PROTOCOL1) {
      // See if this turns the motor off and I can turn it back on...
      retval = packetHandler1->write1ByteTxRx(portHandler, i, AX_TORQUE_ENABLE, 0x1, &error);
      any_errors |= ReportAnyErrors("TQ ON", i, retval, error);
      retval = packetHandler1->write2ByteTxRx(portHandler, i, AX_GOAL_POSITION_L, 0x1ff, &error);
      any_errors |= ReportAnyErrors("Goal", i, retval, error);
    } else if (g_servo_protocol[i].b.protocol == SERVO_PROTOCOL2) {
      retval = packetHandler2->write1ByteTxRx(portHandler, i, DXL_X_TORQUE_ENABLE, 0x1, &error);
      any_errors |= ReportAnyErrors("TQ ON", i, retval, error);
      retval = packetHandler2->write4ByteTxRx(portHandler, i, DXL_X_GOAL_POSITION, 2047, &error);
      any_errors |= ReportAnyErrors("Goal", i, retval, error);
    }
  }

  if (any_errors) Serial.println();
}


//=======================================================================================
void HoldOrFreeServos(byte fHold) {
  word iServo;
  if (!FGetNextCmdNum(&iServo)) {
    for (int i = 0; i < 255; i++) {
      dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[i].b.port];
      if (g_servo_protocol[i].b.protocol == SERVO_PROTOCOL1) {
        // See if this turns the motor off and I can turn it back on...
        packetHandler1->write1ByteTxRx(portHandler, i, AX_TORQUE_ENABLE, fHold);
      } else if (g_servo_protocol[i].b.protocol == SERVO_PROTOCOL2) {
        packetHandler2->write1ByteTxRx(portHandler, i, DXL_X_TORQUE_ENABLE, fHold);
      }
    }
  }
  else {
    if (IsValidServo(iServo)) {
      dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[iServo].b.port];
      if (g_servo_protocol[iServo].b.protocol == SERVO_PROTOCOL1) {
        // See if this turns the motor off and I can turn it back on...
        packetHandler1->write1ByteTxRx(portHandler, iServo, AX_TORQUE_ENABLE, fHold);
      } else if (g_servo_protocol[iServo].b.protocol == SERVO_PROTOCOL2) {
        packetHandler2->write1ByteTxRx(portHandler, iServo, DXL_X_TORQUE_ENABLE, fHold);
      }

    }
  }
}

//=======================================================================================
//=======================================================================================
void RebootServos() {
  word iServo;
  while (FGetNextCmdNum(&iServo)) {
    if (IsValidServo(iServo)) {
      dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[iServo].b.port];
      if (g_servo_protocol[iServo].b.protocol == SERVO_PROTOCOL1) {
        // See if this turns the motor off and I can turn it back on...
        packetHandler1->reboot(portHandler, iServo);
      } else if (g_servo_protocol[iServo].b.protocol == SERVO_PROTOCOL2) {
        packetHandler2->reboot(portHandler, iServo);
      }
    }
  }
}

//=======================================================================================
void SetServoPosition(void) {
  word w1;
  word w2;
  dynamixel::PortHandler *portHandler;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  Serial.println("Set Servo Position");
  if (FGetNextCmdNum(&w2)) {  // We have at least 2 parameters
    g_bServoID = w1;    // So first is which servo

    if (!IsValidServo(g_bServoID)) {
      Serial.println("Servo not found");
      return;
    }

    portHandler = portHandlers[g_servo_protocol[g_bServoID].b.port];
    g_wServoGoalPos = w2;
    if (FGetNextCmdNum(&w2)) {  // We have at least 3 parameters
      g_wServoGoalSpeed = w2;
      if (g_servo_protocol[g_bServoID].b.protocol == SERVO_PROTOCOL1) {
        packetHandler1->write2ByteTxRx(portHandler, g_bServoID, AX_GOAL_SPEED_L, g_wServoGoalSpeed);
      } else {
        packetHandler2->write4ByteTxRx(portHandler, g_bServoID, DXL_X_GOAL_VELOCITY, g_wServoGoalSpeed);
      }
      Serial.print("Goal Speed: ");
      Serial.print(g_wServoGoalSpeed, DEC);
    }
  }
  else {
    if (!IsValidServo(g_bServoID)) {
      Serial.println("Servo not found");
      return;
    }

    portHandler = portHandlers[g_servo_protocol[g_bServoID].b.port];
    g_wServoGoalPos = w1;  // Only 1 paramter so assume it is the new position
  }

  // Now lets try moving that servo there
  if (g_servo_protocol[g_bServoID].b.protocol == SERVO_PROTOCOL1) {
    packetHandler1->write2ByteTxRx(portHandler, g_bServoID, AX_GOAL_POSITION_L, g_wServoGoalPos);
  } else {
    packetHandler2->write4ByteTxRx(portHandler, g_bServoID,  DXL_X_GOAL_POSITION, g_wServoGoalPos);
  }
  Serial.print(" ID: ");
  Serial.print(g_bServoID, DEC);
  Serial.print(" ");
  Serial.println(g_wServoGoalPos, DEC);

}

//=======================================================================================
bool IsValidServo(uint8_t servo_id) {
  if (g_servo_protocol[servo_id].val)
    return true;  // was found before.

  // First lets try Protocol1 ping
  for (uint8_t i = 0; i < COUNT_PORTHANDLERS; i++) {
    //Serial.printf("IsValidServo: %d Need to ping\n", servo_id);
    if (packetHandler1->ping(portHandlers[i], servo_id) == COMM_SUCCESS) {
      g_servo_protocol[servo_id].b.protocol = SERVO_PROTOCOL1;
      g_servo_protocol[servo_id].b.port = i;
      g_count_servos_found++;
      return true;
    }
    if (packetHandler2->ping(portHandlers[i], servo_id) == COMM_SUCCESS) {
      g_servo_protocol[servo_id].b.protocol = SERVO_PROTOCOL2;
      g_servo_protocol[servo_id].b.port = i;
      g_count_servos_found++;
      return true;
    }
  }
  return false;
}


//=======================================================================================
void SetServoReturnDelayTime(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&w2))
    w2 = 0;   // we will default to 0 (our desired)

  Serial.print("Set Servo ID: ");
  Serial.print(w1, DEC);
  Serial.print(" return delay time: ");
  Serial.println(w2, DEC);

  if (!IsValidServo(w1)) {
    Serial.print("Servo: ");
    Serial.print(w1, DEC);
    Serial.println("Was not found");
    return;
  }

  // Now lets update that servo
  int retval;
  uint8_t error;
  dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[w1].b.port];
  if (g_servo_protocol[w1].b.protocol == SERVO_PROTOCOL1) {
    retval = packetHandler1->write1ByteTxRx(portHandler, w1, AX_RETURN_DELAY_TIME, w2, &error);
  } else {
    retval = packetHandler2->write1ByteTxRx(portHandler, w1, DXL_X_RETURN_DELAY_TIME, w2, &error);
  }
  ReportAnyErrors("Set return delay", w1, retval, error);
}



//=======================================================================================
void SetServoID(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&w2))
    return;    // no parameters so bail.

  Serial.print("Set Servo ID From: ");
  Serial.print(w1, DEC);
  Serial.print(" To: ");
  Serial.println(w2, DEC);

  int retval;
  uint8_t error;
  dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[w1].b.port];
  if (g_servo_protocol[w1].b.protocol == SERVO_PROTOCOL1) {
    retval = packetHandler1->write1ByteTxRx(portHandler, w1, AX_ID, w2, &error);
  } else {
    retval = packetHandler2->write1ByteTxRx(portHandler, w1, DXL_X_ID, w2, &error);
  }
  ReportAnyErrors("Set ID", w1, retval, error);
}


//=======================================================================================
void GetServoPositions(void) {

  unsigned long ulBefore;
  unsigned long ulDelta;
  uint16_t w;
  uint32_t pos;
  uint8_t err;
  int retval;


  if (!g_count_servos_found) {
    Serial.println("Previous Find Servos failed to locate any servos: so retry");
    FindServos();
    return;
  }

  for (int i = 0; i < 255; i++) {
    if (!g_servo_protocol[i].val) continue;  // No servo found on that index
    dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[i].b.port];
    Serial.print(i, DEC);
    Serial.print(":");
    ulBefore = micros();
    uint8_t delay_time_reg;
    if (g_servo_protocol[i].b.protocol == SERVO_PROTOCOL1) {
      retval = packetHandler1->read2ByteTxRx(portHandler, i, AX_PRESENT_POSITION_L, &w, &err);
      delay_time_reg = AX_RETURN_DELAY_TIME;
      pos = w;
    } else {
      retval = packetHandler2->read4ByteTxRx(portHandler, i, DXL_X_PRESENT_POSITION, &pos, &err);
      delay_time_reg = DXL_X_RETURN_DELAY_TIME;
    }
    if (!ReportAnyErrors("Current Pos", i, retval, err)) {
      ulDelta = micros() - ulBefore;
      Serial.print(pos, DEC);
      Serial.print(" ");
      Serial.print(ulDelta, DEC);
      Serial.print(" ");
      Serial.println(getServoByte(i, delay_time_reg), DEC);
    } else {
      ulDelta = micros() - ulBefore;
      Serial.print("** Failed(" );
      Serial.print(err, DEC);
      Serial.println(")" );
    }
  }
}

//=======================================================================================

void FindServos(void) {

  g_count_servos_found = 0;
  uint16_t w;
  Serial.println("\nSearch for all servos");

  // Initialize to no servos...
  for (int i = 0; i < 254; i++) {
    g_servo_protocol[i].val = SERVO_NOT_FOUND;
  }

  for (uint8_t port_index = 0; port_index < COUNT_PORTHANDLERS; port_index++) {
    dynamixel::PortHandler *portHandler = portHandlers[port_index];
    Serial.print("Begin Searching on Port: ");
    //Serial.println(port_handler_numbers[port_index], DEC);
    Serial.println(portHandler->getPortName());

    Serial.print("  Begin Protocol 1: ");
    Serial.println(packetHandler1->getProtocolVersion());
    for (int i = 1; i < 254; i++) {
      Serial.print(".");
      if ((i & 0x3f) == 0) Serial.println();
      if (packetHandler1->read2ByteTxRx(portHandler, i, AX_PRESENT_POSITION_L, &w) == COMM_SUCCESS) {
        if (g_servo_protocol[i].val) {
          Serial.println("Multiple servos found with same ID");
        } else {
          g_count_servos_found++;
        }
        g_servo_protocol[i].b.protocol = SERVO_PROTOCOL1;
        g_servo_protocol[i].b.port = port_index;
        g_count_servos_found++;
        Serial.print("    ");
        Serial.print(i, DEC);
        Serial.print(" - ");
        Serial.println(w, DEC);
      }
    }

    Serial.println("  Done");
    Serial.print("  Begin Protocol 2: ");
    Serial.println(packetHandler2->getProtocolVersion());
    for (int i = 1; i < 254; i++) {
      uint16_t model_number;
      uint32_t position;
      if (packetHandler2->ping(portHandler, i, &model_number) == COMM_SUCCESS) {
        if (g_servo_protocol[i].val) {
          Serial.println("Multiple servos found with same ID");
        } else {
          g_count_servos_found++;
        }
        g_servo_protocol[i].b.protocol = SERVO_PROTOCOL2;
        g_servo_protocol[i].b.port = port_index;
        Serial.print("    ");
        Serial.print(i, DEC);
        Serial.print(", Model:");
        Serial.print(model_number, HEX);
        packetHandler2->read4ByteTxRx(portHandler, i, DXL_X_PRESENT_POSITION, &position);
        Serial.print(i, DEC);
        Serial.print(" - ");
        Serial.println(position, DEC);
      }
    }
    Serial.println("  Done");
  }
}


//=======================================================================================
int g_asPositionsPrev[255];
int g_asMins[255];
int g_asMaxs[255];

void TrackServos(boolean fInit) {

  uint16_t w;
  uint32_t dw;
  int pos;
  bool fSomethingChanged = false;
  for (int i = 0; i < 254; i++) {
    if (!g_servo_protocol[i].val) continue;
    dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[i].b.port];
    pos = 0xffff;
    if (g_servo_protocol[i].b.protocol == SERVO_PROTOCOL1) {
      if (packetHandler1->read2ByteTxRx(portHandler, i, AX_PRESENT_POSITION_L, &w) == COMM_SUCCESS)
        pos = w;
    } else {
      if (packetHandler2->read4ByteTxRx(portHandler, i, DXL_X_PRESENT_POSITION, &dw) == COMM_SUCCESS)
        pos = dw;
    }
    if (fInit) {
      g_asMins[i] = pos;
      g_asMaxs[i] = pos;
    }
    if (pos != g_asPositionsPrev[i]) {
      if (!fInit) {
        // only print if we moved more than some deltas
        if (abs(w - g_asPositionsPrev[i]) > 3) {
          Serial.print(IKPinsNames[i]);
          Serial.print("(");
          Serial.print((byte)pgm_axdIDs[i], DEC);
          Serial.print("):");
          Serial.print(pos, DEC);
          /*          Serial.print("(");
                    Serial.print((((long)(w - 512)) * 375L) / 128L, DEC);
                    Serial.print(") "); */
          Serial.print(" ");
          fSomethingChanged = true;
        }
      }
      g_asPositionsPrev[i] = pos;
      if (g_asMins[i] > pos)
        g_asMins[i] = pos;

      if (g_asMaxs[i] < pos)
        g_asMaxs[i] = pos;
    }
  }
  if (fSomethingChanged)
    Serial.println();
}

void TrackPrintMinsMaxs(void) {
  for (int i = 0; i < 254; i++) {
    if (!g_servo_protocol[i].val) continue;
    Serial.print(i, DEC);
    Serial.print(":");
    Serial.print(g_asMins[i], DEC);
    Serial.print(" - ");
    Serial.println(g_asMaxs[i], DEC);
  }
}


//=======================================================================================
void PrintServoValues(void) {

  word wID;
  word w;
  word w_reg_count;
#ifdef DEBUG_IO_PINS
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
#endif
  if (!FGetNextCmdNum(&wID))
    return;
  if (!FGetNextCmdNum(&w_reg_count))
    w_reg_count = 50;
  for (int i = 0; i < w_reg_count; i++) {
    Serial.print(i, DEC);
    Serial.print(":");
#ifdef DEBUG_IO_PINS
    digitalWrite(A2, HIGH);
#endif
    w = getServoByte(wID, i);
#ifdef DEBUG_IO_PINS
    digitalWrite(A2, LOW);
    if (w == (word) - 1)
      digitalWrite(A3, !digitalRead(A3));
#endif
    Serial.print(w, HEX);
    Serial.print(" ");
    if ((i % 10) == 9)
      Serial.println("");
    Serial.flush();  // try to avoid any interrupts while processing.
    // delay(5);
  }
}

//=======================================================================================
void WriteServoValues() {
  word wID;
  word wReg;
  word wVal;
  uint8_t error;
  int retval;
  if (!FGetNextCmdNum(&wID))
    return;    // no parameters so bail.

  if (!IsValidServo(wID)) {
    Serial.print("Write register ID: ");
    Serial.print(wID, DEC);
    Serial.println(" Servo not found");
    return;
  }
  dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[wID].b.port];


  if (!FGetNextCmdNum(&wReg))
    return;    // no parameters so bail.

  while (FGetNextCmdNum(&wVal)) {
    Serial.print("Write register ID: ");
    Serial.print(wID, DEC);
    Serial.print(" Reg: ");
    Serial.print(wReg, DEC);
    Serial.print(" Val: ");
    Serial.print(wVal, DEC);
    if (g_servo_protocol[wID].b.protocol == SERVO_PROTOCOL1) {
      retval = packetHandler1->write1ByteTxRx(portHandler, wID, wReg, wVal, &error);
    } else {
      retval = packetHandler2->write1ByteTxRx(portHandler, wID, wReg, wVal, &error);
    }
    if (!ReportAnyErrors(" Write Reg", wID, retval, error)) {
      Serial.println(" Success");
    } else {
      Serial.println();
    }
    wReg++;   // get to the next reg
  }
}





//=======================================================================================
uint8_t getServoByte(uint8_t id, uint8_t reg) {
  uint8_t val;
  uint8_t dxl_error = 0;                          // Dynamixel error
  int dxl_comm_result;

  if (!IsValidServo(id)) {
    Serial.println("GSB not valid servo");
    return 0xff;
  }
  dynamixel::PortHandler *portHandler = portHandlers[g_servo_protocol[id].b.port];

  if (g_servo_protocol[id].b.protocol == SERVO_PROTOCOL1) {
    dxl_comm_result = packetHandler1->read1ByteTxRx(portHandler, id, reg, &val, &dxl_error);
  } else {
    dxl_comm_result = packetHandler2->read1ByteTxRx(portHandler, id, reg, &val, &dxl_error);
  }
  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
}



//=======================================================================================
void SetBaudRate()
{
  word wBaud;

  if (!FGetNextCmdNum(&wBaud))
    return;    // no parameters so bail.
  Serial.print("Setting Baud to: ");
  Serial.println(wBaud);

  for (uint8_t i = 0; i < COUNT_PORTHANDLERS; i++) {
    if (!portHandlers[i]->setBaudRate(wBaud)) {
      Serial.print("Failed to change the Port ");
      Serial.print(port_handler_numbers[i], DEC);
      Serial.print("Baud to: ");
      Serial.println(wBaud, DEC);
    }
  }

  Serial.println("Doing new Servo Scan");
  FindServos();
}

//=================================================================================
// Lets initialize our memory usage code, to get an idea of how much has been
// used
register uint8_t * stack_ptr asm("sp");
extern char end asm("end");

uint32_t g_end_stack_pointer;
uint32_t g_start_heap_pointer;

void initMemoryUsageTest()
{
  // Guess on start of stack. // probably using less than 100 bytes of stack space...
  g_end_stack_pointer = ((uint32_t)stack_ptr + 100) & 0xfffff000;

  // get the start of the heap ponter
  g_start_heap_pointer = (uint32_t)&end;

  // Print out some memory information
  Serial.printf("starting Heap info: start: %x current: %x\n", g_start_heap_pointer, (uint32_t)_sbrk(0));
  Serial.printf("Start Stack info: end: %x current: %x\n", g_end_stack_pointer, (uint32_t)stack_ptr);
  Serial.println("Try to init memory");
  Serial.flush(); // make sure it has chance to write out.
  uint8_t *sp_minus = stack_ptr - 10;  // leave a little slop
  for (uint8_t *p = (uint8_t*)_sbrk(0); p < sp_minus; p++) *p = 0xff; // init to ff
  Serial.println("After init memory");
}

//=================================================================================
void printMemoryUsage()
{
  uint8_t *current_heap_ptr = (uint8_t*)_sbrk(0);
  Serial.printf("Heap ptr: %x  Usage: %d\n", (uint32_t)current_heap_ptr,
                 (uint32_t)current_heap_ptr - g_start_heap_pointer);

  // stack info
  uint8_t *sp_minus = stack_ptr - 10;  // leave a little slop
  uint8_t *p = current_heap_ptr;

  // try to find out how far the stack has been used
  while ((p < sp_minus) && (*p == 0xff)) p++;
  Serial.printf("Stack Max: %x, usage: %d\n", p, g_end_stack_pointer - (uint32_t)p);
  Serial.printf("Estimated unused memory: %d\n", (uint32_t)(p - current_heap_ptr));
}
