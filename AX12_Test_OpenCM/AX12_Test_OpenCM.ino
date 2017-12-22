
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

//=============================================================================
// Options...
//=============================================================================

// Uncomment the next line if building for a Quad instead of a Hexapod.
//#define QUAD_MODE
//#define TURRET
#define DEBUG_IO_PINS

#define DEFAULT_PORTHANDLER 1
//#define DEFAULT_PORTHANDLER 3

//#define VOLTAGE_ANALOG_PIN 0
//#define SOUND_PIN 1
#define SERVO1_SPECIAL  19     // We wish to reserve servo 1 so we can see servo reset

//=============================================================================
// Define differnt robots..
//=============================================================================

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
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

static const byte pgm_axdIDs[] PROGMEM = {
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



//=============================================================================
// Globals
//=============================================================================
// Global objects
// Handle to port handler and packet handler;
dynamixel::PortHandler *portHandler1;
dynamixel::PortHandler *portHandler3;
dynamixel::PortHandler *portHandler;

dynamixel::PacketHandler *packetHandler;

word           g_wVoltage;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

// Values to use for servo position...
byte          g_bServoID;
word          g_wServoGoalPos;
word          g_wServoGoalSpeed;

//====================================================================================================
// Setup
//====================================================================================================
void setup() {

  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.
  Serial.println("\nCM9.04 Servo Test program");
  
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler1 = dynamixel::PortHandler::getPortHandler("1");
  portHandler3 = dynamixel::PortHandler::getPortHandler("3");
  #if (DEFAULT_PORTHANDLER == 1)
  portHandler = portHandler1;
  Serial.println("Default PortHandler Servos on Serial1 (on the CM904)");
  #else
  portHandler = portHandler3;
  Serial.println("Default PortHandler Servos on Serial3 (on 485 expansion Board)");
  #endif
    
  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  // Open port
  if (!portHandler->openPort()) {
    Serial.print("Failed to open the Dynamixel port!\n");
  }
  if (!portHandler->setBaudRate(DXL_BAUDRATE)) {
    Serial.print("Failed to change the Dynamixel baudrate!\n");
  }
  delay(250);

  delay(1000);
  Serial.print("System Voltage in 10ths: ");
  Serial.println(g_wVoltage = getServoVoltage(LF_COXA), DEC);

}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt

  word wNewVoltage = getServoVoltage(LF_COXA);
  if (wNewVoltage != g_wVoltage) {
    g_wVoltage = wNewVoltage;
    Serial.print("System Voltage in 10ths: ");
    Serial.println(g_wVoltage, DEC);
  }

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
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]");
  Serial.println("s - switch Serial Bus");
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
      case 'f':
      case 'F':
        HoldOrFreeServos(0);
        break;
      case 'h':
      case 'H':
        HoldOrFreeServos(1);
        break;
      case 's':
      case 'S':
        SwitchPortHandler();
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
    }
  }
}



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

//
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
  for (int i = 0; i < NUM_SERVOS; i++) {
    packetHandler->write1ByteTxRx(portHandler, pgm_axdIDs[i], AX_TORQUE_ENABLE, 0x0);
  }
}
//=======================================================================================
void AllServosCenter(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    // See if this turns the motor off and I can turn it back on...
    packetHandler->write1ByteTxRx(portHandler, pgm_axdIDs[i], AX_TORQUE_ENABLE, 0x1);
    packetHandler->write2ByteTxRx(portHandler, pgm_axdIDs[i], AX_GOAL_POSITION_L, 0x1ff);
  }
}
//=======================================================================================
void HoldOrFreeServos(byte fHold) {
  word iServo;

  if (!FGetNextCmdNum(&iServo)) {
    // All servos...
    for (int i = 0; i < NUM_SERVOS; i++) {
      packetHandler->write1ByteTxRx(portHandler, pgm_axdIDs[i], AX_TORQUE_ENABLE, fHold);
    }
  }
  else {
    packetHandler->write1ByteTxRx(portHandler, iServo, AX_TORQUE_ENABLE, fHold);
  }
}

//=======================================================================================

//=======================================================================================
void SetServoPosition(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  Serial.println("Set Servo Position");
  if (FGetNextCmdNum(&w2)) {  // We have at least 2 parameters
    g_bServoID = w1;    // So first is which servo
    g_wServoGoalPos = w2;
    if (FGetNextCmdNum(&w2)) {  // We have at least 3 parameters
      g_wServoGoalSpeed = w2;
      packetHandler->write2ByteTxRx(portHandler, g_bServoID, AX_GOAL_SPEED_L, g_wServoGoalSpeed);
      Serial.print("Goal Speed: ");
      Serial.print(g_wServoGoalSpeed, DEC);
    }
  }
  else
    g_wServoGoalPos = w1;  // Only 1 paramter so assume it is the new position

  // Now lets try moving that servo there
  packetHandler->write2ByteTxRx(portHandler, g_bServoID, AX_GOAL_POSITION_L, g_wServoGoalPos);
  Serial.print(" ID: ");
  Serial.print(g_bServoID, DEC);
  Serial.print(" ");
  Serial.println(g_wServoGoalPos, DEC);
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

  // Now lets try moving that servo there
  packetHandler->write1ByteTxRx(portHandler, w1, AX_RETURN_DELAY_TIME, w2);
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

  // Now lets try moving that servo there
  packetHandler->write1ByteTxRx(portHandler, w1, AX_ID, w2);
}


//=======================================================================================
void GetServoPositions(void) {

  unsigned long ulBefore;
  unsigned long ulDelta;
  uint16_t w;
  uint8_t err;
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print((byte)pgm_axdIDs[i], DEC);
    Serial.print(":");
    ulBefore = micros();
    if (packetHandler->read2ByteTxRx(portHandler, pgm_axdIDs[i], AX_PRESENT_POSITION_L, &w, &err) == COMM_SUCCESS) {
      ulDelta = micros() - ulBefore;
      Serial.print(w, DEC);
      Serial.print(" ");
      Serial.print(ulDelta, DEC);
      Serial.print(" ");
      Serial.println(getServoByte(pgm_axdIDs[i], AX_RETURN_DELAY_TIME), DEC);
    } else {
      ulDelta = micros() - ulBefore;
      Serial.print("** Failed(" );
      Serial.print(err, DEC);
      Serial.print(") ** " );
      Serial.print(ulDelta, DEC);
      Serial.print(" ");
      Serial.print(getServoByte(pgm_axdIDs[i], AX_RETURN_DELAY_TIME), DEC);
      Serial.print("   Retry: ");
      if (packetHandler->read2ByteTxRx(portHandler, pgm_axdIDs[i], AX_PRESENT_POSITION_L, &w) == COMM_SUCCESS) {
        Serial.println(w, DEC);
      } else {
        Serial.println("** Failed **" );
      }
    }
    delay (100);
  }
}

//=======================================================================================
void FindServos(void) {

  uint16_t w;
  if (portHandler == portHandler1)
    Serial.println("Begin default Serial1: ");
  else  
    Serial.println("Begin default Serial3: ");
  for (int i = 0; i < 254; i++) {
    if (packetHandler->read2ByteTxRx(portHandler, i, AX_PRESENT_POSITION_L, &w) == COMM_SUCCESS) {
      Serial.print(i, DEC);
      Serial.print(" - ");
      Serial.println(w, DEC);
    }
    //delay (1);
  }
  Serial.println("Done");

  // Check other 
  dynamixel::PortHandler *other_portHandler;
  if (portHandler == portHandler1) {
    Serial.println("\nCheck for one on other (Serial3): ");
    other_portHandler = portHandler3;
  } else {  
    Serial.println("\nCheck for one on other (Serial1): ");
    other_portHandler = portHandler1;
  }  
  for (int i = 0; i < 254; i++) {
    if (packetHandler->read2ByteTxRx(other_portHandler, i, AX_PRESENT_POSITION_L, &w) == COMM_SUCCESS) {
      Serial.print(i, DEC);
      Serial.print(" - ");
      Serial.println(w, DEC);
    }
    //delay (1);
  }
  Serial.println("Done");
}
//=======================================================================================
int g_asPositionsPrev[NUM_SERVOS];
int g_asMins[NUM_SERVOS];
int g_asMaxs[NUM_SERVOS];

void TrackServos(boolean fInit) {

  uint16_t w;
  bool fSomethingChanged = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (packetHandler->read2ByteTxRx(portHandler, pgm_axdIDs[i], AX_PRESENT_POSITION_L, &w) != COMM_SUCCESS) {
      w = 0xffff;
    }
    if (fInit) {
      g_asMins[i] = w;
      g_asMaxs[i] = w;
    }
    if (w != g_asPositionsPrev[i]) {
      if (!fInit) {
        // only print if we moved more than some delta...
        if (abs(w - g_asPositionsPrev[i]) > 3) {
          Serial.print(IKPinsNames[i]);
          Serial.print("(");
          Serial.print((byte)pgm_axdIDs[i], DEC);
          Serial.print("):");
          Serial.print(w, DEC);
          Serial.print("(");
          Serial.print((((long)(w - 512)) * 375L) / 128L, DEC);
          Serial.print(") ");
          fSomethingChanged = true;
        }
      }
      g_asPositionsPrev[i] = w;
      if (g_asMins[i] > w)
        g_asMins[i] = w;

      if (g_asMaxs[i] < w)
        g_asMaxs[i] = w;
    }
  }
  if (fSomethingChanged)
    Serial.println();
}

void TrackPrintMinsMaxs(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print((byte)pgm_axdIDs[i], DEC);
    Serial.print(":");
    Serial.print(g_asMins[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMins[i] - 512)) * 375L) / 128L, DEC);
    Serial.print(") ");

    Serial.print(g_asMaxs[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMaxs[i] - 512)) * 375L) / 128L, DEC);
    Serial.println(")");
  }
}


//=======================================================================================
void PrintServoValues(void) {

  word wID;
  word w;
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  if (!FGetNextCmdNum(&wID))
    return;
  for (int i = 0; i < 50; i++) {
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

uint8_t getServoVoltage(uint8_t id) {
  uint8_t val;
  if (packetHandler->read1ByteTxRx(portHandler, id, AX_PRESENT_VOLTAGE, &val) == COMM_SUCCESS)
    return val;
  return 0xff;  
}

//=======================================================================================
uint8_t getServoByte(uint8_t id, uint8_t reg) {
  uint8_t val;
  uint8_t dxl_error = 0;                          // Dynamixel error
  int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, reg, &val, &dxl_error);
  return (dxl_comm_result == COMM_SUCCESS) ? val : 0xff;
}

//=======================================================================================
void SwitchPortHandler() {
  if (portHandler == portHandler3) {
    portHandler = portHandler1;
    Serial.println("Set PortHandler to Servos on Serial1 (on the CM904)");
  } else {
    portHandler = portHandler3;
    Serial.println("Set porthandler to Servos on Serial3 (on 485 expansion Board)");
  }  
}

