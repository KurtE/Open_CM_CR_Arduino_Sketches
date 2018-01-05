//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use AX-12 type servos using the
// Arbotix AX12 and bioloid libraries (which may have been updated)
//====================================================================
#include <Arduino.h> // Arduino 1.0

#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#ifdef cTurretRotPin
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS +2)
#else
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)
#endif

#define cPwmMult      128
#define cPwmDiv       375
#define cPFConst      512    // half of our 1024 range

// Some defines for Voltage processing
#define VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE 4000  // Min time in us Until we should do next interpolation, as to not interfer.
#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 150      // Max 6+ times per second
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000    // call at least once per second...
#define VOLTAGE_TIME_TO_ERROR          3000    // Error out if no valid item is returned in 3 seconds...

#include <DynamixelSDK.h>

#define USE_BIOLOIDEX            // Use the Bioloid code to control the AX12 servos...
#include "CMController.h"



#ifdef DBGSerial
//#define DEBUG
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
//#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifdef QUADMODE
static const byte cPinTable[] = {
  cRRCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLFCoxaPin,
  cRRFemurPin, cRFFemurPin, cLRFemurPin, cLFFemurPin,
  cRRTibiaPin, cRFTibiaPin, cLRTibiaPin, cLFTibiaPin
#ifdef c4DOF
  , cRRTarsPin,  cRFTarsPin,  cLRTarsPin,  cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif
};
#elif defined(OCTOMODE)
static const byte cPinTable[] = {
  cRRCoxaPin,  cRMRCoxaPin,  cRMFCoxaPin,  cRFCoxaPin,  cLRCoxaPin,   cLMRCoxaPin,  cLMFCoxaPin,  cLFCoxaPin,
  cRRFemurPin, cRMRFemurPin, cRMFFemurPin, cRFFemurPin, cLRFemurPin,  cLMRFemurPin, cLMFFemurPin, cLFFemurPin,
  cRRTibiaPin, cRMRTibiaPin, cRMFTibiaPin, cRFTibiaPin, cLRTibiaPin,  cLMRTibiaPin, cLMFTibiaPin, cLFTibiaPin
#ifdef c4DOF
  , cRRTarsPin, cRMRTarsPin, cRMFTarsPin,  cRFTarsPin, cLRTarsPin, cLMRTarsPin, cLMFTarsPin, cLFTarsPin
#endif
};
#else
static const byte cPinTable[] = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin,
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin
#ifdef c4DOF
  , cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif
};
#endif
#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)
#ifdef c4DOF
#define FIRSTTARSPIN     (CNT_LEGS*3)
#define FIRSTTURRETPIN   (CNT_LEGS*4)
#else
#define FIRSTTURRETPIN   (CNT_LEGS*3)
#endif
// Not sure yet if I will use the controller class or not, but...
cm904Controller cm904 = cm904Controller();  // Use two phase initialization.
uint32_t g_servos_torque_enabled = 0;    // Are the servos in a free state?
uint32_t g_servos_torque_errors = 0;    //

//============================================================================================
// Lets try rolling our own GPSequence code here...
#define GPSEQ_EEPROM_START 0x40       // Reserve the first 64 bytes of EEPROM for other stuff...
#define GPSEQ_EEPROM_START_DATA  0x50 // Reserved room for up to 8 in header...
#define GPSEQ_EEPROM_SIZE 0x800       // I think we have 2K
#define GPSEQ_EEPROM_MAX_SEQ 5        // For now this is probably the the max we can probably hold...


// Not sure if pragma needed or not...
//#pragma pack(1)
typedef struct {
  byte  bSeqNum;       // the sequence number, used to verify
  byte  bCntServos;    // count of servos
  byte  bCntSteps;     // How many steps there are
  byte  bCntPoses;     // How many poses
}
EEPromPoseHeader;

typedef struct {
  byte bPoseNum;        // which pose to use
  word wTime;        // Time to do pose
}
EEPROMPoseSeq;      // This is a sequence entry

// Pose is just an array of words...


// A sequence is stored as:
//<header> <sequences><poses>



// Some forward references
extern void MakeSureServosAreOn(void);
extern void DoPyPose(byte *psz);
extern void EEPROMReadData(word wStart, uint8_t *pv, byte cnt);
extern void EEPROMWriteData(word wStart, uint8_t *pv, byte cnt);
extern void TCSetServoID(byte *psz);
extern void TCTrackServos();
extern void TCWiggleServos();

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  // First lets get the actual servo positions for all of our servos...
  //  pinMode(0, OUTPUT);
  g_servos_torque_enabled = 0;
  g_servos_torque_errors = 0;
  cm904.setup(1000000, NUMSERVOS);

  cm904.poseSize = NUMSERVOS;
  #ifndef DEFAULT_FRAME_TIME_MS
  #define DEFAULT_FRAME_TIME_MS 10
  #endif
  cm904.frameLength = DEFAULT_FRAME_TIME_MS;
#ifdef OPT_CHECK_SERVO_RESET
  uint16_t w;
  int     count_missing = 0;
  int     missing_servo = -1;
  bool    servo_1_in_table = false;

  for (int i = 0; i < NUMSERVOS; i++) {
    // Set the id
    int servo_id = cPinTable[i];
    cm904.setId(i, servo_id);

    if (cPinTable[i] == 1)
      servo_1_in_table = true;

    // Now try to get it's position
    w = cm904.getServoWord(servo_id, AX_PRESENT_POSITION_L);
    if (w == 0xffff) {
      // Try a second time to make sure.
      delay(25);
      w = cm904.getServoWord(servo_id, AX_PRESENT_POSITION_L);
      if (w == 0xffff) {
        // We have a failure
#ifdef DBGSerial
        DBGSerial.print("Servo(");
        DBGSerial.print(i, DEC);
        DBGSerial.print("): ");
        DBGSerial.print(servo_id, DEC);
        DBGSerial.println(" not found");
#endif
        if (++count_missing == 1)
          missing_servo = servo_id;
      }
    }
    delay(25);
  }

  // Now see if we should try to recover from a potential servo that renumbered itself back to 1.
#ifdef DBGSerial
  if (count_missing) {
    DBGSerial.print("ERROR: Servo driver init: ");
    DBGSerial.print(count_missing, DEC);
    DBGSerial.println(" servos missing");
  }
#endif

  if ((count_missing == 1) && !servo_1_in_table) {
    if ((uint16_t)cm904.getServoWord(1, AX_PRESENT_POSITION_L) != (uint16_t)0xffff) {
#ifdef DBGSerial
      DBGSerial.print("Servo recovery: Servo 1 found - setting id to ");
      DBGSerial.println(missing_servo, DEC);
#endif
      cm904.setServoByte(1, AX_ID, missing_servo);
    }
  }

#else
  cm904.readPose();
#endif
#ifdef cVoltagePin
  for (byte i = 0; i < 8; i++)
    GetBatteryVoltage();  // init the voltage pin
#endif

  // Currently have Turret pins not necessarily same as numerical order so
  // Maybe should do for all pins and then set the positions by index instead
  // of having it do a simple search on each pin...
#ifdef cTurretRotPin
  cm904.setId(FIRSTTURRETPIN, cTurretRotPin);
  cm904.setId(FIRSTTURRETPIN + 1, cTurretTiltPin);
#endif

  // Added - try to speed things up later if we do a query...
  cm904.setRegOnAllServos(AX_RETURN_DELAY_TIME, 0);  // tell servos to give us back their info as quick as they can...

}


//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin
word  g_awVoltages[8] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
word  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

word ServoDriver::GetBatteryVoltage(void) {
  g_iVoltages = (g_iVoltages + 1) & 0x7; // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
  g_wVoltageSum += g_awVoltages[g_iVoltages];

#ifdef CVREF
  return ((long)((long)g_wVoltageSum * CVREF * (CVADR1 + CVADR2)) / (long)(8192 * (long)CVADR2));
#else
  return ((long)((long)g_wVoltageSum * 125 * (CVADR1 + CVADR2)) / (long)(2048 * (long)CVADR2));
#endif
}

#else
word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...
byte g_bLegVoltage = 0;		// what leg did we last check?
unsigned long g_ulTimeLastBatteryVoltage;

word ServoDriver::GetBatteryVoltage(void) {
  // In this case, we have to ask a servo for it's current voltage level, which is a lot more overhead than simply doing
  // one AtoD operation.  So we will limit when we actually do this to maybe a few times per second.
  // Also if interpolating, the code will try to only call us when it thinks it won't interfer with timing of interpolation.
  unsigned long ulDeltaTime = millis() - g_ulTimeLastBatteryVoltage;
  if (g_wLastVoltage != 0xffff) {
    if ( (ulDeltaTime < VOLTAGE_MIN_TIME_BETWEEN_CALLS)
         || (cm904.interpolating &&  (ulDeltaTime < VOLTAGE_MAX_TIME_BETWEEN_CALLS)))
      return g_wLastVoltage;
  }

  // Lets cycle through the Tibia servos asking for voltages as they may be the ones doing the most work...
  register word wVoltage = cm904.getServoByte (cPinTable[FIRSTTIBIAPIN + g_bLegVoltage], AX_PRESENT_VOLTAGE);
  if (++g_bLegVoltage >= CNT_LEGS)
    g_bLegVoltage = 0;
  if (wVoltage != 0xffff) {
    g_ulTimeLastBatteryVoltage = millis();
    g_wLastVoltage = wVoltage * 10;
    return g_wLastVoltage;
  }

  // Allow it to error our a few times, but if the time until we get a valid response exceeds some time limit then error out.
  if (ulDeltaTime < VOLTAGE_TIME_TO_ERROR)
    return g_wLastVoltage;
  return 0;

}
#endif


//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update
{
  MakeSureServosAreOn();
  if (ServosEnabled) {
    DebugToggle(DEBUG_PIN_BEGIN_UPDATE);
    cm904.interpolateStep(true);    // Make sure we call at least once

  }
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif
{
  word    wCoxaSDV;        // Coxa value in servo driver units
  word    wFemurSDV;        //
  word    wTibiaSDV;        //
#ifdef c4DOF
  word    wTarsSDV;        //
#endif
  // The Main code now takes care of the inversion before calling.
  wCoxaSDV = (((long)(sCoxaAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
  wFemurSDV = (((long)((long)(sFemurAngle1)) * cPwmMult) / cPwmDiv + cPFConst);
  wTibiaSDV = (((long)(sTibiaAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
#ifdef c4DOF
  wTarsSDV = (((long)(sTarsAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
#endif
  if (ServosEnabled) {
    cm904.setNextPose(cPinTable[FIRSTCOXAPIN + LegIndex], wCoxaSDV);
    cm904.setNextPose(cPinTable[FIRSTFEMURPIN + LegIndex], wFemurSDV);
    cm904.setNextPose(cPinTable[FIRSTTIBIAPIN + LegIndex], wTibiaSDV);
#ifdef c4DOF
    if ((byte)cTarsLength[LegIndex])   // We allow mix of 3 and 4 DOF legs...
      cm904.setNextPose(cPinTable[FIRSTTARSPIN + LegIndex], wTarsSDV);
#endif
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print(LegIndex, DEC);
    DBGSerial.print("(");
    DBGSerial.print(sCoxaAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wCoxaSDV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print(sFemurAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wFemurSDV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print("(");
    DBGSerial.print(sTibiaAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wTibiaSDV, DEC);
    DBGSerial.print(") :");
  }
#endif
  g_InputController.AllowControllerInterrupts(true);    // Ok for hserial again...
}


//------------------------------------------------------------------------------------------
//[OutputServoInfoForTurret] Set up the outputse servos associated with an optional turret
//         the Leg number passed in.  FIRSTTURRETPIN
//------------------------------------------------------------------------------------------
#ifdef cTurretRotPin
void ServoDriver::OutputServoInfoForTurret(short sRotateAngle1, short sTiltAngle1)
{
  word    wRotateSDV;
  word    wTiltSDV;        //

  // The Main code now takes care of the inversion before calling.
  wRotateSDV = (((long)(sRotateAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
  wTiltSDV = (((long)((long)(sTiltAngle1)) * cPwmMult) / cPwmDiv + cPFConst);

  if (ServosEnabled) {    cm904.setNextPose(cPinTable[FIRSTTURRETPIN], wRotateSDV);
    cm904.setNextPose(cPinTable[FIRSTTURRETPIN + 1], wTiltSDV);
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print("(");
    DBGSerial.print(sRotateAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wRotateSDV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print(sTiltAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wTiltSDV, DEC);
    DBGSerial.print(") :");
  }
#endif
}
#endif
//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
#ifdef cSSC_BINARYMODE
  byte    abOut[3];
#endif

  g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (ServosEnabled) {
      cm904.interpolateSetup(wMoveTime);

  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput)
    DBGSerial.println(wMoveTime, DEC);
#endif
  g_InputController.AllowControllerInterrupts(true);

}
//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
  if (g_servos_torque_enabled) {  // Are there any servos that are active?
    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    cm904.setRegOnAllServos(AX_TORQUE_ENABLE, 0);  // do this as one statement...
#if 0
    for (byte i = 0; i < NUMSERVOS; i++) {
      Relax(cPinTable[i]);
    }
#endif
    g_InputController.AllowControllerInterrupts(true);
    g_servos_torque_enabled = 0;
  }
  g_servos_torque_errors = 0; // assume no errors
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
static uint8_t g_iIdleServoNum  = (uint8_t) - 1;
static uint8_t g_iIdleLedState = 1;  // what state to we wish to set...
void ServoDriver::IdleTime(void)
{
  // Each time we call this set servos LED on or off...
  g_iIdleServoNum++;
  if (g_iIdleServoNum >= NUMSERVOS) {
    g_iIdleServoNum = 0;
    g_iIdleLedState = 1 - g_iIdleLedState;
  }
  cm904.setServoByte(cPinTable[g_iIdleServoNum], AX_LED, g_iIdleLedState);

}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void MakeSureServosAreOn(void)
{
  if (ServosEnabled) {
    // If we logically have turned serovs on and there were no errors we can bypass this
//    if (g_servos_torque_enabled && !g_servos_torque_errors)
    if (g_servos_torque_enabled)
      return;    // we are not free

    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    cm904.readPose();

//    cm904.setRegOnAllServos(AX_TORQUE_ENABLE, 1);  // Use sync write to do it.
    uint32_t servo_mask = 1;
    bool any_servos_updated = false;
    for (byte i = 0; i < NUMSERVOS; i++) {
      if ((g_servos_torque_enabled & servo_mask) == 0) {
        bool succeeded = cm904.setServoByte(cPinTable[i], AX_TORQUE_ENABLE, 1);
        if (succeeded)  {any_servos_updated = true;
          g_servos_torque_enabled |= servo_mask;
          if (g_servos_torque_errors & servo_mask)  {
            g_servos_torque_errors &= ~servo_mask;
#ifdef DBGSerial            
            DBGSerial.print("Torque enabled after error on servo #");
            DBGSerial.println(cPinTable[i]);
#endif
          }
        } else {
          if ((g_servos_torque_errors & servo_mask) == 0) {
            g_servos_torque_errors |= servo_mask;
#ifdef DBGSerial            
            DBGSerial.print("Torque enable failed on servo #");
            DBGSerial.println(cPinTable[i]);
#endif
          }
        }
      }
      servo_mask <<= 1;
    }
    g_InputController.AllowControllerInterrupts(true);
    if (any_servos_updated) {
      delay(2); // give time for the servos to power up
    }
  }
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  ServoDriver::BackgroundProcess(void)
{
  if (ServosEnabled) {
    DebugToggle(DEBUG_PIN_BACKGROUND);

#ifdef cTurnOffVol          // only do if we a turn off voltage is defined
#ifndef cVoltagePin         // and we are not doing AtoD type of conversion...
    int iTimeToNextInterpolate =
#endif
#endif
      cm904.interpolateStep(false);    // Do our background stuff...

    // Hack if we are not interpolating, maybe try to get voltage.  This will acutally only do this
    // a few times per second.
#ifdef cTurnOffVol          // only do if we a turn off voltage is defined
#ifndef cVoltagePin         // and we are not doing AtoD type of conversion...
    if (iTimeToNextInterpolate > VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE )      // At least 4ms until next interpolation.  See how this works...
      GetBatteryVoltage();
#endif
#endif
  }
}


#ifdef OPT_TERMINAL_MONITOR
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void)
{
  DBGSerial.println("V - Voltage");
  DBGSerial.println("M - Toggle Motors on or off");
  DBGSerial.println("F<frame length> - FL in ms");    // BUGBUG::
  DBGSerial.println("A - Toggle AX12 speed control");
  DBGSerial.println("T - Test Servos");
  DBGSerial.println("I - Set Id <frm> <to");
  DBGSerial.println("S - Track Servos");
  DBGSerial.println("W - Wiggle Test");
#ifdef OPT_FIND_SERVO_OFFSETS
  DBGSerial.println("O - Enter Servo offset mode");
#endif
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean ServoDriver::ProcessTerminalCommand(byte *psz, byte bLen)
{
  if ((bLen == 1) && ((*psz == 'm') || (*psz == 'M'))) {
    g_fEnableServos = !g_fEnableServos;
    if (g_fEnableServos)
      DBGSerial.println("Motors are on");
    else
      DBGSerial.println("Motors are off");

    return true;
  }
  if ((bLen == 1) && ((*psz == 'v') || (*psz == 'V'))) {
    DBGSerial.print("Voltage: ");
    DBGSerial.println(GetBatteryVoltage(), DEC);
#ifdef cVoltagePin
    DBGSerial.print("Raw Analog: ");
    DBGSerial.println(analogRead(cVoltagePin));
#endif

    DBGSerial.print("From Servo 2: ");
    DBGSerial.println(cm904.getServoByte (2, AX_PRESENT_VOLTAGE), DEC);
  }

  if ((bLen == 1) && ((*psz == 't') || (*psz == 'T'))) {
    // Test to see if all servos are responding...
    for (int i = 1; i <= NUMSERVOS; i++) {
      int iPos;
      iPos = cm904.getServoWord(i, AX_PRESENT_POSITION_L);
      DBGSerial.print(i, DEC);
      DBGSerial.print("=");
      DBGSerial.println(iPos, DEC);
      delay(5);
    }
  }
  if ((*psz == 'i') || (*psz == 'I')) {
    TCSetServoID(++psz);
  }
  if ((*psz == 's') || (*psz == 'S')) {
    TCTrackServos();
  }

  if ((*psz == 'w') || (*psz == 'W')) {
    TCWiggleServos();
  }

  if ((bLen >= 1) && ((*psz == 'f') || (*psz == 'F'))) {
    psz++;  // need to get beyond the first character
    while (*psz == ' ')
      psz++;  // ignore leading blanks...
    byte bFrame = 0;
    while ((*psz >= '0') && (*psz <= '9')) {  // Get the frame count...
      bFrame = bFrame * 10 + *psz++ - '0';
    }
    if (bFrame != 0) {
      DBGSerial.print("New Servo Cycles per second: ");
      DBGSerial.println(1000 / bFrame, DEC);
      extern cm904Controller bioloid;
      cm904.frameLength = bFrame;
    }
  }

#ifdef OPT_FIND_SERVO_OFFSETS
  else if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
#endif
  return false;

}

//==============================================================================
// TCSetServoID - debug function to update servo numbers.
//==============================================================================
void TCSetServoID(byte *psz)
{
  word wFrom = GetCmdLineNum(&psz);
  word wTo = GetCmdLineNum(&psz);

  if (wFrom  && wTo) {
    // Both specified, so lets try
    DBGSerial.print("Change Servo from: ");
    DBGSerial.print(wFrom, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(wTo, DEC);
    cm904.setServoByte(wFrom, AX_ID, wTo);
/*    if (ax12ReadPacket(6)) { // get the response...
      DBGSerial.print(" Resp: ");
      DBGSerial.println(ax_rx_buffer[4], DEC);
    }
    else
      DBGSerial.println(" failed");
  */
  }
}

//==============================================================================
// TCTrackServos - Lets set a mode to track servos.  Can use to help figure out
// proper initial positions and min/max values...
//==============================================================================
void TCTrackServos()
{
  // First read through all of the servos to get their position.
  uint16_t auPos[NUMSERVOS];
  uint16_t  uPos;
  int i;
  boolean fChange;

  // Clear out any pending input characters
  while (DBGSerial.read() != -1)
    ;

  for (i = 0; i < NUMSERVOS; i++) {
    auPos[i] = cm904.getServoWord(cPinTable[i], AX_PRESENT_POSITION_L);
  }

  // Now loop until we get some input on the serial
  while (!DBGSerial.available()) {
    fChange = false;
    for (int i = 0; i < NUMSERVOS; i++) {
      uPos = cm904.getServoWord(cPinTable[i], AX_PRESENT_POSITION_L);
      // Lets put in a littl delta or shows lots
      if (abs(auPos[i] - uPos) > 2) {
        auPos[i] = uPos;
        if (fChange)
          DBGSerial.print(", ");
        else
          fChange = true;
        DBGSerial.print(cPinTable[i], DEC);
        DBGSerial.print(": ");
        DBGSerial.print(uPos, DEC);
        // Convert back to angle.
        int iAng = (((int)uPos - cPFConst) * cPwmDiv) / cPwmMult;
        DBGSerial.print("(");
        DBGSerial.print(iAng, DEC);
        DBGSerial.print(")");
      }
    }
    if (fChange)
      DBGSerial.println();
    delay(25);
  }

}

//==============================================================================
// TCWiggleServos - Lets set a mode to track servos.  Can use to help figure out
// proper initial positions and min/max values...
//==============================================================================
void TCWiggleServos()
{
  // Clear out any pending input characters
  while (DBGSerial.read() != -1)
    ;

  uint8_t iServo = 0;
  while (!DBGSerial.available()) {
    DBGSerial.print("Servo: ");
    DBGSerial.print(cPinTable[iServo], DEC);
    DBGSerial.print("(");
    DBGSerial.print(iServo, DEC);
    DBGSerial.print(") ");
    uint16_t w = cm904.getServoWord(cPinTable[iServo], AX_PRESENT_POSITION_L);
    DBGSerial.print(w, DEC);
    DBGSerial.print(" ");
    uint8_t servo_enabled = cm904.getServoByte(cPinTable[iServo], AX_TORQUE_ENABLE);
    DBGSerial.println(servo_enabled, DEC);
    if (w != 0xffff) {

      cm904.setServoByte(cPinTable[iServo], AX_TORQUE_ENABLE, 1);  // Use sync write to do it.

      cm904.setServoWord(cPinTable[iServo], AX_GOAL_POSITION_L, w-50);
      delay(500);
      cm904.setServoWord(cPinTable[iServo], AX_GOAL_POSITION_L, w+50);
      delay(500);
      cm904.setServoWord(cPinTable[iServo], AX_GOAL_POSITION_L, w);
      delay(500);
    } else {
      delay(500);
    }
    iServo++;
    if (iServo >= NUMSERVOS) iServo = 0;
  }


}

#endif

//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos...
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS

void FindServoOffsets()
{

}
#endif  // OPT_FIND_SERVO_OFFSETS

//==============================================================================
// EEPromReadData - Quick and dirty function to read multiple bytes in from
//  eeprom...
//==============================================================================
void EEPROMReadData(word wStart, uint8_t *pv, byte cnt) {
  while (cnt--) {
    *pv++ = EEPROM.read(wStart++);
  }
}



