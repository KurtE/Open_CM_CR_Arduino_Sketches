#define DEBUG_CONTROLLER
#ifdef USERC100
//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix RobotisRC100 version - Try to emulate most of PS2, but PS2 has 16 buttons and RobotisRC100
// has 10. so some things may not be there, others may be doubled up.
//
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
// Quick and Dirty description of controls... WIP
// In most cases I try to mention what button on the PS2 things coorespond to..
// On/OFF - Turning the RobotisRC100 2 on and off (PS2 start button)
// R1 - options (Change walk gait, Change Leg in Single Leg, Change GP sequence) (Select on PS2)
// R2 - Toggle walk method...  Run Sequence in GP mode
// R3 - Walk method (Not done yet) - (PS2 R3)
// L4 - Ballance mode on and off
// L5 - Stand/Sit (Triangle on PS2)
// L6+Right Joy UP/DOWN - Body up/down - (PS2 Dpad Up/Down)
// L6+Right Joy Left/Right - Speed higher/lower - (PS2 DPad left/right)
// Right Top(S7) - Cycle through options of Normal walk/Double Height/Double Travel) - (PS2 R1, R2)
// Left Top(S8) - Cycle through modes (Walk, Translate, Rotate, Single Leg) (PS2: Circle, X, L1, L2)

// Note: Left some descriptions of PS2 stuff, especially parts still left to Map/Implement.
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate,
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls] - Need to check...
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls] - How to do sequences???
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
//[CONSTANTS]
enum {
  WALKMODE = 0, TRANSLATEMODE, ROTATEMODE,
  MODECNT
};
enum {
  NORM_NORM = 0, NORM_LONG, HIGH_NORM, HIGH_LONG
};


#define cTravelDeadZone 6      //The deadzone for the analog input from the remote

#define RC100_TO  150000        // if we don't get a valid message in this number of mills turn off About 2.5 minutes. 

#ifndef USER
#define USER 13
#endif

//=============================================================================
//=============================================================================
/* bitmasks for buttons array */
#define BUT_U      0x01
#define BUT_D      0x02
#define BUT_L      0x04
#define BUT_R      0x08
#define BUT_1      0x10
#define BUT_2      0x20
#define BUT_3      0x40
#define BUT_4      0x80
#define BUT_RT     0x100
#define BUT_LT     0x200

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif


/* the RobotisRC100 will send out a frame at about 18 if any button is down, this class helps decipher the output. */
class RobotisRC100
{
  public:
    RobotisRC100();
    void begin(unsigned long baud);
    int ReadMsgs();         // must be called regularly to clean out Serial buffer

    // joystick values are -125 to 125
    signed char rightV;      // vertical stick movement = forward speed
    signed char rightH;      // horizontal stick movement = sideways or angular speed
    //  signed char leftV;      // vertical stick movement = tilt
    //  signed char leftH;      // horizontal stick movement = pan (when we run out of pan, turn body?)

    // buttons are 0 or 1 (PRESSED), and bitmapped
    uint16_t buttons;  //

    uint16_t buttons_prev;    // previous state of buttons.
  private:
    // internal variables used for reading messages
    int input_state;              // -1 = waiting for new packet
    uint8_t data_l;         // low byte from packet
    uint8_t data_h;         // high byte from packet
    uint8_t connected;      // Have we received any messages.
};


//=============================================================================
// Global - Local to this file only...
//=============================================================================
RobotisRC100 command = RobotisRC100();
unsigned long g_ulLastMsgTime;
short  g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet
boolean g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
#define RobotisRC100InputController InputController
// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller



static short   g_BodyYOffset;
static short   g_BodyYShift;
static byte    ControlMode;
static byte    HeightSpeedMode;
//static bool  DoubleHeightOn;
static bool    DoubleTravelOn;
static byte    bJoystickWalkMode;

static byte    buttonsPrev;
static byte    extPrev;

// some external or forward function references.
extern void RobotisRC100TurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void RobotisRC100InputController::Init(void)
{
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
#ifdef DBGSerial
  DBGSerial.print("RobotisRC100 Init: ");
#endif
  command.begin(57600);

  ControlMode = WALKMODE;
  HeightSpeedMode = NORM_NORM;
  //    DoubleHeightOn = false;
  DoubleTravelOn = false;
  bJoystickWalkMode = 0;

}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void RobotisRC100InputController::AllowControllerInterrupts(boolean fAllow __attribute__((unused)))
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the RobotisRC100 and then
//process any commands.
//==============================================================================
void RobotisRC100InputController::ControlInput(void)
{
  // See if we have a new command available...
  if (command.ReadMsgs() > 0) {
    // If we receive a valid message than turn robot on...
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;

    if (!g_InControlState.fRobotOn ) {
      g_InControlState.fRobotOn = true;
      fAdjustLegPositions = true;
    }

    // [SWITCH MODES]

    // Cycle through modes...
    if ((command.buttons & BUT_LT) && !(buttonsPrev & BUT_LT)) {
      if (++ControlMode >= MODECNT) {
        ControlMode = WALKMODE;    // cycled back around...
        MSound( 2, 50, 2000, 50, 3000);
      }
      else {
        MSound( 1, 50, 2000);
      }
    }

    //Stand up, sit down
    if ((command.buttons & BUT_3) && !(buttonsPrev & BUT_3)) {
      if (g_BodyYOffset > 0)
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 35;
      fAdjustLegPositions = true;
      g_fDynamicLegXZLength = false;
    }


    // Start of with X, Y just being +- half range...
    int x = (command.buttons & BUT_R) ? 64 : ((command.buttons & BUT_L) ? -64 : 0);
    int y = (command.buttons & BUT_U) ? 64 : ((command.buttons & BUT_D) ? -64 : 0);

    if (command.buttons & BUT_RT ) {
      // raise or lower the robot on the joystick up /down
      // Maybe should have Min/Max
      int delta = y / 25;
      if (delta) {
        g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
        fAdjustLegPositions = true;
      }

      // Also use right Horizontal to manually adjust the initial leg positions.
      //sLegInitXZAdjust = lx/10;        // play with this.
      //sLegInitAngleAdjust = ly/8;
      x = 0;
      y = 0;

      // Likewise for Speed control
      delta = x / 16;   //
      if ((delta < 0) && g_InControlState.SpeedControl) {
        if ((word)(-delta) <  g_InControlState.SpeedControl)
          g_InControlState.SpeedControl += delta;
        else
          g_InControlState.SpeedControl = 0;
        MSound( 1, 50, 1000 + g_InControlState.SpeedControl);
      }
      if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
        g_InControlState.SpeedControl += delta;
        if (g_InControlState.SpeedControl > 2000)
          g_InControlState.SpeedControl = 2000;
        MSound( 1, 50, 1000 + g_InControlState.SpeedControl);
      }

      x = 0; // don't walk when adjusting the speed here...
      y = 0;
    }

    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (((command.buttons & BUT_4) && !(buttonsPrev & BUT_4))
          && abs(g_InControlState.TravelLength.x) < cTravelDeadZone //No movement
          && abs(g_InControlState.TravelLength.z) < cTravelDeadZone
          && abs(g_InControlState.TravelLength.y * 2) < cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType + 1;                  // Go to the next gait...
        if (g_InControlState.GaitType < NUM_GAITS) {               // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000);
        }
        else {
          MSound (2, 50, 2000, 50, 2250);
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound( 1, 50, 2000);
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2)
          g_InControlState.LegLiftHeight = 80;
        else
          g_InControlState.LegLiftHeight = 50;
      }

      if (command.buttons & BUT_1) {
        g_InControlState.TravelLength.x = 0;
        g_InControlState.TravelLength.z = 0;
        g_InControlState.TravelLength.y = -x/4; //Right Stick Left/Right 
      } else {
        g_InControlState.TravelLength.x = -x;
        g_InControlState.TravelLength.z = -y;
        g_InControlState.TravelLength.y = 0; //Right Stick Left/Right
      }

      //[Translate functions]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x =  SmoothControl(((x) * 2 / 3), g_InControlState.BodyPos.x, SmDiv);
        g_InControlState.BodyPos.z =  SmoothControl(((y) * 2 / 3), g_InControlState.BodyPos.z, SmDiv);
        g_InControlState.BodyRot1.y = 0; //SmoothControl(((command.rightH)*2), g_InControlState.BodyRot1.y, SmDiv);

        //      g_InControlState.BodyPos.x = (lx)/2;
        //      g_InControlState.BodyPos.z = -(ly)/3;
        //      g_InControlState.BodyRot1.y = (command.rightH)*2;
        g_BodyYShift = 0; //(-(command.rightV)/2);
      }
    }
    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (y);
      g_InControlState.BodyRot1.y = (command.rightH) * 2;
      g_InControlState.BodyRot1.z = (x);
      g_BodyYShift = (-(command.rightV) / 2);
    }

    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(x), abs(y)), abs(command.rightH));

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift,  0);

    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when we have finished any previous adjustment

      if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust)
          g_fDynamicLegXZLength = true;

        sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
        // Handle maybe change angles...
        if (sLegInitAngleAdjust)
          RotateLegInitAngles(sLegInitAngleAdjust);

        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }

    if (fAdjustLegPositions && !g_fDynamicLegXZLength)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

    // Save away the buttons state as to not process the same press twice.
    buttonsPrev = command.buttons;
    g_ulLastMsgTime = millis();
  }
  else {
    // We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
    if (g_InControlState.fRobotOn) {
      if ((millis() - g_ulLastMsgTime) > RC100_TO)
        RobotisRC100TurnRobotOff();
    }
  }
}

//==============================================================================
// RobotisRC100TurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void RobotisRC100TurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  g_InControlState.fRobotOn = 0;

#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

  g_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}
//================================================================================
#ifdef OPT_TERMINAL_MONITOR_IC
// Optional stuff to allow me to have Input device debug support
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void RobotisRC100InputController::ShowTerminalCommandList(void)
{
  //DBGSerial.println("X - Show RC100B Info");
}

boolean RobotisRC100InputController::ProcessTerminalCommand(byte *psz, byte bLen)
{
  return false;

}
#endif
//===============================================================================

//==============================================================================
// The below class code is based on the RobotisRC100 class by Michael Ferguson...
// I included and updated for my own usage...  As I may not always use
// Serial and I wish to decouple usage of joysticks from the names...
//==============================================================================

//==============================================================================
// RobotisRC100::RobotisRC100 - Constructor
//==============================================================================

RobotisRC100::RobotisRC100() {
  input_state = 0;
  connected = false;
}

//==============================================================================
// RobotisRC100::begin
//==============================================================================
void RobotisRC100::begin(unsigned long baud) {
  Serial2.begin(57600);
}

//==============================================================================
// ReadMsgs
//==============================================================================

// process messages coming from RobotisRC100
// format = 0xFF 55 DATA_L ~DATA_L DATA_H ~DATA_H
// But maybe also handle messages: CONNECT 9999<D><A>
// And: DISCONNECT<D><A>

int RobotisRC100::ReadMsgs() {
  int received_packet = 0;
  uint16_t new_buttons = 0;
  while (Serial2.available() > 0) {
    uint8_t ch = Serial2.read();
/*    DBGSerial.print(ch, HEX);
    DBGSerial.print("-");
    DBGSerial.print(input_state, DEC);
    DBGSerial.print(" "); */
    switch (input_state) {
      case 0:
        if (ch == 0xff)  input_state = 1;
        if (ch == 'C') input_state = 20; // look for CONNECT
        if (ch == 'D') input_state = 30; // look for DISCONNECT
        break;
      case 1:
        input_state = (ch == 0x55) ? 2 : 0;
        break;
      case 2: // Read in DATA_L
        data_l = ch;
        input_state = 3;
        break;
      case 3: // Read in ~DATA_L
        input_state = ((data_l + ch) == 0xff) ? 4 : 0;
        break;
      case 4: // Read in DATA_H
        data_h = ch;
        input_state = 5;
        break;
      case 5: // Read in ~DATA_H
        if ((data_h + ch) == 0xff) {
          received_packet = 1;
          new_buttons = (data_h << 8) | data_l;
        }
        input_state = 0;
        break;

      // CONNECT nnnn<cr><LF>
      case 20: input_state = (ch == 'O') ? 21 : 0; break;
      case 21: input_state = (ch == 'N') ? 22 : 0; break;
      case 22: input_state = (ch == 'N') ? 23 : 0; break;
      case 23: input_state = (ch == 'E') ? 24 : 0; break;
      case 24: input_state = (ch == 'C') ? 25 : 0; break;
      case 25: input_state = (ch == 'T') ? 26 : 0; break;
      case 26: input_state = (ch == ' ') ? 27 : 0; break;
      case 27:
#ifdef DBGSerial
        DBGSerial.println("\nRC100 Connect Message Received");
#endif
        connected = true;
        input_state = 0;
        break;
      // CONNECT nnnn<cr><LF>
      case 30: input_state = (ch == 'I') ? 31 : 0; break;
      case 31: input_state = (ch == 'S') ? 32 : 0; break;
      case 32: input_state = (ch == 'C') ? 33 : 0; break;
      case 33: input_state = (ch == 'O') ? 34 : 0; break;
      case 34: input_state = (ch == 'N') ? 35 : 0; break;
      case 35: input_state = (ch == 'N') ? 36 : 0; break;
      case 36: input_state = (ch == 'E') ? 37 : 0; break;
      case 37: input_state = (ch == 'C') ? 38 : 0; break;
      case 38: input_state = (ch == 'T') ? 39 : 0; break;
      case 39: input_state = (ch == 0xD) ? 40 : 0; break;
      case 40: 
        if (ch == 0xA) {
      case 41:
#ifdef DBGSerial
          DBGSerial.println("\nRC100 DisConnect Message Received");
#endif
          connected = false;
          RobotisRC100TurnRobotOff(); // do what is necessary to say the robot is off...
        }
        input_state = 0;
        break;

    }
  }
  if (received_packet) {
    // We have a packet.
    buttons_prev = buttons;
    buttons = new_buttons;
    // init some old variables until I find another way
    rightH = 0;
    rightV = 0;

#ifdef DEBUG_CONTROLLER
#ifdef DBGSerial
    if (g_fDebugOutput) {
      DBGSerial.print("RC100 Input: ");
      DBGSerial.print(buttons, HEX);
      DBGSerial.print(" Prev: ");
      DBGSerial.println(buttons_prev, HEX);
    }
#endif
#endif
    return 1;
  }
  return 0;
}

//==============================================================================
//==============================================================================
#endif // USERC100
