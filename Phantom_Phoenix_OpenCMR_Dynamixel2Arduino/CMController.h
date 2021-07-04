#include <actuator.h>
#include <Dynamixel2Arduino.h>

/*
  BioloidController.h - ArbotiX Library for Bioloid Pose Engine
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef CM904_h
#define CM904_h
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


//#include <DynamixelSDK.h>

#define BIOLOID_MAX_SERVOS    20

/* pose engine runs at 30Hz (33ms between frames)
   recommended values for interpolateSetup are of the form X*BIOLOID_FRAME_LENGTH - 1 */
#define BIOLOID_FRAME_LENGTH      20
/* we need some extra resolution, use 13 bits, rather than 10, during interpolation */
#define BIOLOID_SHIFT             3

/** a structure to hold transitions **/
typedef struct {
  unsigned int * pose;    // addr of pose to transition to
  int time;               // time for transition
} transition_t;

typedef struct {
    uint8_t         id;                          // servo id for this index
    uint8_t         handler;                     // Which port/packet handler
    uint8_t         type;                        // What type of servo 0=AX, 1=XL430
    uint16_t        pose;                        // the current pose, updated by Step(), set out by Sync()
    uint16_t        nextpose;                    // the destination pose, where we put on load
    int16_t         speed;                       // speeds for interpolation
} servo_info_t;  
/** Bioloid Controller Class for mega324p/644p clients. **/
#define MAX_PORT_PROTOL_HANDLERS 4  // Currently sort of hard coded...
#define MAX_PORTS 2                 // 
enum {SERVO_AXMX=0, SERVO_XL430, SERVO_TYPE_COUNT};


class cm904Controller
{
  public:
    // Public Defines and data types
    typedef enum {REG_GOAL_POSITION=0, REG_TORQUE_ENABLE, REG_ID, REG_LED, REG_PRESENT_POSITION, REG_VOLTAGE} servo_logical_reg_t;

    // Public constructor and Initialize functions. 
    /* New-style constructor/setup */
    cm904Controller() {};
    void setup(int servo_cnt, HardwareSerial &hserial=Serial1, uint8_t protocol=1, uint32_t baud=1000000); 
//    bool addPortProtocol(HardwareSerial &hSerial=Serial1, uint8_t protocol=1, uint32_t baud=1000000);
    static Dynamixel2Arduino dxl;

    // Handle to port handler and packet handler;
//    dynamixel::PortHandler *portHandler[MAX_PORT_PROTOL_HANDLERS];
//    dynamixel::PacketHandler *packetHandler[MAX_PORT_PROTOL_HANDLERS];
    uint8_t port_packet_servo_count_[MAX_PORT_PROTOL_HANDLERS];
    // Add some support for finding servos and setting values outside of pose
    uint8_t findServo(uint8_t id, servo_info_t *pservo=NULL);  // if found returns the handler number, if not 0xff

//    uint8_t getServoByte(uint8_t id, uint8_t reg);
    uint32_t getServoValue(uint8_t id, servo_logical_reg_t reg);
    bool setServoValue(uint8_t id, servo_logical_reg_t reg, uint32_t val);

    // Functions that work with random servos 
    bool setServoByte(uint8_t handler, uint8_t id, uint8_t reg, uint8_t val);
    bool setServoWord(uint8_t handler, uint8_t id, uint8_t reg, uint16_t val);
//    void setRegOnAllServos(uint8_t bReg, uint8_t bVal);

    // Some common functions for getting or setting common things, whose index and size
    // may differ depending on Servo type.
    uint32_t getServoPosition(uint8_t index, uint8_t &error);   // We will use by index instead of ID. 
    void  setTorqueAllservos(bool torqueOn);

    /* Pose Manipulation */
    void readPose();                            // read a pose in from the servos
    void writePose();                           // write a pose out to the servos
    int getNextpose(int id);
    void setNextPose(int id, int pos);          // set a servo value in the next pose
    int setNextPoseAngle(int id, int angle);   // set the servo value by angle.       
    int convertPosToAngle(int id, int uPos);

    bool setId(int index, int id);              // set the id of a particular storage index
    int getId(int index);                       // get the id of a particular storage index
    servo_info_t *mapIDtoServoInfo(int id);     // Lets map an ID to servo info...
    /* Pose Engine */
    void interpolateSetup(int time);            // calculate speeds for smooth transition
    int interpolateStep(boolean fWait = true);                   // move forward one step in current interpolation
    unsigned char interpolating;                // are we in an interpolation? 0=No, 1=Yes
    unsigned char runningSeq;                   // are we running a sequence? 0=No, 1=Yes
    int poseSize;                               // how many servos are in this pose, used by Sync()

    uint8_t frameLength;                        // Allow variable frame lengths, to test...

  private:
    uint8_t     count_handlers_;                // Count of handlers. 
    servo_info_t * servos_;                     // Information about each of the servos;
/*    unsigned int * pose_;                       // the current pose, updated by Step(), set out by Sync()
    unsigned int * nextpose_;                   // the destination pose, where we put on load
    int * speed_;                               // speeds for interpolation
    unsigned char * id_;                        // servo id for this index
*/
    //    unsigned long lastframe_;                   // time last frame was sent out
    unsigned long nextframe_;                   //
    transition_t * sequence;                    // sequence we are running
    int transitions;                            // how many transitions we have left to load
    uint8_t bioloid_buffer_[BIOLOID_MAX_SERVOS * 5];                // bioloid call buffer.

};
#endif
