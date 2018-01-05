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


#include <DynamixelSDK.h>

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

/** Bioloid Controller Class for mega324p/644p clients. **/
class cm904Controller
{
  public:
    /* New-style constructor/setup */
    cm904Controller() {};
    void setup(uint32_t baud, int servo_cnt);

    // Handle to port handler and packet handler;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    uint8_t getServoByte(uint8_t id, uint8_t reg);
    uint16_t getServoWord(uint8_t id, uint8_t reg);
    bool setServoByte(uint8_t id, uint8_t reg, uint8_t val);
    bool setServoWord(uint8_t id, uint8_t reg, uint16_t val);
    void setRegOnAllServos(uint8_t bReg, uint8_t bVal);

    /* Pose Manipulation */
    void readPose();                            // read a pose in from the servos
    void writePose();                           // write a pose out to the servos
    int getCurPose(int id);                     // get a servo value in the current pose
    int getNextPose(int id);                    // get a servo value in the next pose
    void setNextPose(int id, int pos);          // set a servo value in the next pose
    void setNextPoseByIndex(int index, int pos);  // set a servo value by index for next pose
    void setId(int index, int id);              // set the id of a particular storage index
    int getId(int index);                       // get the id of a particular storage index

    /* Pose Engine */
    void interpolateSetup(int time);            // calculate speeds for smooth transition
    int interpolateStep(boolean fWait = true);                   // move forward one step in current interpolation
    unsigned char interpolating;                // are we in an interpolation? 0=No, 1=Yes
    unsigned char runningSeq;                   // are we running a sequence? 0=No, 1=Yes
    int poseSize;                               // how many servos are in this pose, used by Sync()

    // Kurt's Hacks
    uint8_t frameLength;                        // Allow variable frame lengths, to test...

  private:
    unsigned int * pose_;                       // the current pose, updated by Step(), set out by Sync()
    unsigned int * nextpose_;                   // the destination pose, where we put on load
    int * speed_;                               // speeds for interpolation
    unsigned char * id_;                        // servo id for this index

    //    unsigned long lastframe_;                   // time last frame was sent out
    unsigned long nextframe_;                   //
    transition_t * sequence;                    // sequence we are running
    int transitions;                            // how many transitions we have left to load
    uint8_t bioloid_buffer_[BIOLOID_MAX_SERVOS * 5];                // bioloid call buffer.

};
#endif

