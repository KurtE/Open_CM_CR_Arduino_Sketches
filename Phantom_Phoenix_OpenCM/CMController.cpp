#include "CMController.h"
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define DEVICENAME                      "3"                 // Check which port is being used on your controller


/* new-style setup */
void cm904Controller::setup(uint32_t baud, int servo_cnt) {
  int i;
  // setup storage
  id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
  pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
  nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
  speed_ = (int *) malloc(servo_cnt * sizeof(int));
  // initialize
  poseSize = servo_cnt;
  for (i = 0; i < poseSize; i++) {
    id_[i] = i + 1;
    pose_[i] = 512;
    nextpose_[i] = 512;
  }
  interpolating = 0;
  nextframe_ = millis();

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  // Open port
  if (!portHandler->openPort()) {
#ifdef DBGSerial
    DBGSerial.print("Failed to open the Dynamixel port!\n");
#endif
    return;
  }
  if (!portHandler->setBaudRate(baud)) {
#ifdef DBGSerial
    DBGSerial.print("Failed to change the Dynamixel baudrate!\n");
#endif
    return;
  }

}
void cm904Controller::setId(int index, int id) {
  id_[index] = id;
}
int cm904Controller::getId(int index) {
  return id_[index];
}

/* read in current servo positions to the pose. */
void cm904Controller::readPose() {
  for (int i = 0; i < poseSize; i++) {
    // Read present position
    uint16_t pos;
    uint8_t dxl_error = 0;                          // Dynamixel error
    packetHandler->read2ByteTxRx(portHandler, id_[i], AX_PRESENT_POSITION_L, &pos, &dxl_error);
    pose_[i] = pos << BIOLOID_SHIFT;
  }
}
uint8_t cm904Controller::getServoByte(uint8_t id, uint8_t reg) {
  uint8_t val;
  uint8_t dxl_error = 0;                          // Dynamixel error
  packetHandler->read1ByteTxRx(portHandler, id, reg, &val, &dxl_error);
  return val;
}

uint16_t cm904Controller::getServoWord(uint8_t id, uint8_t reg) {
  uint16_t val;
  uint8_t dxl_error = 0;                          // Dynamixel error
  int ret = packetHandler->read2ByteTxRx(portHandler, id, reg, &val, &dxl_error);
  if (ret != COMM_SUCCESS) return 0xffff;
  return val;
}

bool cm904Controller::setServoByte(uint8_t id, uint8_t reg, uint8_t val)
{
  uint8_t error;
  int ret = packetHandler->write1ByteTxRx(portHandler, id, reg, val, &error); 
  return ((ret == COMM_SUCCESS) && (error == 0));
}

bool cm904Controller::setServoWord(uint8_t id, uint8_t reg, uint16_t val)
{
  uint8_t error;
  int ret = packetHandler->write2ByteTxRx(portHandler, id, reg, val, &error); 
  return ((ret == COMM_SUCCESS) && (error == 0));
}

//--------------------------------------------------------------------
//[SetRegOnAllServos] Function that is called to set the state of one
//  register in all of the servos, like Torque on...
//--------------------------------------------------------------------
void cm904Controller::setRegOnAllServos(uint8_t bReg, uint8_t bVal)
{
  uint8_t *pb = bioloid_buffer_;
  for (int i = 0; i < poseSize; i++)
  {
    *pb++ = id_[i];
    *pb++ = bVal;
  }
  packetHandler->syncWriteTxOnly(portHandler, bReg, 1, bioloid_buffer_, 2 * poseSize);
}




/* write pose out to servos using sync write. */
void cm904Controller::writePose() {
  int temp;
  uint8_t *pb = bioloid_buffer_;
  for (int i = 0; i < poseSize; i++)
  {
    *pb++ = id_[i];
    temp = pose_[i] >> BIOLOID_SHIFT;
    *pb++ = (temp & 0xff);
    *pb++ = (temp >> 8);
  }
  packetHandler->syncWriteTxOnly(portHandler, AX_GOAL_POSITION_L, 2, bioloid_buffer_, 3 * poseSize);
}

/* set up for an interpolation from pose to nextpose over TIME
  milliseconds by setting servo speeds. */
void cm904Controller::interpolateSetup(int time) {
  int i;
  int frames = (time / frameLength) + 1;
  nextframe_ = millis() + frameLength;
  // set speed each servo...
  for (i = 0; i < poseSize; i++) {
    if (nextpose_[i] > pose_[i]) {
      speed_[i] = (nextpose_[i] - pose_[i]) / frames + 1;
    }
    else {
      speed_[i] = (pose_[i] - nextpose_[i]) / frames + 1;
    }
  }
  interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
#define WAIT_SLOP_FACTOR 10
int cm904Controller::interpolateStep(boolean fWait) {
  if (interpolating == 0) return 0x7fff;
  int i;
  int complete = poseSize;
  if (!fWait) {
    if (millis() < (nextframe_ - WAIT_SLOP_FACTOR)) {
      return (millis() - nextframe_);    // We still have some time to do something...
    }
  }
  while (millis() < nextframe_) ;
  nextframe_ = millis() + frameLength;
  // update each servo
  for (i = 0; i < poseSize; i++) {
    int diff = nextpose_[i] - pose_[i];
    if (diff == 0) {
      complete--;
    }
    else {
      if (diff > 0) {
        if (diff < speed_[i]) {
          pose_[i] = nextpose_[i];
          complete--;
        }
        else
          pose_[i] += speed_[i];
      }
      else {
        if ((-diff) < speed_[i]) {
          pose_[i] = nextpose_[i];
          complete--;
        }
        else
          pose_[i] -= speed_[i];
      }
    }
  }
  if (complete <= 0) interpolating = 0;
  writePose();
  return 0;
}

/* get a servo value in the current pose */
int cm904Controller::getCurPose(int id) {
  for (int i = 0; i < poseSize; i++) {
    if ( id_[i] == id )
      return ((pose_[i]) >> BIOLOID_SHIFT);
  }
  return -1;
}
/* get a servo value in the next pose */
int cm904Controller::getNextPose(int id) {
  for (int i = 0; i < poseSize; i++) {
    if ( id_[i] == id )
      return ((nextpose_[i]) >> BIOLOID_SHIFT);
  }
  return -1;
}
/* set a servo value in the next pose */
void cm904Controller::setNextPose(int id, int pos) {
  for (int i = 0; i < poseSize; i++) {
    if ( id_[i] == id ) {
      nextpose_[i] = (pos << BIOLOID_SHIFT);
      return;
    }
  }
}

/* Added by Kurt */
void cm904Controller::setNextPoseByIndex(int index, int pos) {  // set a servo value by index for next pose
  if (index < poseSize) {
    nextpose_[index] = (pos << BIOLOID_SHIFT);
  }
}

