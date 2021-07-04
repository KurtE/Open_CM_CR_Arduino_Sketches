#include "CMController.h"

// BUGBUG assuming Hex currently to define if H
#include "Hex_Cfg.h"
// Define quick and dirty translation for different servo messages.
const static uint8_t g_axmx_regs[] = {AX_GOAL_POSITION_L, AX_TORQUE_ENABLE, AX_ID, AX_LED, AX_PRESENT_POSITION_L, AX_PRESENT_VOLTAGE};
const static uint8_t g_axmx_reg_len[] = {2, 1, 1, 1, 2, 1};
const static uint8_t g_xl430_regs[] = {DXL_X_GOAL_POSITION, DXL_X_TORQUE_ENABLE, DXL_X_ID, DXL_X_LED, DXL_X_PRESENT_POSITION, DXL_X_PRESENT_INPUT_VOLTAGE};
const static uint8_t g_xl430_reg_len[] = {4, 1, 1, 1, 4, 2};


// Conversion factors for AX like servos 300 degrees (3000 in tenths) range 1024 Center 512 
#define cPwmMult      128
#define cPwmDiv       375
#define cPFConst      512    // half of our 1024 range

// Conversion factors for XL430 like servos 360 degrees (3600 in tenths) range 4096 Center 2048 
#define cPwmMultXL      225
#define cPwmDivXL       256
#define cPFConstXL      2048    // half of our 1024 range

Dynamixel2Arduino cm904Controller::dxl(DXL_SERIAL, DXL_DIRECTION_PIN);

void cm904Controller::setup(int servo_cnt, HardwareSerial &hserial, uint8_t protocol, uint32_t baud) {
  int i;
  // setup storage
  servos_ = (servo_info_t *)malloc(servo_cnt * sizeof(servo_info_t));
  if (!servos_) {
#ifdef DBGSerial
    DBGSerial.print("Failed to allocate Servo structure!\n");
    return;
#endif
  }

  count_handlers_ = 0;

  // initialize
  poseSize = servo_cnt;
  servo_info_t * servo = servos_;
  for (i = 0; i < poseSize; i++) {
    servo->id = i + 1;      // Assume IDs are sequential
    servo->handler = 0xff;     // first handler
    servo->type = 0xff;        // AX like servo
    servo->pose = 512;      // assume center is 512    
    servo->nextpose = 512;
    servo->speed = 0;       
    servo++;
  }
  interpolating = 0;
  nextframe_ = millis();

  dxl.setPortProtocolVersion((float)protocol);
  dxl.begin(baud);
  port_packet_servo_count_[count_handlers_] = 0;
  count_handlers_++;  
}

// Allow us to add Port/Protocol pairs to the list... 
#if 0
bool cm904Controller::addPortProtocol(HardwareSerial &hSerial, uint8_t protocol, uint32_t baud) {
  if ((count_handlers_+1) >= MAX_PORT_PROTOL_HANDLERS) return false;

  if (portName != NULL) {
    portHandler[count_handlers_] = dynamixel::PortHandler::getPortHandler(portName);
    // Open port
    if (!portHandler[count_handlers_]->openPort()) {
      #ifdef DBGSerial
      DBGSerial.print("Failed to open Dynamixel port!\n");
      #endif
      return false;
    }
    if (!portHandler[count_handlers_]->setBaudRate(baud)) {
      #ifdef DBGSerial
      DBGSerial.print("Failed to change the Dynamixel baudrate!\n");
      #endif
      return false;
    }
  } else {
    portHandler[count_handlers_] = portHandler[count_handlers_-1]; // Use previous one. 
  }
  packetHandler[count_handlers_] = dynamixel::PacketHandler::getPacketHandler((protocol==2)? 2.0 : 1.0);
  port_packet_servo_count_[count_handlers_] = 0;
  count_handlers_++;    // update count... 
  return true;
}
#endif


bool cm904Controller::setId(int index, int id) {
  servo_info_t *servo = &servos_[index];
  servo->id = id;

  // This function will try to locate the servo and fill in
  uint8_t handler = findServo(id, servo);
  if (handler != 0xff) {
    // The find servo filled in type and handler, lets also clear out pose info
    servo->pose = servo->nextpose = (servo->type == SERVO_XL430)? 2048 : 512;
    return true;
  }
  return false;
}


uint8_t cm904Controller::findServo(uint8_t id, servo_info_t *pservo) { // if found returns the handler number, if not 0xff
  uint16_t servo_model;
  int error;
  for (uint8_t handler = 0; handler < count_handlers_; handler++) {
    // See if we can see this servo with this handler 
    error = packetHandler[handler]->read2ByteTxRx(portHandler[handler], id, 
        AX_MODEL_NUMBER_L, &servo_model);
    if (error == COMM_SUCCESS) {
      if (pservo) {  
        // caller passed in pointer to servo data structure so fill some of it in.
        // Also try to keep a count of how many servos are on different handler
        if (pservo->handler != 0xff) port_packet_servo_count_[pservo->handler]--;
        pservo->handler = handler;
        port_packet_servo_count_[pservo->handler]++;
        // Probably should look into more models, but found list of models /
        // in the Dynamixel Workbench library dynamixel_item.h
        //AX_12A(12), AX_12W(300), AX_18A(18)
        //MX_12W(360), MX_28(29). MX_28_2(30), MX_64(310), MX_64_2(311), MX_106(320),MX_106_2(321)
        //XL_320(350), XL430_W250(1060), XM430_W210(1030), ...
        // Maybe over time should look at how some of the others than AX12/18 and XL430 work
        pservo->type = (servo_model > 1000)? SERVO_XL430 : SERVO_AXMX;
        DBGSerial.print("Servo found:" );
        DBGSerial.print(id, DEC);
        DBGSerial.print(" handler: ");
        DBGSerial.print(handler, DEC);
        DBGSerial.print(" Type: ");
        DBGSerial.print(pservo->type);
        DBGSerial.print(" Count on handler: ");
        DBGSerial.println(port_packet_servo_count_[pservo->handler], DEC);
      }
      return handler;
    }
  }
  return 0xff;
}

int cm904Controller::getId(int index) {
  return servos_[index].id;
}

uint32_t cm904Controller::getServoPosition(uint8_t index, uint8_t &perror) {
  // We will use by index instead of ID. 
  servo_info_t *servo = &servos_[index];
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t pos2;
  uint32_t pos4;
  if (servo->handler == 0xff) {
    perror = 0xff;  // error out
    return 0;
  }
  switch (servo->type) {
    case SERVO_XL430:
      perror = packetHandler[servo->handler]->read4ByteTxRx(portHandler[servo->handler], servo->id, 
          DXL_X_PRESENT_POSITION, &pos4, &dxl_error);
      return pos4;

    case SERVO_AXMX:
    default:
      perror = packetHandler[servo->handler]->read2ByteTxRx(portHandler[servo->handler], servo->id, 
          AX_PRESENT_POSITION_L, &pos2, &dxl_error);
      return pos2;

  }

}

/* read in current servo positions to the pose. */
void cm904Controller::readPose() {
  servo_info_t * servo = servos_;
  uint8_t error;
  DBGSerial.print("ReadPose:");
  for (int i = 0; i < poseSize; i++) {
    // Read present position
    uint16_t pose = getServoPosition(i, error) << BIOLOID_SHIFT;
    if (error != COMM_SUCCESS) {
      // If we had an error reading servo, lets try locating it again.
      // Maybe will need to add error count or like
      uint8_t handler = findServo(servo->id, servo);
      if (handler == 0xff) {
        // Try again...
        pose = getServoPosition(i, error) << BIOLOID_SHIFT;
      }     
    }
    if (error != COMM_SUCCESS)
      pose = 0xffff;
    servo->pose = pose;
    DBGSerial.print(" ");
    DBGSerial.print(servo->id, DEC);
    DBGSerial.print(":");
    DBGSerial.print(pose >> BIOLOID_SHIFT, DEC);
    servo++;
  }
  DBGSerial.println();
}

#if 0
uint8_t cm904Controller::getServoByte(uint8_t id, uint8_t reg) {
  uint8_t val;
  uint8_t dxl_error = 0;                          // Dynamixel error
  packetHandler[0]->read1ByteTxRx(portHandler[0], id, reg, &val, &dxl_error);
  return val;
}

uint16_t cm904Controller::getServoWord(uint8_t id, uint8_t reg) {
  uint16_t val;
  uint8_t dxl_error = 0;                          // Dynamixel error
  int ret = packetHandler[0]->read2ByteTxRx(portHandler[0], id, reg, &val, &dxl_error);
  if (ret != COMM_SUCCESS) return 0xffff;
  return val;
}
#endif
uint32_t cm904Controller::getServoValue(uint8_t id, servo_logical_reg_t reg) {
  servo_info_t *servo = cm904Controller::mapIDtoServoInfo(id);
  DBGSerial.print("Get Servo Value, ID: ");
  DBGSerial.print(id, DEC);

  uint8_t servo_reg;
  uint8_t servo_reg_len;
  if (!servo) return (uint32_t)-1;
  if (servo->type == SERVO_XL430) {
    DBGSerial.print(" XL type ");
    servo_reg = g_xl430_regs[(uint8_t)reg];
    servo_reg_len = g_xl430_reg_len[(uint8_t)reg];
  } else {
    DBGSerial.print(" AX type ");
    servo_reg = g_axmx_regs[(uint8_t)reg];
    servo_reg_len = g_axmx_reg_len[(uint8_t)reg];
  }

  if (servo->handler == 0xff) return (uint32_t)-1;
  DBGSerial.print(" REG:");
  DBGSerial.print(servo_reg, DEC);
  DBGSerial.print(" Size: ");
  DBGSerial.print(servo_reg_len, DEC);
  // Now lets switch depending on size...
  uint8_t val;
  uint16_t val2;
  uint32_t val4;
  uint8_t error;
  int ret;
  switch (servo_reg_len) {
    default:
      ret = packetHandler[servo->handler]->read1ByteTxRx(portHandler[servo->handler], id, servo_reg, &val, &error); 
      if ((ret != COMM_SUCCESS) || (error != 0)) break;
      DBGSerial.print(" Val: ");
      DBGSerial.println(val, DEC);
      return val;
      break;
    case 2:
      ret = packetHandler[servo->handler]->read2ByteTxRx(portHandler[servo->handler], id, servo_reg, &val2, &error); 
      if ((ret != COMM_SUCCESS) || (error != 0)) break;
      DBGSerial.print(" Val: ");
      DBGSerial.println(val2, DEC);
      return val2;
      break;
    case 4:
      ret = packetHandler[servo->handler]->read4ByteTxRx(portHandler[servo->handler], id, servo_reg, &val4, &error); 
      if ((ret != COMM_SUCCESS) || (error != 0)) break;
      DBGSerial.print(" Val: ");
      DBGSerial.println(val4, DEC);
      return val4;
      break;
  }
  DBGSerial.print(" Error: ");
  DBGSerial.print(error, HEX);
  DBGSerial.print(" Ret: ");
  DBGSerial.println(ret, HEX);
  return (uint32_t)-1;
}



bool cm904Controller::setServoValue(uint8_t id, servo_logical_reg_t reg, uint32_t val) {
  servo_info_t *servo = cm904Controller::mapIDtoServoInfo(id);
  uint8_t servo_reg;
  uint8_t servo_reg_len;
  if (!servo) return false;
  if (servo->type == SERVO_XL430) {
    servo_reg = g_xl430_regs[(uint8_t)reg];
    servo_reg_len = g_xl430_reg_len[(uint8_t)reg];
  } else {
    servo_reg = g_axmx_regs[(uint8_t)reg];
    servo_reg_len = g_axmx_reg_len[(uint8_t)reg];
  }

  if (servo->handler == 0xff) return false;

  // Now lets switch depending on size...
  uint8_t error;
  int ret;
  switch (servo_reg_len) {
    default:
      ret = packetHandler[servo->handler]->write1ByteTxRx(portHandler[servo->handler], id, servo_reg, val, &error); 
      break;
    case 2:
      ret = packetHandler[servo->handler]->write2ByteTxRx(portHandler[servo->handler], id, servo_reg, val, &error); 
      break;
    case 4:
      ret = packetHandler[servo->handler]->write4ByteTxRx(portHandler[servo->handler], id, servo_reg, val, &error); 
      break;
        
  }
  return ((ret == COMM_SUCCESS) && (error == 0));
}

bool cm904Controller::setServoByte(uint8_t handler, uint8_t id, uint8_t reg, uint8_t val)
{
  uint8_t error;
  int ret = packetHandler[handler]->write1ByteTxRx(portHandler[handler], id, reg, val, &error); 
  return ((ret == COMM_SUCCESS) && (error == 0));
}

bool cm904Controller::setServoWord(uint8_t handler, uint8_t id, uint8_t reg, uint16_t val)
{
  uint8_t error;
  int ret = packetHandler[handler]->write2ByteTxRx(portHandler[handler], id, reg, val, &error); 
  return ((ret == COMM_SUCCESS) && (error == 0));
}

void cm904Controller::setTorqueAllservos(bool torqueOn) {
  servo_info_t * servo = servos_;
  uint8_t error;
  uint8_t val = torqueOn? 1 : 0;
  for (int i = 0; i < poseSize; i++)
  {
    uint8_t reg = (servo->type == SERVO_XL430)? DXL_X_TORQUE_ENABLE : AX_TORQUE_ENABLE;
    if (servo->handler != 0xff)
      packetHandler[servo->handler]->write1ByteTxRx(portHandler[servo->handler], servo->id, reg, val, &error); 
    servo++;
  }
}

//--------------------------------------------------------------------
//[SetRegOnAllServos] Function that is called to set the state of one
//  register in all of the servos, like Torque on...
//--------------------------------------------------------------------
/*
void cm904Controller::setRegOnAllServos(uint8_t bReg, uint8_t bVal)
{
  uint8_t *pb = bioloid_buffer_;
  for (int i = 0; i < poseSize; i++)
  {
    *pb++ = id_[i];
    *pb++ = bVal;
  }
  packetHandler[0]->syncWriteTxOnly(portHandler[0], bReg, 1, bioloid_buffer_, 2 * poseSize);
}
*/


//--------------------------------------------------------------------
// writePose: This is more complicated now as allow for multiple
// ports, with multiple protocols and multiple servo types... 
//--------------------------------------------------------------------

/* write pose out to servos using sync write. */
void cm904Controller::writePose() {
  uint16_t temp;

  for (uint8_t handler = 0; handler < count_handlers_; handler++) {
    if (port_packet_servo_count_[handler] == 0) continue; // No servos using this handler so 
    for (uint8_t servo_type = SERVO_AXMX; servo_type < SERVO_TYPE_COUNT; servo_type++) {
      uint8_t *pb = bioloid_buffer_;
      servo_info_t * servo = servos_;
      for (int i = 0; i < poseSize; i++)
      {
        if ((servo->handler == handler) && (servo->type == servo_type)) {
          *pb++ = servo->id;
          temp = servo->pose >> BIOLOID_SHIFT;
          *pb++ = (temp & 0xff);
          *pb++ = (temp >> 8);
          if (servo_type == SERVO_XL430) {
            *pb++ = 0;
            *pb++ = 0;
          }
        }
        servo++;
      }
      if (pb != bioloid_buffer_) {
        if (servo_type == SERVO_XL430) {
          packetHandler[handler]->syncWriteTxOnly(portHandler[handler], DXL_X_GOAL_POSITION, 4, 
              bioloid_buffer_, (uint16_t)(pb - bioloid_buffer_));
        } else {
          packetHandler[handler]->syncWriteTxOnly(portHandler[handler], AX_GOAL_POSITION_L, 2, 
              bioloid_buffer_, (uint16_t)(pb - bioloid_buffer_));
        }
      }

    }
  }
}

/* set up for an interpolation from pose to nextpose over TIME
  milliseconds by setting servo speeds. */
void cm904Controller::interpolateSetup(int time) {
  int i;
  int frames = (time / frameLength) + 1;
  nextframe_ = millis() + frameLength;
  // set speed each servo...
  servo_info_t * servo = servos_;
  for (i = 0; i < poseSize; i++) {
    if (servo->nextpose > servo->pose) {
      servo->speed = (servo->nextpose - servo->pose) / frames + 1;
    }
    else {
      servo->speed = (servo->pose - servo->nextpose) / frames + 1;
    }
    servo++;
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
  servo_info_t * servo = servos_;
  for (i = 0; i < poseSize; i++) {
    int diff = servo->nextpose - servo->pose;
    if (diff == 0) {
      complete--;
    }
    else {
      if (diff > 0) {
        if (diff < servo->speed) {
          servo->pose = servo->nextpose;
          complete--;
        }
        else
          servo->pose += servo->speed;
      }
      else {
        if ((-diff) < servo->speed) {
          servo->pose = servo->nextpose;
          complete--;
        }
        else
          servo->pose -= servo->speed;
      }
    }
    servo++;
  }
  if (complete <= 0) interpolating = 0;
  writePose();
  return 0;
}


int  cm904Controller::getNextpose(int id) {
  servo_info_t *servo = mapIDtoServoInfo(id);
  if (servo) {
    return (servo->nextpose >> BIOLOID_SHIFT);
  }
  return -1;
}

/* set a servo value in the next pose */
void cm904Controller::setNextPose(int id, int pos) {
  servo_info_t * servo = servos_;
  for (int i = 0; i < poseSize; i++) {
    if ( servo->id == id ) {
      servo->nextpose = (pos << BIOLOID_SHIFT);
      return;
    }
    servo++;
  }
}

//---------------------------------------------------------
// Move conversion from angle to value to here as to 
// make it easier to know which type servo
// Conversion factors for AX like servos 300 degrees (3000 in tenths) range 1024 Center 512 
int cm904Controller::setNextPoseAngle(int id, int angle) {   // set the servo value by angle 10ths degree.       
  servo_info_t * servo = mapIDtoServoInfo(id);
  if (servo) {
    int pos;
    if (servo->type == SERVO_XL430) pos  = (((long)(angle)) * cPwmMultXL) / cPwmDivXL + cPFConstXL;
    else pos  = (((long)(angle)) * cPwmMult) / cPwmDiv + cPFConst;
    servo->nextpose = (pos << BIOLOID_SHIFT);
    return pos;
  }
  return -1;
}

int cm904Controller::convertPosToAngle(int id, int uPos) {
  // Convert a position back to an angle in 10ths of degree
  servo_info_t * servo = mapIDtoServoInfo(id);
  if (servo) {
    if (servo->type == SERVO_XL430) 
      return (((int)uPos - cPFConstXL) * cPwmDivXL) / cPwmMultXL;
    else 
      return (((int)uPos - cPFConst) * cPwmDiv) / cPwmMult;
  }
  return 0;
}

servo_info_t *cm904Controller::mapIDtoServoInfo(int id) {
  servo_info_t * servo = servos_;
  for (int i = 0; i < poseSize; i++) {
    if ( servo->id == id ) return servo;
    servo++;
  }
  return NULL;
}
