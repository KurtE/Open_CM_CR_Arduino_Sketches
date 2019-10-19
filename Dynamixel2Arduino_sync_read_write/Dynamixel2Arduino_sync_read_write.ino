/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>
#include "syncReadWrite.h"
#include "dxlSYncRead.h"

#define DXL_SERIAL   Serial1 //OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 28; //OpenCM9.04 EXP Board's DIR PIN. (To use the DXL port on the OpenCM 9.04 board, you must use 28 for DIR PIN.)

const uint8_t ServoIDList[] = {19, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12};
#define DXL_X_GOAL_POSITION        116 // 4
#define DXL_X_PRESENT_POSITION     132 // 4 (R)
#define DXL_X_PRESENT_INPUT_VOLTAGE  144 // 2 (R)
#define DXL_X_PRESENT_TEMPERATURE  146 // 1 (R)
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

SyncWrite syncwrite(DXL_X_GOAL_POSITION, 4, sizeof(ServoIDList));  // setup for a sync write starting at address 116, for 4 bytes for 2 servos.
SyncRead  syncread(DXL_X_PRESENT_POSITION, 4, sizeof(ServoIDList));  // setup to do sync read on 128, 4 bytes 2 servos

DXLSyncReadDataHeader_t *syncread_c = nullptr;

void setup() {
  // put your setup code here, to run once:
  while (!DEBUG_SERIAL) ;
  DEBUG_SERIAL.begin(115200);
#if defined(BOARD_OpenCM904) && (DXL_SERIAL == Serial1)
  Serial.println("*** OpenCM9.04 setDxlMode ***");
  Serial1.setDxlMode(true);
#endif

  dxl.begin(1000000); // Working with the XL430-W250
  //  dxl.scan();
  XelInfoFromPing_t ping_info;
  delay(250);

  // create a C Sync read object
  syncread_c = DXLSyncReadInit(DXL_X_PRESENT_INPUT_VOLTAGE, 3, sizeof(ServoIDList), nullptr, 0);

  // fill the members of structure for syncWrite
  //syncwrite.init(); // initialize buffers addID will  automatically call this as well
  for (uint8_t i = 0; i < sizeof(ServoIDList); i++) {
    if (dxl.ping(ServoIDList[i], &ping_info, 1)) {
      DEBUG_SERIAL.printf("  %d Type:%x Ver:%x\n", ServoIDList[i],
                    ping_info.model_number, ping_info.firmware_version);
    }
    dxl.torqueOff(ServoIDList[i]);
    dxl.setOperatingMode(ServoIDList[i], OP_POSITION);
    dxl.torqueOn(ServoIDList[i]);
    syncwrite.addID(ServoIDList[i]);
    syncread.addID(ServoIDList[i]);
    DXLSyncReadAddID(syncread_c, ServoIDList[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static int32_t position = 2048;
  static int32_t position_increment = 16;
  static const int32_t MAX_POS_DELTA = 256;
  int32_t recv_position;

  // set value to data buffer for syncWrite
  position += position_increment;
  if ((position >= (2048 + MAX_POS_DELTA)) || (position <= (2048 - MAX_POS_DELTA))) {
    position_increment = -position_increment;  // reverse direction
  }
  for (uint8_t i = 0; i < sizeof(ServoIDList); i++) {
    syncwrite.setItemByIndex(i, &position);
  }
  syncwrite.send(dxl);
  delay(100);

  // Print the read data using SyncRead
  DEBUG_SERIAL.println(F("======= Sync Read ======="));
  bool succeeded = syncread.doRead(dxl);
  if (succeeded) {
    DEBUG_SERIAL.print(F("Succeeded"));
  } else {
    DEBUG_SERIAL.print(F("Error status code: "));
    DEBUG_SERIAL.print(dxl.getLastLibErrCode(), DEC);
  }
  DEBUG_SERIAL.print(F(" Items Returned: "));
  DEBUG_SERIAL.println(syncread.receiveCount(), DEC);

  for (uint8_t index = 0; index < syncread.receiveCount(); index++) {
    DEBUG_SERIAL.print(F("Index: ")); DEBUG_SERIAL.print(index); DEBUG_SERIAL.print(" ");
    uint8_t id = syncread.retrieveIDByIndex(index);
    DEBUG_SERIAL.print(F("ID: ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.print(" ");
    syncread.retrieveValueByIndex(index, &recv_position);
    DEBUG_SERIAL.print(F(", Present Velocity: ")); DEBUG_SERIAL.print(recv_position); DEBUG_SERIAL.print(" ");
    uint8_t error = syncread.retrieveErrorByIndex(index);
    DEBUG_SERIAL.print(F(", Packet Error: ")); DEBUG_SERIAL.print(error); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(F(", Param Length: ")); DEBUG_SERIAL.print(syncread.retrieveLengthByIndex(index)); DEBUG_SERIAL.print(" ");
    if (error & 0x80) {
      // Hardware error, try to reset the servo...
      uint8_t hw_error;
      dxl.read(id, 70, 1, &hw_error, 1, 100); 
      DEBUG_SERIAL.print(F(", HW error:"));  DEBUG_SERIAL.print(hw_error); DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(F(" *** reboot Servo ***"));
      dxl.torqueOff(id);
      dxl.reboot(id, 25);
      dxl.torqueOn(id);
      
    }
    
    DEBUG_SERIAL.println();
    
  }
  // Now try a C version of Sync Read
  DEBUG_SERIAL.println(F("======= Sync Read C Version ======="));
  DXLSyncReadSendReceive(syncread_c, 100);
  DEBUG_SERIAL.print(F(" Items Returned: "));
  DEBUG_SERIAL.println(DXLSyncReadreceiveCount(syncread_c), DEC);
  for (uint8_t index = 0; index < DXLSyncReadreceiveCount(syncread_c); index++) {
    DEBUG_SERIAL.print(F("Index: ")); DEBUG_SERIAL.print(index); DEBUG_SERIAL.print(" ");
    uint16_t wVoltage;
    uint8_t bTemp;


    DEBUG_SERIAL.print(F("ID: ")); DEBUG_SERIAL.print(DXLSyncReadRetrieveIDByIndex(syncread_c, index)); DEBUG_SERIAL.print(" ");
    DXLSyncReadRetrieveValueByIndex(syncread_c, index, &wVoltage, 0, 2);
    DXLSyncReadRetrieveValueByIndex(syncread_c, index, &bTemp, 2, 1);
    DEBUG_SERIAL.print(F(", Voltage: ")); DEBUG_SERIAL.print(wVoltage); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(F(", Temp: ")); DEBUG_SERIAL.print(bTemp); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(F(", Packet Error: ")); DEBUG_SERIAL.print(DXLSyncReadRetrieveErrorByIndex(syncread_c, index)); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(F(", Param Length: ")); DEBUG_SERIAL.print(DXLSyncReadRetrieveLengthByIndex(syncread_c, index)); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println();
    
  }
  
  Serial.println("Press any key to continue");
  while (!Serial.available());
  while (Serial.read() != -1) ;
  //  delay(1000);

}

// BUGBUG: these need real C version, will hack up for now...
extern "C" {
  const dxl_packet_t *DXLRxPacket() {
    return (const dxl_packet_t *)dxl.rxPacket();
  }

  boolean DXLTxPacketInst(uint8_t id, uint8_t instruction, void* buffer, uint16_t length) {
    return dxl.txPacketInst(id, instruction, (uint8_t*)buffer, length);
  }

}
