#ifndef __globals_h__
#define __globals_h__

//=================================================================
// Include files
//=================================================================
#include <DynamixelSDK.h>

//=================================================================
// Definitions
//=================================================================
#define BUFFER_SIZE 1024      // Should be big enough
#define AX_SYNC_READ_MAX_DEVICES    120
#define DXL_MAX_RETURN_PACKET_SIZE   235
#define AX_ID_DEVICE        200    // Default ID
#define DBGSerial     Serial2
// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 // Check which port is being used on your controller

#ifdef DBGSerial
#define DBGPrint(x) DBGSerial.print(x)
#define DBGPrint2(x, y) DBGSerial.print(x, y)
#define DBGPrintln(x) DBGSerial.print(x)
#define DBGPrintln2(x, y) DBGSerial.print(x, y)
#else
#define DBGPrint(x) 
#define DBGPrint(x, y)
#define DBGPrintln(x)
#define DBGPrintln(x, y)
#endif
//=================================================================
// Global variables.
//=================================================================
extern dynamixel::PortHandler *portHandler;

extern uint8_t from_usb_buffer[BUFFER_SIZE];
extern uint16_t from_usb_buffer_count;
extern uint8_t from_port_buffer[BUFFER_SIZE];
extern unsigned long last_message_time;

//=================================================================
// External function definitions
//=================================================================
extern bool ProcessUSBInputData();
extern void MaybeFlushUSBOutputData();
extern void InitalizeHardwareAndRegisters();
extern void sendProtocol1StatusPacket(uint8_t err, uint8_t* data, uint8_t count_bytes);
extern void sendProtocol2StatusPacket(uint8_t err, uint8_t* data, uint16_t count_bytes);


#endif
