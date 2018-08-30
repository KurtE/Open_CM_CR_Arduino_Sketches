#ifndef __globals_h__
#define __globals_h__

//=================================================================
// Include files
//=================================================================
#include <Arduino.h>
#include <EEPROM.h>

//=================================================================
// Definitions
//=================================================================
//#define DBGSerial     Serial2


#define BUFFER_SIZE 1024      // Should be big enough
#define AX_SYNC_READ_MAX_DEVICES    120
#define DXL_MAX_RETURN_PACKET_SIZE   235
#define AX_ID_DEVICE        200    // Default ID
// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 // Check which port is being used on your controller

//Dynamixel device Control table
#define AX_ID_DEVICE        200    // Default ID
#define AX_ID_BROADCAST     0xfe
#define MODEL_NUMBER_L      0x90  //  model #400 by E-manual
#define MODEL_NUMBER_H      0x01
#define FIRMWARE_VERSION    19  // Firmware version, needs to be updated with every new release
#define BAUD_RATE           0x03
#define BAUD_RATE_UART       2
#define RETURN_LEVEL         2
#define RETURN_DELAY         0
#define DYNAMIXEL_CHANNEL    0  // 0 = Serial1 and 1 = Serial3
#define REG_TABLE_SIZE      83

// Device register index
enum {
  CM904_MODEL_NUMBER_L              = 0,  // P1, P2
  CM904_MODEL_NUMBER_H              = 1,  // P1, P2
  CM904_FIRMWARE_VERSION            = 2,  // P1
  CM904_ID                          = 3,  // P1
  CM904_BAUD_RATE                   = 4,  // P1
  CM904_RETURN_DELAY_TIME           = 5,  // P1
  CM904_P2_FIRMWARE_VERSION         = 6,  // P2
  CM904_P2_ID                       = 7,
  CM904_P2_BAUD_RATE_UART           = 8,
  CM904_P2_RETURN_DELAY_TIME        = 9,
  CM904_STATUS_RETURN_LEVEL         = 10,
  CM904_BOOTLOADER_VERSION          = 11,
  CM904_P2_BAUD_RATE_BUS            = 12,
  CM904_DYNAMIXEL_CHANNEL           = 16,
  CM904_EEPROM_LAST_INDEX           = 16,
  CM904_BUTTON_STATUS               = 26,
  CM904_RANDOM_NUMBER               = 77,
  CM904_GREEN_LED                   = 79,
  CM904_MOTION_LED                  = 82
};



#ifdef DBGSerial
#define DBGPrint(x) DBGSerial.print(x)
#define DBGPrint2(x, y) DBGSerial.print(x, y)
#define DBGPrintln(x) DBGSerial.println(x)
#define DBGPrintln2(x, y) DBGSerial.println(x, y)
#define DBGFlush()          DBGSerial.flush();
#else
#define DBGPrint(x) 
#define DBGPrint2(x, y)
#define DBGPrintln(x)
#define DBGPrintln2(x, y)
#define DBGFlush()
#endif
//=================================================================
// Global variables.
//=================================================================
extern uint8_t from_usb_buffer[BUFFER_SIZE];
extern uint16_t from_usb_buffer_count;
extern uint8_t from_port_buffer[BUFFER_SIZE];
extern unsigned long last_message_time;

//=================================================================
// External function definitions and classes
//=================================================================
// In Main sketch.
extern void signal_abort(uint8_t error);

// USB Host device functions
extern bool ProcessUSBInputData();
extern void MaybeFlushUSBOutputData();
extern void InitalizeHardwareAndRegisters();
extern void pass_bytes(uint8_t nb_bytes);
extern void passBufferedDataToServos(void);
extern void sendProtocol1StatusPacket(uint8_t err, uint8_t* data, uint8_t count_bytes);
extern void sendProtocol2StatusPacket(uint8_t err, uint8_t* data, uint16_t count_bytes);

// DXL_Buss functions
class _DXL_BUSS {
public:
  static bool   openPort(uint8_t dxl_buss, uint8_t baud_index);
  static void   closePort();
  static bool   processInput();
  static void   write(uint8_t *pb, uint16_t cb);
  static void   switchToInput(void);

  // variables that probably don't need to be made public here, but...
  static uint8_t  _dxl_buss;
  static uint8_t   _dxl_baud;
  static uint8_t  _direction_pin;
  static bool     _port_write_mode;
};
extern _DXL_BUSS DXL_BUSS;

#endif
