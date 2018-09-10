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
#define DBGSerial     Serial2


#define REG_TABLE_SIZE      400 // Aways up the register table... 
#define BUFFER_SIZE 1024      // Should be big enough
#define AX_SYNC_READ_MAX_DEVICES    120
#define DXL_MAX_RETURN_PACKET_SIZE   512
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
  CM904_RANDOM_NUMBER               = 77,   //P2 1rw
  CM904_GREEN_LED                   = 79,
  CM904_MOTION_LED                  = 82,
  CMS904_Port1_IR_Sensor            = 360, //P2 2r
  CMS904_Port4_IR_Sensor            = 366, //P2 2r
  CMS904_Port1_DMS_Sensor           = 368, //P2 2r
  CMS904_Port2_DMS_Sensor           = 370, //P2 2r
  CMS904_Port3_DMS_Sensor           = 372, //P2 2r
  CMS904_Port4_DMS_Sensor           = 374, //P2 2r
  CMS904_Port1_Touch_Sensor         = 376, //P2 1r
  CMS904_Port2_Touch_Sensor         = 377, //P2 1r
  CMS904_Port3_Touch_Sensor         = 378, //P2 1r
  CMS904_Port4_Touch_Sensor         = 379, //P2 1r
  CMS904_Port2_LED_Module           = 381, //P2 1rw
  CMS904_Port3_LED_Module           = 382, //P2 1rw
  CMS904_Port2_User_Device          = 386, //P2 2rw
  CMS904_Port3_User_Device          = 388, //P2 2rw
  CMS904_Port1_Temperature_Sensor   = 392, //P2 1r
  CMS904_Port2_Temperature_Sensor   = 393, //P2 1r
  CMS904_Port3_Temperature_Sensor   = 394, //P2 1r
  CMS904_Port4_Temperature_Sensor   = 395, //P2 1r
  CMS904_Port1_Ultrasonic_Sensor    = 396, //P2 1r
  CMS904_Port2_Ultrasonic_Sensor    = 397, //P2 1r
  CMS904_Port3_Ultrasonic_Sensor    = 398, //P2 1r
  CMS904_Port4_Ultrasonic_Sensor    = 399, //P2 1r
  CMS904_Port1_Magnetic_Sensor      = 400, //P2 1r
  CMS904_Port2_Magnetic_Sensor      = 401, //P2 1r
  CMS904_Port3_Magnetic_Sensor      = 402, //P2 1r
  CMS904_Port4_Magnetic_Sensor      = 403, //P2 1r
  CMS904_Port1_Motion_Sensor        = 404, //P2 1r
  CMS904_Port2_Motion_Sensor        = 405, //P2 1r
  CMS904_Port3_Motion_Sensor        = 406, //P2 1r
  CMS904_Port4_Motion_Sensor        = 407, //P2 1r
  CMS904_Port2_Color_Sensor         = 409, //P2 1r
  CMS904_Port3_Color_Sensor         = 410  //P2 1r
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
