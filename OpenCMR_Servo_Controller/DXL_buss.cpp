//-----------------------------------------------------------------------------
// Functions for dealing with the DXL Buss.
//-----------------------------------------------------------------------------
#include "globals.h"
#include <EEPROM.h>
//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#if defined(__OPENCR__)
//#define DXL_DIR_3 BDPIN_DXL_DIR
#elif defined(__OPENCM904__)
#define DXL_DIR_1   28      // Direction pin for Serial1
#define DXL_DIR_3   22
#endif

//-----------------------------------------------------------------------------
// Globals.
//-----------------------------------------------------------------------------
_DXL_BUSS DXL_BUSS;
uint8_t  _DXL_BUSS::_dxl_buss;
uint8_t  _DXL_BUSS::_dxl_baud;
uint8_t  _DXL_BUSS::_direction_pin;
bool     _DXL_BUSS::_port_write_mode = false;

//-----------------------------------------------------------------------------
// convertBaudIndex
//-----------------------------------------------------------------------------
uint32_t _DXL_BUSS::convertBaudIndex(uint8_t baud_index) {
  switch (baud_index) {
    case 0: return 9600;
    case 1: return 57600;
    case 2: return 115200;
    default:
    case 3: return 1000000;
    case 4: return 2000000;
    case 5: return 3000000;
    case 6: return 4000000;
    case 7: return 4500000;
  }
}

//-----------------------------------------------------------------------------
// openPort
//-----------------------------------------------------------------------------
bool   _DXL_BUSS::openPort(uint8_t dxl_buss, uint8_t baud_index) {
  DBGPrintln("_DXL_BUSS::openPort"); DBGFlush();
  _dxl_buss = dxl_buss;
  _dxl_baud = baud_index;

  uint32_t dxl_baud = convertBaudIndex(baud_index);
  DBGPrint("    Before begin: "); DBGFlush();
  DBGPrintln2(dxl_baud, DEC);
#if defined(__OPENCM904__)
  if (dxl_buss == 0) {
    Serial1.setDxlMode(true);
    Serial1.begin(dxl_baud);
    _direction_pin = DXL_DIR_1;
  } else {
    Serial3.setDxlMode(true);
    Serial3.begin(dxl_baud);
    _direction_pin = DXL_DIR_3;
  }
  // Initialize the direction Pin
  DBGPrint("    Before pin mode: ");
  DBGPrintln2(_direction_pin, DEC);
  pinMode(_direction_pin, OUTPUT);
  digitalWrite(_direction_pin, LOW);
  _port_write_mode = false;


#elif defined(__OPENCR__)
  Serial3.begin(dxl_baud);
  drv_dxl_init(); // make sure we init the DXL Direction pin
  drv_dxl_tx_enable(false); // Set into read mode

//  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
//  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);

#endif
  DBGPrintln("    After begin"); DBGFlush();

  return true;
}

//-----------------------------------------------------------------------------
// closePort - Close out the serial
//-----------------------------------------------------------------------------
void   _DXL_BUSS::closePort() {
#if defined(__OPENCM904__) 
  if (_dxl_buss)
    Serial3.end();
  else
    Serial1.end();
  _dxl_buss = 0;
 #elif defined(__OPENCR__)
  Serial3.end();
 #endif 
}

//-----------------------------------------------------------------------------
//  setBaudRate - Update the baud rate.
//-----------------------------------------------------------------------------
void   _DXL_BUSS::setBaudRate(uint8_t baud_index) {
  DBGPrintln("_DXL_BUSS::setBaudRate"); DBGFlush();
  _dxl_baud = baud_index;
  uint32_t dxl_baud = convertBaudIndex(baud_index);
#if defined(__OPENCM904__) 
  if (_dxl_buss == 0) {
    Serial1.begin(dxl_baud);
  } else {
    Serial3.begin(dxl_baud);
  }
 #elif defined(__OPENCR__)
  Serial3.begin(dxl_baud);
 #endif 

}

//-----------------------------------------------------------------------------
// ProcessInput - Handle any data we receive on the DXL Buss
//    currently we just forward to USB...
//-----------------------------------------------------------------------------
bool   _DXL_BUSS::processInput() {
  // See if any data available from port actually the read takes care of MAX size plus empty
  DBGDigitalWrite(1, HIGH);
  bool return_value = false;
  int ch;
  for (;;) {
#if defined(__OPENCM904__) 
    ch = (_dxl_buss) ? Serial3.read() : Serial1.read();
#elif defined(__OPENCR__)
    ch = Serial3.read();
 #endif 

    if (ch == -1)
      break;
    return_value = true;
    Serial.write((uint8_t)ch);
  }
  DBGDigitalWrite(1, LOW);
  return return_value;
}

//-----------------------------------------------------------------------------
//  _DXL_BUSS::write - Make sure we are in write mode and then
//      do writes to DXL Buss.
//-----------------------------------------------------------------------------
void   _DXL_BUSS::write(uint8_t *pb, uint16_t cb) {
#ifdef __OPENCM904__
  // See if we are already in Write mode.
  DBGDigitalWrite(4, HIGH);
  if (!_port_write_mode) {
    if (_dxl_buss) {
      digitalWriteFast(DXL_DIR_3, HIGH);
    } else {
      digitalWriteFast(DXL_DIR_1, HIGH);
    }
    _port_write_mode = true;
  }
  if (_dxl_buss) {
    Serial3.write(pb, cb);
  } else {
    Serial1.write(pb, cb);
  }
#elif defined(__OPENCR__)
  // See if we are already in Write mode.
  DBGDigitalWrite(4, HIGH);
  if (!_port_write_mode) {
    DBGDigitalWrite(3, HIGH);
    drv_dxl_tx_enable(true);
    _port_write_mode = true;
  }
  Serial3.write(pb, cb);
#endif

#ifdef DBGSerial
  if (print_debug_outputs) {
    DBGSerial.print("<");
    while (cb) {
      DBGSerial.print(" ");
      DBGSerial.print(*pb++, HEX);
      cb--;
    }
    DBGSerial.print(">");
  }
#endif
  DBGDigitalWrite(4, LOW);
}

//-----------------------------------------------------------------------------
// _DXL_BUSS::switchToInput - make sure we are in mode to receive any input
//    back from servos on AX Buss
//-----------------------------------------------------------------------------
void   _DXL_BUSS::switchToInput(void) {
#if defined(__OPENCM904__)
  if (_dxl_buss) {
    Serial3.flush();
  } else {
    Serial1.flush();
  }
  if (_port_write_mode) {
    // Note may speed up later...
    if (_dxl_buss)
      digitalWriteFast(DXL_DIR_3, LOW);
    else
      digitalWriteFast(DXL_DIR_1, LOW);
    _port_write_mode = false;
  }
#elif defined(__OPENCR__)
  Serial3.flush();
  if (_port_write_mode) {
    drv_dxl_tx_enable(false);
    _port_write_mode = false;
  }
  DBGDigitalWrite(3, LOW);
#endif
}
