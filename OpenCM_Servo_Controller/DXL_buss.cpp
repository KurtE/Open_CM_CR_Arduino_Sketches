//-----------------------------------------------------------------------------
// Functions for dealing with the DXL Buss.
//-----------------------------------------------------------------------------
#include "globals.h"
#include <EEPROM.h>
//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define DXL_DIR_1   28      // Direction pin for Serial1
#define DXL_DIR_3   22
//-----------------------------------------------------------------------------
// Globals.
//-----------------------------------------------------------------------------
_DXL_BUSS DXL_BUSS;
uint8_t  _DXL_BUSS::_dxl_buss;
uint8_t  _DXL_BUSS::_dxl_baud;
uint8_t  _DXL_BUSS::_direction_pin;
bool     _DXL_BUSS::_port_write_mode = false;


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
bool   _DXL_BUSS::openPort(uint8_t dxl_buss, uint8_t baud_index) {
  DBGPrintln("_DXL_BUSS::openPort"); DBGFlush();
  _dxl_buss = dxl_buss;
  _dxl_baud = baud_index;

  uint32_t dxl_baud;
  switch (baud_index) {
    case 0: dxl_baud = 9600; break;
    case 1: dxl_baud = 57600; break;
    case 2: dxl_baud = 115200; break;
    default:
    case 3: dxl_baud = 1000000; break;
    case 4: dxl_baud = 2000000; break;
    case 5: dxl_baud = 3000000; break;
    case 6: dxl_baud = 4000000; break;
    case 7: dxl_baud = 4500000; break;
  }

  DBGPrint("    Before begin: ");
  DBGPrintln2(dxl_baud, DEC);
  if (dxl_buss == 0) {
    Serial1.setDxlMode(true);
    Serial1.begin(dxl_baud);
    _direction_pin = DXL_DIR_1;
  } else {
    Serial3.setDxlMode(true);
    Serial3.begin(dxl_baud);
    _direction_pin = DXL_DIR_3;
  }
  DBGPrintln("    After begin");

  // Initialize the direction Pin
  DBGPrint("    Before pin mode: ");
  DBGPrintln2(_direction_pin, DEC);
  pinMode(_direction_pin, OUTPUT);
  digitalWrite(_direction_pin, LOW);
  _port_write_mode = false;

  return true;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void   _DXL_BUSS::closePort() {
  if (_dxl_buss)
    Serial3.end();
  else
    Serial1.end();
  _dxl_buss = 0;
}

//-----------------------------------------------------------------------------
// ProcessInput - Handle any data we receive on the DXL Buss
//    currently we just forward to USB...
//-----------------------------------------------------------------------------
bool   _DXL_BUSS::processInput() {
  // See if any data available from port actually the read takes care of MAX size plus empty

  bool return_value = false;
  int ch;
  for (;;) {
    ch = (_dxl_buss) ? Serial3.read() : Serial1.read();
    if (ch == -1)
      break;
    return_value = true;
    Serial.write((uint8_t)ch);
  }
  return return_value;
}

//-----------------------------------------------------------------------------
//  _DXL_BUSS::write - Make sure we are in write mode and then
//      do writes to DXL Buss.
//-----------------------------------------------------------------------------
void   _DXL_BUSS::write(uint8_t *pb, uint16_t cb) {
  // See if we are already in Write mode.
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

#ifdef DBGSerial
  DBGSerial.print("<");
  while (cb) {
    DBGSerial.printf(" %02x", *pb++);
    cb--;
  }
  DBGSerial.print(">");
#endif
}

//-----------------------------------------------------------------------------
// _DXL_BUSS::switchToInput - make sure we are in mode to receive any input
//    back from servos on AX Buss
//-----------------------------------------------------------------------------
void   _DXL_BUSS::switchToInput(void) {
  if (_port_write_mode) {
    // Note may speed up later...
    if (_dxl_buss)
      digitalWriteFast(DXL_DIR_3, LOW);
    else
      digitalWriteFast(DXL_DIR_1, LOW);
    _port_write_mode = false;
  }
}
