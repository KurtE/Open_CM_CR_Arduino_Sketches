//-----------------------------------------------------------------------------
// Functions for dealing with the DXL Buss.
//-----------------------------------------------------------------------------
#include "globals.h"
#include <EEPROM.h>
//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#if defined(__OPENCR__)
#define DXL_DIR_3 BDPIN_DXL_DIR
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
    Serial1.transmitterEnable(DXL_DIR_1);
    _direction_pin = DXL_DIR_1;
  } else {
    Serial3.setDxlMode(true);
    Serial3.begin(dxl_baud);
    Serial3.transmitterEnable(DXL_DIR_3);
    _direction_pin = DXL_DIR_3;
  }
#elif defined(__OPENCM904__)
  Serial3.begin(dxl_baud);
  Serial3.transmitterEnable(DXL_DIR_3);
  _direction_pin = DXL_DIR_3;

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);

#endif
  DBGPrintln("    After begin"); DBGFlush();

  // Initialize the direction Pin
#if 0
  DBGPrint("    Before pin mode: ");
  DBGPrintln2(_direction_pin, DEC);
  pinMode(_direction_pin, OUTPUT);
  digitalWrite(_direction_pin, LOW);
  _port_write_mode = false;
#endif
  return true;
}

//-----------------------------------------------------------------------------
// closePort - Close out the serial 
//-----------------------------------------------------------------------------
void   _DXL_BUSS::closePort() {
  if (_dxl_buss)
    Serial3.end();
  else
    Serial1.end();
  _dxl_buss = 0;
}

//-----------------------------------------------------------------------------
//  setBaudRate - Update the baud rate. 
//-----------------------------------------------------------------------------
void   _DXL_BUSS::setBaudRate(uint8_t baud_index) {
  DBGPrintln("_DXL_BUSS::setBaudRate"); DBGFlush();
  _dxl_baud = baud_index;
  uint32_t dxl_baud = convertBaudIndex(baud_index);
  if (_dxl_buss == 0) {
    Serial1.begin(dxl_baud);
  } else {
    Serial3.begin(dxl_baud);
  }
  
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
  digitalWriteFast(4, HIGH);
#if 0
  if (!_port_write_mode) {
    if (_dxl_buss) {
      digitalWriteFast(DXL_DIR_3, HIGH);
    } else {
      digitalWriteFast(DXL_DIR_1, HIGH);
    }
    _port_write_mode = true;
  }
#endif  
  if (_dxl_buss) {
    Serial3.write(pb, cb);
  } else {
    Serial1.write(pb, cb);
  }

#ifdef DBGSerial
  DBGSerial.print("<");
  while (cb) {
    DBGSerial.print(" ");
    DBGSerial.print(*pb++, HEX);
    cb--;
  }
  DBGSerial.print(">");
#endif
  digitalWriteFast(4, LOW);
}

//-----------------------------------------------------------------------------
// _DXL_BUSS::switchToInput - make sure we are in mode to receive any input
//    back from servos on AX Buss
//-----------------------------------------------------------------------------
void   _DXL_BUSS::switchToInput(void) {
  if (_dxl_buss) {
    Serial3.flush();
  } else {
    Serial1.flush();
  }
#if 0
  if (_port_write_mode) {
    // Note may speed up later...
    if (_dxl_buss)
      digitalWriteFast(DXL_DIR_3, LOW);
    else
      digitalWriteFast(DXL_DIR_1, LOW);
    _port_write_mode = false;
  }
#endif  
}
