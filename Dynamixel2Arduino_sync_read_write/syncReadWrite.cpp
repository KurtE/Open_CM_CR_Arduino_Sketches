
#include "syncReadWrite.h"

extern "C" {
void dumpBuffer(const char *title, const uint8_t *pb, uint16_t count) {
  if (title) Serial.println(title);
  Serial.printf("0000: ");
  for (uint16_t i = 0; i < count; i++) {
    Serial.printf("%02x ", *pb++);
    if ((i & 0xf) == 0xf) Serial.printf("\n%04x: ", i+1);
    }
  Serial.println();
}

void printForC(const char *title, int value, int eol) {
  if (title) Serial.print(title);
  Serial.print(value, DEC);
  if (eol) Serial.println();
}
void printHexForC(const char *title, int value, int eol) {
  if (title) Serial.print(title);
  Serial.print(value, HEX);
  if (eol) Serial.println();
}

}

//=============================================================================
// SyncWrite class
//=============================================================================
SyncWrite::~SyncWrite() {
  if (_buffer_malloced) {
    free(_buffer);
  }
}


void SyncWrite::setBuffer(uint8_t *buffer, uint16_t buffer_size) {
  if (_buffer_malloced) {
    free(_buffer);
    _buffer_malloced = 0;
  }
  _buffer = buffer;
  _buffer_size = buffer_size;
}

// Lets allocate any buffers we need
// This assumes we have counts already setup...
bool SyncWrite::init() {
  if (!_buffer) {
    // We need to allocate it
    _buffer_size = suggestedBufferSize(_max_servos, _node_size);
    _buffer = (uint8_t*)malloc(_buffer_size);
    if (!_buffer) return false;
  }
  // should maybe verify that buffer size is valid for our counts, but...
  _init = 1;
  return true;
}

int SyncWrite::addID(uint8_t id) {
  if (!_init) init(); // make the call optional here...
  if (!_buffer_size) return -1;

  uint16_t buffer_index = 4; // Save at least enough room for Protocol 2
  uint8_t index = 0;

  // make sure not already in list
  for (; index < _cnt_servos; index++, buffer_index += _node_size + 1) {
    if (_buffer[buffer_index] == id)
      return index; // return the found location.
  }
  if (index >= _max_servos)
    return -1;    // no more room
  _cnt_servos++;  // increment count;

  _buffer[buffer_index] = id;
  return index;

}

int SyncWrite::setItem(uint8_t id, void *val,  uint16_t val_offset, uint16_t val_length) {
  return setItemByIndex(addID(id), val, val_offset, val_length);

}
int SyncWrite::setItemByIndex(int index, void *val,  uint16_t val_offset, uint16_t val_length) {
  if (val_length == 0xffff) val_length = _node_size;
  //Serial.printf("SyncWrite::setItemByIndex %d %d %d: ", index, val_offset, val_length);
  //dumpBuffer(NULL, (uint8_t*)val, val_length);
  if (!_buffer || (index < 0) || (index >= _cnt_servos) || ((val_offset + val_length) > _node_size)) return -1; // fail

  uint16_t buffer_index = 5 + (_node_size + 1) * index + val_offset;
  memcpy(&_buffer[buffer_index], val, val_length);
  return index;

}


bool  SyncWrite::send(Dynamixel2Arduino &dxl) {
  // We need to fill in the header information for ths sync write.
  if (!_buffer) return false;

  //uint32_t pre_time_us = micros();
  //lib_err_code_t error_code;

  if (dxl.getPortProtocolVersion() == 1.0) {
    //protocol 1
    _buffer[2] = _starting_addr & 0xff;
    _buffer[3] = _node_size & 0xff;

    return dxl.txInstPacket(DXL_BROADCAST_ID, DXL_INST_SYNC_READ, &_buffer[2], _cnt_servos * (_node_size + 1) + 2);
  }
  //protocol 2
  _buffer[0] = _starting_addr & 0xff;
  _buffer[1] = _starting_addr >> 8;
  _buffer[2] = _node_size & 0xff;
  _buffer[3] = _node_size >> 8;
  //dumpBuffer("TX SYNC_WRITE", _buffer, _cnt_servos * (_node_size + 1) + 4);
  return dxl.txInstPacket(DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, _buffer,  _cnt_servos * (_node_size + 1) + 4);
}

//=============================================================================
// SyncRead class
//=============================================================================
SyncRead::~SyncRead() {
  if (_buffer_malloced) {
    free(_buffer);
  }
}


void SyncRead::setBuffer(uint8_t *buffer, uint16_t buffer_size) {
  if (_buffer_malloced) {
    free(_buffer);
    _buffer_malloced = 0;
  }
  _buffer = buffer;
  _buffer_size = buffer_size;
}

// Lets allocate any buffers we need
// This assumes we have counts already setup...
bool SyncRead::init() {
  if (!_buffer) {
    // We need to allocate it
    _buffer_size = suggestedBufferSize(_max_servos, _node_size);
    _buffer = (uint8_t*)malloc(_buffer_size);
    if (!_buffer) return false;
  }
  // should maybe verify that buffer size is valid for our counts, but...
  _init = 1;
  return true;
}


int SyncRead::idToIndex(uint8_t id) { // converts an id to index does not add if not in list..
  if (!_init) init(); // make the call optional here...
  if (!_buffer_size) return -1;


  // make sure not already in list
  for (int index = 0; index < _cnt_servos; index++) {
    if (_buffer[index + 4] == id)
      return index; // return the found location.
  }
  return -1;
}


int SyncRead::addID(uint8_t id) {
  //Serial.printf("SyncRead::addID %d\n", id);
  int index = idToIndex(id);
  if (index >= 0) return index;

  if ((!_buffer_size) || (_cnt_servos >= _max_servos)) return -1;

  index = _cnt_servos++;  // increment count;

  _buffer[index + 4] = id;
  return index;

}

int SyncRead::idToRxIndex(uint8_t id) {
  uint8_t *pb = &_buffer[4 + _max_servos];
  for (uint8_t index = 0; index < _cnt_received; index++)  {
    if (*pb == id)
      return index;
    pb += 4 + _node_size;
  }
  return -1;
}

SyncReadReturnItem_t *SyncRead::retrieveItem(uint8_t id) {
  uint8_t *pb = &_buffer[4 + _max_servos];
  for (uint8_t index = 0; index < _cnt_received; index++)  {
    if (*pb == id)
      return (SyncReadReturnItem_t *)pb;
    pb += 4 + _node_size;
  }
  return nullptr;
}

SyncReadReturnItem_t *SyncRead::retrieveItemByIndex(uint8_t index) {
  if (index >= _cnt_received)
    return nullptr;
  return (SyncReadReturnItem_t *)(void*)(&_buffer[4 + _max_servos + index * (4 + _node_size)]);
}

uint8_t SyncRead::retrieveIDByIndex(uint8_t index) {
  SyncReadReturnItem_t *psrri = retrieveItemByIndex(index);
  return psrri ? psrri->id : 0xff;
}

uint8_t SyncRead::retrieveErrorByIndex(uint8_t index) {
  SyncReadReturnItem_t *psrri = retrieveItemByIndex(index);
  return psrri ? psrri->error : 0xff;
}

uint16_t SyncRead::retrieveLengthByIndex(uint8_t index) {
  SyncReadReturnItem_t *psrri = retrieveItemByIndex(index);
  return psrri ? psrri->length : 0;

}

bool SyncRead::retrieveValueByIndex(uint8_t index, void *val, uint16_t val_offset, uint16_t val_length) {
  SyncReadReturnItem_t *psrri = retrieveItemByIndex(index);
  if (!psrri) return false;
  if (val_length == 0xffff) val_length = _node_size;
  if ((val_offset + val_length) > _node_size) return false;
  memcpy(val, &psrri->data[val_offset], val_length);
  return true;
}

bool  SyncRead::doRead(Dynamixel2Arduino &dxl,  uint32_t timeout) {
  //Serial.printf("SyncRead::doRead buf:%x cnt:%d\n", _buffer, _cnt_servos);
  _cnt_received = 0;  // assume we have not received anything
  SyncReadReturnItem_t *psrri = (SyncReadReturnItem_t *)(&_buffer[4 + _max_servos]);
  // We need to fill in the header information for ths sync write.
  if (!_buffer || !_cnt_servos ) {
    dxl.setLastLibErrCode((DXLLibErrorCode)DXL_LIB_ERROR_NULLPTR);
    return false;
  }

  if (dxl.getPortProtocolVersion() == 1.0) {
    // Protocol 1 does not support this.
    Serial.println("Error protocol 1...");
    dxl.setLastLibErrCode((DXLLibErrorCode)DXL_LIB_ERROR_NOT_SUPPORTED);
    return false;
  }

  //protocol 2
  _buffer[0] = _starting_addr & 0xff;
  _buffer[1] = _starting_addr >> 8;
  _buffer[2] = _node_size & 0xff;
  _buffer[3] = _node_size >> 8;
  //dumpBuffer("TX SYNC_READ", _buffer, _cnt_servos + 4);
  if (!dxl.txInstPacket(DXL_BROADCAST_ID, DXL_INST_SYNC_READ, _buffer,  _cnt_servos + 4)) return false; // failed on tx

  uint32_t time_last_packet = millis();
  while (1)
  {
    const InfoToParseDXLPacket_t *prx = dxl.rxStatusPacket(psrri->data, _node_size);
    if (prx  && prx->inst_idx == DXL_INST_STATUS) {
      psrri->id     = prx->id;
      psrri->error  = prx->err_idx;
      psrri->length = _node_size;
      //Serial.printf("%x %x %d: ", prx->id, prx->error, prx->param_length);
      //dumpBuffer("RX SYNC_READ", prx->p_param, prx->param_length);
      //memcpy(psrri->data, prx->p_param, (psrri->length < _node_size) ? psrri->length : _node_size);

      _cnt_received++;
      psrri = (SyncReadReturnItem_t *)(((uint8_t*)psrri) + 4 + _node_size);
      if (_cnt_received >= _cnt_servos)
        return true;
      time_last_packet = millis();
    }

    if (millis() - time_last_packet >= timeout) {
      dxl.setLastLibErrCode((DXLLibErrorCode)DXL_LIB_ERROR_TIMEOUT);
      return false;
    }
  }
}
