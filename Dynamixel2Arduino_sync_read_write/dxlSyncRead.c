#include <Arduino.h>
#include "dxlSyncRead.h"

// BUGBUG: these need real C version, will hack up for now...
extern boolean DXLTxPacketInst(uint8_t id, uint8_t instruction, void* buffer, uint16_t length);
extern const InfoToParseDXLPacket_t *DXLRxStatusPacket(void *buffer, uint16_t length);

extern void dumpBuffer(const char *title, const uint8_t *pb, uint16_t count);
extern void printForC(const char *title, int value, int eol);
extern void printHexForC(const char *title, int value, int eol);

#define DXL_BROADCAST_ID        0xFE
#define INST_SYNC_READ  0x82
#define RX_PACKET_TYPE_STATUS   0
#define RX_PACKET_TYPE_INST     1


// BUGBUG setup for max size...
uint8_t g_syncRead_param[254 + 4];
uint16_t g_syncRead_cnt_ids = 0;
uint32_t syncRead_write_default_timeout = 10;

uint8_t syncRead(const uint8_t *p_param, uint16_t param_len, uint8_t id_cnt,
                 uint8_t *p_recv_buf, uint16_t recv_buf_capacity, uint32_t timeout)
{
  // We setup the initial area with starting address and the like...
  //dumpBuffer("TX SYNC_READ(c)", (void*)&SRData->starting_addr, SRData->cnt_servos + 4);
  // make sure buffer is big enough...
  if (!p_recv_buf || ((id_cnt * (param_len + 2) + 4) > recv_buf_capacity)) {
    return 0;   // no buffer or not big enough...
  }

  if (!DXLTxPacketInst(DXL_BROADCAST_ID, INST_SYNC_READ, (void*)p_param,  id_cnt + 4)) return 0; // failed on tx

  // Now we need to loop through retrieving the data.
  uint32_t time_last_packet = millis();
 
  // lets remember a few things at the start of the buffer... 
  p_recv_buf[0] = id_cnt;  // count we asked for
  p_recv_buf[1] = 0;     // count we received.
  p_recv_buf[2] = param_len & 0xff;
  p_recv_buf[3] = param_len >> 8;
  uint8_t *item_ptr = p_recv_buf + 4;
  while (1)
  {
    // Question is how to store the data in user buffer?
    // for now: ID, ERR, (maybe data len?) DATA
    const InfoToParseDXLPacket_t *prx = DXLRxStatusPacket(item_ptr + 2, param_len);
    if (prx  && prx->inst_idx == DXL_INST_STATUS) {
      item_ptr[0]     = prx->id;
      item_ptr[1]  = prx->err_idx;
      //Serial.printf("%x %x %d: ", prx->id, prx->error, prx->param_length);
      //dumpBuffer("RX SYNC_READ", prx->p_param, prx->param_length);

      p_recv_buf[1]++; // count received. 
      item_ptr += 2 + param_len;
      if (p_param[1] >= id_cnt)
        return id_cnt;
      time_last_packet = millis();
    }

    if (millis() - time_last_packet >= timeout) {
      //dxl.setLastLibErrCode((DYNAMIXEL::lib_err_code_t)DYNAMIXEL::DXL_LIB_ERROR_TIMEOUT);
      return p_recv_buf[1];
    }
  }
}
/* Easy functions for Sync Read */

bool beginSyncRead(uint16_t addr, uint16_t addr_len)
{
  g_syncRead_param[0] = addr & 0xff;
  g_syncRead_param[1] = addr  >> 8;
  g_syncRead_param[2] = addr_len & 0xff;
  g_syncRead_param[3] = addr_len >> 8;
  g_syncRead_cnt_ids = 0;
  return true;
}

bool addSyncReadID(uint8_t id)
{
  if (id == DXL_BROADCAST_ID) return false;
  g_syncRead_param[4 + g_syncRead_cnt_ids++] = id;
  return true;
}

uint8_t sendSyncRead(uint8_t *p_recv_buf, uint16_t recv_buf_capacity)
{
  return syncRead(g_syncRead_param, 4, g_syncRead_cnt_ids, p_recv_buf, recv_buf_capacity, syncRead_write_default_timeout);
}

int getSyncReadResultIDIndex(uint8_t id, uint8_t *p_recv_buf) {
  // We need to walk the list to find our item ID... 
  // Maybe we could keep pointer to last one we searched for... 
  uint16_t param_len = p_recv_buf[2] + p_recv_buf[3] << 8;
  uint8_t count_returned = p_recv_buf[1];
  uint8_t *item_ptr = p_recv_buf + 4;

  for (int index = 0; index < count_returned; index++) {
    if (item_ptr[0] == id) {
      return index;
    }
    item_ptr += 2 + param_len;
  }
  return -1;
}


uint8_t *getSyncReadResult(uint8_t index, uint8_t *p_recv_buf, uint8_t *err) {
  // We need to walk the list to find our item ID... 
  // Maybe we could keep pointer to last one we searched for... 
  uint16_t param_len = p_recv_buf[2] + p_recv_buf[3] << 8;
  uint8_t count_returned = p_recv_buf[1];

  if (index >= count_returned) {
    if (err) *err = 0xff; // set to ff as an indicator of not found...
    return NULL;     
  }
  
  uint8_t *item_ptr = p_recv_buf + 4 + (2 + param_len)*index;
  if (err) *err = item_ptr[1];
  return &item_ptr[2];
}   
 
int getSyncReadResultID(uint8_t index, uint8_t *p_recv_buf, uint8_t *err) {
  uint8_t *pitem_data = getSyncReadResult(index, p_recv_buf, err);
  if (pitem_data) return pitem_data[0];
  return -1;
}

// Note: I don't verify the offset is valid
uint8_t getSyncReadResult1(uint8_t index, uint8_t offset, uint8_t *p_recv_buf, uint8_t *err) {
  uint8_t *pitem_data = getSyncReadResult(index, p_recv_buf, err);
  if (pitem_data) return pitem_data[offset];
  return 0xff; // error
}

// Note: I don't verify we actually received enouth data.
uint16_t getSyncReadResult2(uint8_t index, uint8_t offset, uint8_t *p_recv_buf, uint8_t *err) {
  uint8_t *pitem_data = getSyncReadResult(index, p_recv_buf, err);
  if (pitem_data) return pitem_data[offset] + (uint16_t)(pitem_data[offset + 1] << 8);
  return (uint16_t)-1; // error
}

uint32_t getSyncReadResult4(uint8_t index, uint8_t offset, uint8_t *p_recv_buf, uint8_t *err) {
  uint8_t *pitem_data = getSyncReadResult(index, p_recv_buf, err);
  if (pitem_data) return pitem_data[offset] + (uint32_t)(pitem_data[offset + 1] << 8)
                           + (uint32_t)(pitem_data[offset + 2] << 16) 
                           + (uint32_t)(pitem_data[offset + 3] << 24);
  return (uint32_t)-1; // error
}
