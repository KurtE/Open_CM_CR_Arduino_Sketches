#ifndef __DXL_SYNC_READ_H__
#define __DXL_SYNC_READ_H__
//#include <Dynamixel2Arduino.h>
#ifdef __cplusplus
extern "C"{
#endif

typedef struct {
	uint8_t 	id;
	uint8_t		error;
	uint16_t	length;
	uint8_t		data[0];
} __attribute__((packed)) DXLSyncReadReturnItem_t;

typedef struct {
	uint8_t			max_servos;
	uint8_t 		cnt_servos;
	uint8_t			cnt_received;
	uint16_t	 	offset_received_data;
	// Next parts are the start of the SYNCREAD send message
	uint16_t		starting_addr;
	uint16_t		node_size;
	uint8_t         ids[0];
	// DXLSyncReadReturnItem_t retrieved_items[max_servos];

	// receive data
} __attribute__((packed)) DXLSyncReadDataHeader_t;

// HACKS to get DXL Data.
  typedef struct DxlPacket
  {
    uint8_t   header[3];
    uint8_t   reserved;
    uint8_t   id;
    uint8_t   cmd;
    uint8_t   error;
    uint8_t   type;
    uint16_t  index;
    uint16_t  packet_length;
    uint16_t  param_length;
    uint16_t  crc;
    uint16_t  crc_received;
    uint8_t   check_sum;
    uint8_t   check_sum_received;
    uint8_t   *p_param;
    uint32_t  dummy;
    uint8_t   data[0];
  } dxl_packet_t;



#define DXLSyncReadBufferSize(NODE_SIZE, CNT_SERVOS) (sizeof(DXLSyncReadDataHeader_t) + CNT_SERVOS + (sizeof(DXLSyncReadReturnItem_t)+(NODE_SIZE)) * (CNT_SERVOS))

//extern DXLSyncReadDataHeader_t *DXLSyncReadInit(uint16_t starting_addr, uint16_t node_size, uint8_t max_servos);
extern DXLSyncReadDataHeader_t *DXLSyncReadInit(uint16_t starting_addr, uint16_t node_size, uint8_t max_servos, void *buffer, uint16_t buffer_size) ;

// Could add in lots of query and set functions... 
extern int DXLSyncReadAddID(DXLSyncReadDataHeader_t *SRData, uint8_t id);

extern int DXLSyncReadSendReceive(DXLSyncReadDataHeader_t *SRData, uint32_t timeout);

extern DXLSyncReadReturnItem_t *DXLSyncReadRetrieveItem(DXLSyncReadDataHeader_t *SRData, uint8_t id);
extern uint8_t DXLSyncReadreceiveCount(DXLSyncReadDataHeader_t *SRData);
extern uint8_t DXLSyncReadRetrieveIDByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index);
extern uint8_t DXLSyncReadRetrieveErrorByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index);
extern uint16_t DXLSyncReadRetrieveLengthByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index);
extern int DXLSyncReadRetrieveValueByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index, void *val, uint16_t val_offset, uint16_t val_length);
#ifdef __cplusplus
} // extern c
#endif

#endif //__DXL_SYNC_READ_H__
