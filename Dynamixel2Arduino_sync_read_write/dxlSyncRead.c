
#include <Arduino.h>
#include "dxlSyncRead.h"


// BUGBUG: these need real C version, will hack up for now...
extern const dxl_packet_t *DXLRxPacket();
extern boolean DXLTxPacketInst(uint8_t id, uint8_t instruction, void* buffer, uint16_t length);
extern void dumpBuffer(const char *title, const uint8_t *pb, uint16_t count);
extern void printForC(const char *title, int value, int eol);
extern void printHexForC(const char *title, int value, int eol);

#define DXL_BROADCAST_ID        0xFE
#define INST_SYNC_READ  0x82
#define RX_PACKET_TYPE_STATUS   0
#define RX_PACKET_TYPE_INST     1



//DXLSyncReadDataHeader_t *DXLSyncReadInit(uint16_t starting_addr, uint16_t node_size, uint8_t max_servos) {
//	return DXLSyncReadInit(starting_addr, node_size, max_servos, NULL, 0);
//}

DXLSyncReadDataHeader_t *DXLSyncReadInit(uint16_t starting_addr, uint16_t node_size, uint8_t max_servos, void *buffer, uint16_t buffer_size) 
{
	uint16_t size_required = sizeof(DXLSyncReadDataHeader_t) + max_servos + sizeof(DXLSyncReadReturnItem_t) * max_servos;
	if (buffer && (size_required > buffer_size))
		return NULL; // not big enough;
	if (!buffer) {
		buffer = malloc(size_required);
		if (!buffer) 
			return NULL;  // it failed to allocat
	}
	// We will remember the last allocated version to use as default
	DXLSyncReadDataHeader_t *SRData = (DXLSyncReadDataHeader_t *)buffer;

	// Ok Lets initialize the memory
	SRData->max_servos = max_servos;
	SRData->cnt_servos = 0;
	SRData->cnt_received = 0;
	SRData->offset_received_data = sizeof(DXLSyncReadDataHeader_t) + max_servos;
	SRData->starting_addr = starting_addr;
	SRData->node_size = node_size;

    //printHexForC("DXLSyncReadInit SRData:", (uint32_t)SRData, 0);printForC(" Header size: ", sizeof(DXLSyncReadDataHeader_t),0); printForC(" Offset: ", SRData->offset_received_data, 1); 
  	//dumpBuffer(NULL, (uint8_t*)SRData, sizeof(DXLSyncReadDataHeader_t));

	return SRData; 
}



// Could add in lots of query and set functions... 
int DXLSyncReadAddID(DXLSyncReadDataHeader_t *SRData, uint8_t id) {
	uint8_t index;

	if (!SRData) return -1;	// we have not init this data yet...
	for (index = 0; index < SRData->cnt_servos; index++) {
		if (SRData->ids[index] == id)
			return index;
	}
	if ( SRData->cnt_servos >=  SRData->max_servos)
		return -1;
	index = SRData->cnt_servos++;

	SRData->ids[index] = id;
	return index;
}

int DXLSyncReadSendReceive(DXLSyncReadDataHeader_t *SRData, uint32_t timeout) {
	if (!SRData) return 0;	// we have not init this data yet...

	SRData->cnt_received = 0;  // assume we have not received anything

	// We need to fill in the header information for ths sync write.
	if (!SRData->cnt_servos ) {
		//dxl.setLastLibErrCode((DYNAMIXEL::lib_err_code_t)DYNAMIXEL::DXL_LIB_ERROR_NULLPTR);
		return 0;
  	}
/*
  if (dxl.getPortProtocolVersion() == 1.0) {
    // Protocol 1 does not support this.
    Serial.println("Error protocol 1...");
    dxl.setLastLibErrCode((DYNAMIXEL::lib_err_code_t)DYNAMIXEL::DXL_LIB_ERROR_NOT_SUPPORTED);
    return false;
  }
*/

	//protocol 2
	// We setup the initial area with starting address and the like...
	//dumpBuffer("TX SYNC_READ(c)", (void*)&SRData->starting_addr, SRData->cnt_servos + 4);
	if (!DXLTxPacketInst(DXL_BROADCAST_ID, INST_SYNC_READ, (void*)&SRData->starting_addr,  SRData->cnt_servos + 4)) return false; // failed on tx

	uint32_t time_last_packet = millis();
	DXLSyncReadReturnItem_t *psrri = (DXLSyncReadReturnItem_t *)((uint8_t*)SRData + SRData->offset_received_data);
	while (1)
	{
		const dxl_packet_t *prx = DXLRxPacket();
		if (prx  && prx->type == RX_PACKET_TYPE_STATUS) {
			psrri->id     = prx->id;
			psrri->error  = prx->error;
			psrri->length = prx->param_length;
		  	//Serial.printf("%x %x %d: ", prx->id, prx->error, prx->param_length);
		    //printHexForC(NULL, (uint32_t)psrri, 0);printForC(" ", prx->id, 0); printForC(" ", prx->error, 0); printForC(" ", prx->param_length, 1);
		  	//dumpBuffer("RX SYNC_READ", prx->p_param, prx->param_length);
		  	memcpy(psrri->data, prx->p_param, (psrri->length < SRData->node_size) ? psrri->length : SRData->node_size);

		  	SRData->cnt_received++;
		  	psrri = (DXLSyncReadReturnItem_t *)(((uint8_t*)psrri) + 4 + SRData->node_size);
		  	if (SRData->cnt_received >= SRData->cnt_servos) {
				//dumpBuffer("TX SYNC_READ(c)", ((uint8_t*)SRData + SRData->offset_received_data),(uint32_t)psrri - (uint32_t)((uint8_t*)SRData + SRData->offset_received_data));
		    	return  SRData->cnt_servos;
		  	}
		  	time_last_packet = millis();
		}

		if (millis() - time_last_packet >= timeout) {
	  		//dxl.setLastLibErrCode((DYNAMIXEL::lib_err_code_t)DYNAMIXEL::DXL_LIB_ERROR_TIMEOUT);
	 	 	return SRData->cnt_servos;
		}
	}
}


uint8_t DXLSyncReadreceiveCount(DXLSyncReadDataHeader_t *SRData) {
	return SRData? SRData->cnt_received : 0;
}

DXLSyncReadReturnItem_t *DXLSyncReadretrieveItem(DXLSyncReadDataHeader_t *SRData, uint8_t id) {
	if (!SRData) return NULL;

	uint8_t *pb = ((uint8_t*)SRData + SRData->offset_received_data);
  	for (uint8_t index = 0; index < SRData->cnt_received; index++)  {
    	if (*pb == id)
      		return (DXLSyncReadReturnItem_t *)pb;
    	pb += 4 + SRData->node_size;
    }
    return NULL;
}

DXLSyncReadReturnItem_t *DXLSyncReadretrieveItemByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index) {
	if (!SRData) return NULL;
	if (index >= SRData->cnt_received) return NULL;
	return (DXLSyncReadReturnItem_t *)((uint8_t*)SRData + SRData->offset_received_data + index*(4 + SRData->node_size));
}

uint8_t DXLSyncReadRetrieveIDByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index) {
	DXLSyncReadReturnItem_t *psrri = DXLSyncReadretrieveItemByIndex(SRData, index);
	printHexForC("psrri: ", (uint32_t)psrri, 0);
	return psrri ? psrri->id : 0xff;
}

uint8_t DXLSyncReadRetrieveErrorByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index) {
	DXLSyncReadReturnItem_t *psrri = DXLSyncReadretrieveItemByIndex(SRData, index);
	return psrri ? psrri->error : 0xff;
}
uint16_t DXLSyncReadRetrieveLengthByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index) {
	DXLSyncReadReturnItem_t *psrri = DXLSyncReadretrieveItemByIndex(SRData, index);
	return psrri ? psrri->length : 0xff;
}

int DXLSyncReadRetrieveValueByIndex(DXLSyncReadDataHeader_t *SRData, uint8_t index, void *val, uint16_t val_offset, uint16_t val_length) {
  DXLSyncReadReturnItem_t *psrri = DXLSyncReadretrieveItemByIndex(SRData, index);
	if (!psrri) return 0;
	if (val_length == 0xffff) val_length = SRData->node_size;
	if ((val_offset + val_length) > SRData->node_size) return 0;
	memcpy(val, &psrri->data[val_offset], val_length);
	return 1;
}
