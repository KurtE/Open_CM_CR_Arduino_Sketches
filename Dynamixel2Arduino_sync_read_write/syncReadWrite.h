#include <Dynamixel2Arduino.h>
#ifndef DXL_MAX_NODE_BUFFER_SIZE
#define DXL_MAX_NODE_BUFFER_SIZE 8
#endif

#ifndef DXL_MAX_NODE
#define DXL_MAX_NODE 254
#endif


class SyncWrite {
public:
	SyncWrite(uint16_t starting_addr=0, uint16_t node_size=DXL_MAX_NODE_BUFFER_SIZE, uint8_t max_servos=DXL_MAX_NODE, 
			uint8_t *buffer=nullptr, uint16_t buffer_size=0)
		:  _starting_addr(starting_addr), _node_size(node_size), _max_servos(max_servos), _buffer(buffer), _buffer_size(buffer_size) {};

	~SyncWrite();
	static constexpr uint16_t suggestedBufferSize(uint8_t max_servos, uint16_t node_size)
		 {return (uint16_t)max_servos * (node_size+1) + 4;}

	inline uint16_t startingAddr() {return _starting_addr;}	 
	inline uint16_t nodeSize() {return _node_size;}
	inline uint8_t maxServos() {return _max_servos;}
	inline uint8_t countServos() {return _cnt_servos;}

	inline void startingAddr(uint16_t val) {_starting_addr = val;}
	inline void nodeSize(uint16_t val) {_node_size = val;}
	inline void maxServos(uint8_t val) {_max_servos = val;}
	inline void countServos(uint8_t val) {_cnt_servos = val;}

	void setBuffer(uint8_t *buffer, uint16_t buffer_size);
	bool init();

	// Setup ids and values
	int addID(uint8_t id);		// Adds a servo id;  returns index
	int setItem(uint8_t id, void *val, uint16_t val_offset=0, uint16_t val_length=0xffff);
	int setItemByIndex(int index, void *val, uint16_t val_offset=0, uint16_t val_length=0xffff);

	bool send(Dynamixel2Arduino &pdxl);

private:
	// Ones that can be defined by constructor
	uint16_t		_starting_addr;
	uint16_t		_node_size;
	uint8_t			_max_servos;
	uint8_t			*_buffer;
	uint16_t		_buffer_size;

	uint8_t 		_cnt_servos = 0;
	uint8_t			_cnt_receive = 0;
	uint8_t			_init = 0;
	uint8_t			_buffer_malloced = 0;
};

typedef struct {
	uint8_t 	id;
	uint8_t		error;
	uint16_t	length;
	uint8_t		data[0];
} SyncReadReturnItem_t;

class SyncRead {
public:
	SyncRead(uint16_t starting_addr=0, uint16_t node_size=DXL_MAX_NODE_BUFFER_SIZE, uint8_t max_servos=DXL_MAX_NODE, 
			uint8_t *buffer=nullptr, uint16_t buffer_size=0)
		:  _starting_addr(starting_addr), _node_size(node_size), _max_servos(max_servos), _buffer(buffer), _buffer_size(buffer_size) {};

	~SyncRead();
	static constexpr uint16_t suggestedBufferSize(uint8_t max_servos, uint16_t node_size)
		 {return (uint16_t)max_servos * (node_size+5) + 4;}

	inline uint16_t startingAddr() {return _starting_addr;}	 
	inline uint16_t nodeSize() {return _node_size;}
	inline uint8_t maxServos() {return _max_servos;}
	inline uint8_t countServos() {return _cnt_servos;}
	inline uint8_t receiveCount() {return _cnt_received;}

	inline void startingAddr(uint16_t val) {_starting_addr = val;}
	inline void nodeSize(uint16_t val) {_node_size = val;}
	inline void maxServos(uint8_t val) {_max_servos = val;}
	inline void countServos(uint8_t val) {_cnt_servos = val;}

	void setBuffer(uint8_t *buffer, uint16_t buffer_size);
	bool init();

	// Setup ids and 
	int idToIndex(uint8_t id);	// converts an id to index does not add if not in list..
	int addID(uint8_t id);		// Adds a servo id;  returns index

	int idToRxIndex(uint8_t id);
	SyncReadReturnItem_t *retrieveItem(uint8_t id);
	SyncReadReturnItem_t *retrieveItemByIndex(uint8_t index);
	uint8_t retrieveIDByIndex(uint8_t index);
	uint8_t retrieveErrorByIndex(uint8_t index);
	uint16_t retrieveLengthByIndex(uint8_t index);
	bool retrieveValueByIndex(uint8_t index, void *val, uint16_t val_offset=0, uint16_t val_length=0xffff);

	bool doRead(Dynamixel2Arduino &pdxl,  uint32_t timeout=100);

private:
	// Ones that can be defined by constructor
	uint16_t		_starting_addr;
	uint16_t		_node_size;
	uint8_t			_max_servos;
	uint8_t			*_buffer;
	uint16_t		_buffer_size;

	uint8_t 		_cnt_servos = 0;
	uint8_t			_init = 0;
	uint8_t			_buffer_malloced = 0;
	uint8_t 		_cnt_received = 0;
};
