#define INACTIVITY_TIME 1000

#ifndef _DW1000Device_H_INCLUDED
#define _DW1000Device_H_INCLUDED

#include "dwm_time.h"
// #include "DW1000Mac.h"

class DW1000Mac;

class DW1000Device;

class DW1000Device {
public:
	//Constructor and destructor
	DW1000Device();
	DW1000Device(uint8_t address[], uint8_t shortAddress[]);
	DW1000Device(uint8_t address[], bool shortOne = false);
	~DW1000Device();
	
	//setters:
	void setReplyTime(uint16_t replyDelayTimeUs);
	void setAddress(char address[]);
	void setAddress(uint8_t* address);
	void setShortAddress(uint8_t address[]);
	
	void setRange(float range);
	void setRXPower(float power);
	void setFPPower(float power);
	void setQuality(float quality);
	
	void setReplyDelayTime(uint16_t time) { _replyDelayTimeUS = time; }
	
	void setIndex(int8_t index) { _index = index; }
	
	//getters
	uint16_t getReplyTime() { return _replyDelayTimeUS; }
	
	uint8_t* getuint8_tAddress();
	
	int8_t getIndex() { return _index; }
	
	//String getAddress();
	uint8_t* getuint8_tShortAddress();
	uint16_t getShortAddress();
	//String getShortAddress();
	
	float getRange();
	float getRXPower();
	float getFPPower();
	float getQuality();
	
	bool isAddressEqual(DW1000Device* device);
	bool isShortAddressEqual(DW1000Device* device);
	
	//functions which contains the date: (easier to put as public)
	// timestamps to remember
	DW1000Time timePollSent;
	DW1000Time timePollReceived;
	DW1000Time timePollAckSent;
	DW1000Time timePollAckReceived;
	DW1000Time timeRangeSent;
	DW1000Time timeRangeReceived;
	
	void    noteActivity();
	bool isInactive();


private:
	//device ID
	uint8_t         _ownAddress[8];
	uint8_t         _shortAddress[2];
	int32_t      _activity;
	uint16_t     _replyDelayTimeUS;
	int8_t       _index; // not used
	
	int16_t _range;
	int16_t _RXPower;
	int16_t _FPPower;
	int16_t _quality;
	
	void randomShortAddress();
	
};


#endif
