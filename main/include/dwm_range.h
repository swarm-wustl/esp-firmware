#include "dwm.h"
#include "hal.h"
#include "hardware.h"


// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define BLINK 4
#define RANGING_INIT 5

#define LEN_DATA 90

//Max devices we put in the networkDevices array ! Each DW1000Device is 74 Bytes in SRAM memory for now.
#define MAX_DEVICES 4

//Default Pin for module:
#define DEFAULT_RST_PIN 9
#define DEFAULT_SPI_SS_PIN 10

//Default value
//in ms
#define DEFAULT_RESET_PERIOD 200
//in us
#define DEFAULT_REPLY_DELAY_TIME 7000

//sketch type (anchor or tag)
#define TAG 0
#define ANCHOR 1

//default timer delay
#define DEFAULT_TIMER_DELAY 80

//debug mode
#ifndef DEBUG
#define DEBUG false
#endif

#include <stdint.h>
typedef uint8_t byte;


class DWMRanging{
    public:
        static byte data[LEN_DATA]; 
		

        //Initialization
        static void    startAsAnchor(char address[], const byte mode[], const bool randomShortAddress = true);
	    static void    startAsTag(char address[], const byte mode[], const bool randomShortAddress = true);
        static void startAsAnchor(DWM<HW::SPI>* device);
        static void startAsTag(DWM<HW::SPI>* device);
        static void startAsAnchor();
        static void startAsTag();
		static void anchorTest();
		static void tagTest();
		static void anchorLoop();
		static void tagLoop();
        
        
        //Setters
        

        //Getters
        static void setReplyTime(uint16_t replyDelayTimeUs);
	    static void setResetPeriod(uint32_t resetPeriod);

        //Ranging
        //ranging functions
        static int16_t detectMessageType(byte datas[]); // TODO check return type
        static void loop();
    private:
        //sketch type (tag or anchor)
	static int16_t          _type; //0 for tag and 1 for anchor
	// TODO check type, maybe enum?
	// message flow state
	static volatile byte    _expectedMsgId;
	// message sent/received state
	static volatile bool _sentAck;
	static volatile bool _receivedAck;
	// protocol error state
	static bool          _protocolFailed;
	// reset line to the chip
	static uint8_t     _RST;
	static uint8_t     _SS;
	// watchdog and reset period
	static uint32_t    _lastActivity;
	static uint32_t    _resetPeriod;
	// reply times (same on both sides for symm. ranging)
	static uint16_t     _replyDelayTimeUS;
	//timer Tick delay
	static uint16_t     _timerDelay;
	// ranging counter (per second)
	static uint16_t     _successRangingCount;
	static uint32_t    _rangingCountPeriod;
	//ranging filter
	static volatile bool _useRangeFilter;
	static uint16_t         _rangeFilterValue;


    //methods
    	//methods
	static void handleSent();
	static void handleReceived();
	static void noteActivity();
	static void resetInactive();
	



	//for ranging protocole (ANCHOR)
	static void transmitInit();
	static void transmit(byte datas[]);
	static void transmit(byte datas[], DWMTimestamp time);
	static void transmitBlink();
	static void transmitRangingInit(DWMDevice* myDistantDevice);
	static void transmitPollAck(DWMDevice* myDistantDevice);
	static void transmitRangeReport(DWMDevice* myDistantDevice);
	static void transmitRangeFailed(DWMDevice* myDistantDevice);
	static void receiver();
	
	//for ranging protocole (TAG)
	static void transmitPoll(DWMDevice* myDistantDevice);
	static void transmitRange(DWMDevice* myDistantDevice);
	
	//methods for range computation
	static void computeRangeAsymmetric(DWMDevice* myDistantDevice, DWMTimestamp* myTOF);
	DWM<HW::SPI>& dwm_;
};