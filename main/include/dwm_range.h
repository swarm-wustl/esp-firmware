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


class DWM_RANGING{
    public:
        static byte data[LEN_DATA]; 


        //Initialization
        static void startAsAnchor();
        static void startAsTag();
        
        
        //Setters
        

        //Getters
        static void setReplyTime(uint16_t replyDelayTimeUs);
	    static void setResetPeriod(uint32_t resetPeriod);

        //Ranging
        //ranging functions
        static int16_t detectMessageType(byte datas[]); // TODO check return type
        static void loop();
    private:
};