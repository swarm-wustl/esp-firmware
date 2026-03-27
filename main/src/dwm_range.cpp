#include "dwm.h"
#include "dwm_range.h"
#include "error.h"
#include "hardware.h"


// message sent/received state

volatile bool anchorRec = false;
volatile bool anchorSent = true;

#define LEN_DATA 16
byte anchor_data[LEN_DATA];
HW::SPI spi{GPIO_NUM_4}; 
DWMTimestamp time;

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL;
// protocol error state
bool protocolFailed = false;
// timestamps to remember
DWMTimestamp timePollSent;
DWMTimestamp timePollReceived;
DWMTimestamp timePollAckSent;
DWMTimestamp timePollAckReceived;
DWMTimestamp timeRangeSent;
DWMTimestamp timeRangeReceived;
// last computed range/time
DWMTimestamp timeComputedRange;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;



void DWMRanging::startAsAnchor(DWM<HW::SPI> *device)
{
    // save the address
    //  DEBUG monitoring
    log("### DWM-ranging-anchor ###");
    // initialize the driver
    // general configuration, ignoring for now
    // DEBUG chip info and registers pretty printed
    char msg[128];

    _type = ANCHOR;
    receiver();
    noteActivity();

}


void DWMRanging::startAsTag(char address[], const byte mode[], const bool randomShortAddress)
{
    _type = TAG;
}

// Temporary 2 machine test functions

void DWMRanging::loop()
{
}

/// Private functions for handling different message types

void DWMRanging::handleSent()
{
    // status change on sent success
    _sentAck = true;
}

void DWMRanging::handleReceived()
{
    // status change on received success
    _receivedAck = true;
}

void DWMRanging::noteActivity()
{
    // update activity timestamp, so that we do not reach "resetPeriod"
    _lastActivity = millis();
}

void DWMRanging::resetInactive()
{
    // if inactive
    if (_type == ANCHOR)
    {
        _expectedMsgId = POLL;
        receiver();
    }
    noteActivity();
}

void DWMRanging::receiver()
{
    DWM<HW::SPI>::newReceive();
    // so we don't need to restart the receiver manually
    DWM<HW::SPI>::receivePermanently();
    DWM<HW::SPI>::startReceive();
}

void transmitPollAck() {
    DWM<HW::SPI>::newTransmit();
    // DW1000.setDefaults();
    data[0] = POLL_ACK;
    // delay the same amount as ranging tag
    DWMTimestamp deltaTime = DWMTimestamp(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DWMTimestamp.setDelay(deltaTime);
    
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeFailed() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_FAILED;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}




void DWMRanging::anchorLoop()
{
    if (!anchorRec && !anchorSent)
    {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod)
        {
            DWMRanging::resetInactive();
        }
        return;
    }

    if (anchorSent)
    {
        anchorSent= false;
        byte msgId = anchor_data[0];
        if (msgId == POLL_ACK)
        {   

            DWM<HW::SPI>::getTransmitTimestamp(timePollAckSent);
            noteActivity();
        }
    }
    if (anchorRec)
    {
        anchorRec = false;
        // get message and parse
        DWM<HW::SPI>::getRXData();
        byte msgId = anchor_data[0];

        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }
        if (msgId == POLL) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            DWM<HW::SPI>::getReceiveTimestamp(timePollReceived);
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            DWM<HW::SPI>::getReceiveTimestamp(timeRangeReceived);
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent.setTimeStamp(data + 1);
                timePollAckReceived.setTimeStamp(data + 6);
                timeRangeSent.setTimeStamp(data + 11);
                // (re-)compute range as two-way ranging is done
                computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
                transmitRangeReport(timeComputedRange.getAsMicroSeconds());
                float distance = timeComputedRange.getAsMeters();
                Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
                Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
                Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
                //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
                //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
                //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
                // update sampling rate (each second)
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            }
            else {
                transmitRangeFailed();
            }

            noteActivity();
        }

    }

}