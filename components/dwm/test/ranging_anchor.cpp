#include "dwm.h"
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
ESP32::SPI spi{GPIO_NUM_4};
DWMRegisterView<ESP32::SPI, DWMRegisterID::TX_BUFFER> tx_buf_reg{spi};
std::array<std::byte, 1024> data;
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
uitn64_t replyDelayTimeRaw = 191692800;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
int64_t start = esp_timer_get_time();

int64_t millis(){

    return (esp_timer_get_time() - start) / 1000;
}

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


void noteActivity()
{
    // update activity timestamp, so that we do not reach "resetPeriod"
    //TODO: MIllis from arduino not usable, modify this. 
    lastActivity = millis();
}

void resetInactive()
{
    // if inactive
    if (_type == ANCHOR)
    {
        _expectedMsgId = POLL;
        receiver();
    }
    noteActivity();
}

void receiver()
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
    DWMTimestamp deltaTime = DWMTimestamp(replyDelayTimeRaw);
   // DWMTimestamp deltaTime = DWMTimestamp(replyDelayTimeUS, DWMTimestamp::MICROSECONDS);
    DWMTimestamp.setDelay(deltaTime);
   
    DWM<HW::SPI>::setData(data);
    DWM<HW::SPI>::startTransmit();
}

void transmitRangeReport(float curRange) {
    DWM<HW::SPI>::newTransmit();
    // DW1000.newTransmit();
    // DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    // write final ranging result
    std::memcpy(&data[1], &curRange, sizeof(curRange));
    DWM<HW::SPI>::setData(data);
    DWM<HW::SPI>::startTransmit();
}

void transmitRangeFailed() {
    DWM<HW::SPI>::newTransmit();
    data[0] = RANGE_FAILED;
    DWM<HW::SPI>::setData(data);
    DWM<HW::SPI>::startTransmit();
}


void computeRangeAsymmetric() {
    // asymmetric two-way ranging (more computation intense, less error prone)
    Duration round1 = (timePollAckReceived - timePollSent).wrap();
    Duration reply1 = (timePollAckSent - timePollReceived).wrap();
    Duration round2 = (timeRangeReceived - timePollAckSent).wrap();
    Duration reply2 = (timeRangeSent - timePollAckReceived).wrap();
    Duration tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    timeComputedRange.update(tof);
}


void anchorLoop()
{
     int64_t curMillis = millis();
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
        byte msgId = data[0];
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
        DWM<HW::SPI>::getRXData(data);
        tx_buf_reg.read_data();
        data = tx_buf_reg.value();
        byte msgId = data[0];

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
                timePollSent.update(data[1]);
                timePollAckReceived.update(data[6]);
                timeRangeSent.update(data[11]);
                // (re-)compute range as two-way ranging is done
                computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
                auto range_as_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(timeComputedRange.getDuration());
                transmitRangeReport(range_as_microseconds);
                double range_as_seconds = std::chrono::duration_cast<std::chrono::seconds>(timeComputedRange.getDuration())
                double range_as_meters = range_as_seconds * 3e8;
                float distance = range_as_meters
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