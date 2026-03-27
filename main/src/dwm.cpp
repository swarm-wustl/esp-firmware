#include "dwm.h"

#include "esp32.h"
#include "hardware.h"
#include "error.h"
#include "freertos/task.h"
#include <cstdlib>
#include <vector>


DWMDevice::DWMDevice(){
    randomShortAddress();
}

float DWMDevice::getRange(){ return float(_range)/100.0f; }
uint16_t DWMDevice:: getReplyTime() {return _replyDelayTimeUS;}
void DWMDevice::randomShortAddress() {
	_shortAddress[0] = rand() % 256;
	_shortAddress[1] = rand() % 256;
}


void DWMDevice::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }



uint64_t DWMTimestamp::getTimestamp() const {
        return raw_time_;
};

/**
 * Get timestamp as byte array
 * @param data var where data should be written
 */
void DWMTimestamp::getTimestamp(uint8_t data[]) const {
	memset(data, 0, LENGTH_TIMESTAMP);
	for(uint8_t i = 0; i < LENGTH_TIMESTAMP; i++) {
		data[i] = (uint8_t)((raw_time_ >> (i*8)) & 0xFF);
	}
}


void DWMTimestamp::setTimeStamp(uint64_t time){
    raw_time_ = time;
}

/**
 * Set timestamp
 * @param data timestamp as byte array
 */
void DWMTimestamp::setTimeStamp(uint8_t data[]) {
	raw_time_ = 0;
	for(uint8_t i = 0; i < LENGTH_TIMESTAMP; i++) {
		raw_time_ |= ((int64_t)data[i] << (i*8));
	}
}



template <HAL::GenericSPIController SPI>
DWM<SPI>::DWM(SPI spi, uint8_t rst_pin, uint8_t irq_pin) : 
    spi_{std::move(spi)},
    rst_pin_{rst_pin}, irq_pin_{irq_pin}
{
    hard_reset();

    // std::array<std::byte, DWM_LEN_DEV_ID> rx{};
    // read_reg(DWM_REG_DEV_ID, rx);
    // log("ID received: %X", std::bit_cast<uint32_t>(rx));

    auto id_reg = get_reg_view<DWM_REG_DEV_ID>();
    log("Reg size: %u", id_reg.size());
    log("Reg value: %X", id_reg.value());
    id_reg |= 0xFFFFFF;
    log("Reg value (should be same): %X", id_reg.value());
    
    auto sys_status_reg = get_reg_view<DWM_REG_SYSTEM_EVENT_STATUS>();
    log("Value before: %llX", sys_status_reg.value());
    sys_status_reg.clear_flags(0xFF);
    log("Value after: %llX", sys_status_reg.value());

    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();

    log("Current transmit bit rate: %X %X", ((tx_fctrl.bit(14) << 1) | tx_fctrl.bit(13)), tx_fctrl.bit_range(14, 13));
    logf("Bit rate, PRF, preamble length (but nice!):", tx_bit_rate(), tx_prf(), tx_preamble_length());

    set_tx_bit_rate(BitRate::KBPS_100);
    set_tx_prf(PRF::MHZ_4);
    set_tx_preamble_length(PreambleLength::LEN_2048);

    logf("New bit rate, PRF, preamble length:", tx_bit_rate(), "--", tx_prf(), "--", tx_preamble_length());
    hard_reset();
    logf("Reset bit rate, PRF, preamble length:", tx_bit_rate(), "--", tx_prf(), "--", tx_preamble_length());

    /* *** */

    auto sys_time_reg = get_reg_view<DWM_REG_SYS_TIME>();

    // Use the DW1000's own timestamp for precise intervals
    auto start = sys_time_reg.value();
    auto target_duration = std::chrono::milliseconds{300};

    while (true) {
        auto current = sys_time_reg.value();
        auto elapsed = current - start;
        
        if (elapsed >= target_duration) {
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            logf("DELTA SYS TIME:", us, "microseconds");
            start = current;  // Reset for next interval
        }
        
        vTaskDelay(1);  // Small delay to not busy-wait
    }
}

// TODO: this should use some sort of HAL::GenericGPIOController
// should not directly interact with ESP32 HAL!
template <HAL::GenericSPIController SPI>
void DWM<SPI>::hard_reset() {
    gpio_num_t rst = static_cast<gpio_num_t>(rst_pin_);

    gpio_set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  // hold low for >10ms
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // wait for startup
}

template <HAL::GenericSPIController SPI>
std::string_view DWM<SPI>::tx_bit_rate() const {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_bit_rate = tx_fctrl.bit_range(14, 13); // TODO: constants? 

    return BitRateToString(static_cast<BitRate>(raw_bit_rate)); 
}

// template <HAL::GenericSPIController SPI>
// std::string_view DWM<SPI>::getPrintableDeviceIdentifier(char msgBuffer[])
// {
// 	byte data[DWM_LEN_DEV_ID];
// 	readBytes(DEV_ID, NO_SUB, data, LEN_DEV_ID);
// 	sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
// 			(uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
// }

// void DW1000Class::getPrintableExtendedUniqueIdentifier(char msgBuffer[])
// {
// 	byte data[LEN_EUI];
// 	readBytes(EUI, NO_SUB, data, LEN_EUI);
// 	sprintf(msgBuffer, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
// 			data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
// }

template <HAL::GenericSPIController SPI>
void DWM<SPI>::getTransmitTimestamp(DWMTimestamp &time)
{
    auto tx_time = get_reg_view<DWM_REG_TX_TIME>();
    time.setTimeStamp = tx_time.value().getTimeStamp();
}
template <HAL::GenericSPIController SPI>
void DWM<SPI>::getReceiveTimestamp(DWMTimestamp &time)
{
	auto rx_time = get_reg_view<DWM_REG_TX_TIME>();
    time.setTimeStamp = rx_time.value().getTimeStamp();
}



/* device state management. */

template <HAL::GenericSPIController SPI>
void DWM<SPI>::idle(){
    auto sys_ctrl = get_reg_view(DWM_SYSTEM_CTRL);
    sys_ctrl.clear();
    uint8_t trxoff_bit = 6;
    sys_ctrl.write_bit_range(6,6,1);
    _deviceMode = IDLE_MODE;
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::newReceive(){
    idle();
    auto sys_ctrl = get_reg_view(DWM_SYSTEM_CTRL);
    sys_ctrl.clear();
    clearReceiveStatus();
    _deviceMode = TX_MODE;
}  

template <HAL::GenericSPIController SPI>
void DWM<SPI>::startReceive(){
    auto sys_ctrl = get_reg_view(DWM_SYSTEM_CTRL);
    sys_ctrl.write_bit_range(8,8,1);
}  

template <HAL::GenericSPIController SPI>
void DWM<SPI>::setReceiverAutoReenable(bool val){
    auto sys_conf = get_reg_view(DWM_REG_SYS_CONF);
    sys_conf.write_bit_range(29,29,val);
    clearReceiveStatus();

}  
template <HAL::GenericSPIController SPI>
void DWM<SPI>::receivePermanently(){
    _permanentReceive = true;
    setReceiverAutoReenable(true);
    
}  


template <HAL::GenericSPIController SPI>
void DWM<SPI>::startReceive(){
    idle();
    auto sys_ctrl = get_reg_view(DWM_SYSTEM_CTRL);
    sys_ctrl.clear();
    clearReceiveStatus();
    setBit(_sysctrl, LEN_SYS_CTRL, SFCST_BIT, !_frameCheck);
	setBit(_sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
	writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}  

template <HAL::GenericSPIController SPI>
auto DWM<SPI>::getRXData(){
    auto rx_reg = get_reg_view(DWM_REG_RX_BUFFER);
    auto data = rx_reg.value();
    return data;
}  


template <HAL::GenericSPIController SPI>
void DWM<SPI>::clearReceiveStatus(){
    auto sys_status_reg = get_reg_view<DWM_REG_SYSTEM_EVENT_STATUS>();
    log("Value before: %llX", sys_status_reg.value());
    sys_status_reg.clear_flags(0x5F400);
    log("Value after: %llX", sys_status_reg.value());

}  
template <HAL::GenericSPIController SPI>
void DWM<SPI>::clearTransmitStatus(){
     auto sys_status_reg = get_reg_view<DWM_REG_SYSTEM_EVENT_STATUS>();
    log("Value before: %llX", sys_status_reg.value());
    sys_status_reg.clear_flags(0xF0);
    log("Value after: %llX", sys_status_reg.value());
}
//Transmit state control:
template <HAL::GenericSPIController SPI>
void DWM<SPI>::newTransmit()
{
	idle();
    auto sys_ctrl = get_reg_view(DWM_SYSTEM_CTRL);
    sys_ctrl.clear();
    clearReceiveStatus();
	clearTransmitStatus();
    _deviceMode = TX_MODE;
}
template <HAL::GenericSPIController SPI>
void DWM<SPI>::startTransmit()
{
    //Ignoring write Preamble and pulse, frame info for now. 
	// writeTransmitFrameControlRegister();
    auto sys_ctrl = get_reg_view(DWM_SYSTEM_CTRL);
    sys_ctrl.write_bit_range(1,1,1);
	if (_permanentReceive)
	{
        sys_ctrl.clear();
        _deviceMode = RX_MODE;
		startReceive();
	}else{
        _deviceMode = IDLE_MODE;
    }
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::setDelay(const DWMTimestamp &delay)
{
    auto sys_ctrl = get_reg_view<DWM_REG_SYSTEM_CTRL>;
	if (_deviceMode == TX_MODE)
	{
        sys_ctrl.write_bit_range(2,2,1);
		setBit(_sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
	}
	else if (_deviceMode == RX_MODE)
	{
        sys_ctrl.write_bit_range(2,2,1);
		setBit(_sysctrl, LEN_SYS_CTRL, RXDLYS_BIT, true);
	}
	else
	{
		// in idle, ignore
		return DW1000Time();
	}
	byte delayBytes[5];
	DW1000Time futureTime;
	getSystemTimestamp(futureTime);
	futureTime += delay;
	futureTime.getTimestamp(delayBytes);
	delayBytes[0] = 0;
	delayBytes[1] &= 0xFE;
	writeBytes(DX_TIME, NO_SUB, delayBytes, LEN_DX_TIME);
	// adjust expected time with configured antenna delay
	futureTime.setTimestamp(delayBytes);
	futureTime += _antennaDelay;
	return futureTime;
}




template <HAL::GenericSPIController SPI>
void DWM<SPI>::set_tx_bit_rate(BitRate br) {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    tx_fctrl.write_bit_range(14, 13, static_cast<uint64_t>(br));
}



template <HAL::GenericSPIController SPI>
std::string_view DWM<SPI>::tx_prf() const {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_prf = tx_fctrl.bit_range(17, 16); // TODO: constants? 
    
    return PRFToString(static_cast<PRF>(raw_prf));
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::set_tx_prf(PRF prf) {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    tx_fctrl.write_bit_range(17, 16, static_cast<uint64_t>(prf));
}

template <HAL::GenericSPIController SPI>
uint16_t DWM<SPI>::tx_preamble_length() const {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();

    uint8_t raw_psr = tx_fctrl.bit_range(19, 18); // TODO: constants? 
    uint8_t raw_pe = tx_fctrl.bit_range(21, 20);  // TODO: constants?
    uint8_t psr_pe_combined = (raw_psr << 2) | raw_pe;

    return PreambleLengthToUInt(static_cast<PreambleLength>(psr_pe_combined));
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::set_tx_preamble_length(PreambleLength pl) {
    uint8_t psr_pe_combined = static_cast<uint8_t>(pl);
    uint8_t raw_psr = (psr_pe_combined >> 2) & 0b11;
    uint8_t raw_pe = psr_pe_combined & 0b11;
    
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    tx_fctrl.write_bit_range(19, 18, raw_psr);
    tx_fctrl.write_bit_range(21, 20, raw_pe);
}

// Allows for templated definition in .cpp file
template class DWM<HW::SPI>;