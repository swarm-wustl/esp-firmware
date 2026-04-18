#include "dwm.h"

#include "esp32.h"
#include "hardware.h"
#include "error.h"
#include "freertos/task.h"
#include <cstdlib>
#include <vector>


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
auto DWM<SPI>::setData(std::array data){
    //TODO add error handling.
    ESP32::SPI spi{GPIO_NUM_4};
    DWMRegisterView<ESP32::SPI, DWMRegisterID::TX_BUFFER> tx_buf_reg{spi};
    tx_buf_reg.write_data(std::span{data[0]});
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

