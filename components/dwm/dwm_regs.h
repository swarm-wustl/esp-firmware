// Written with Claude
#ifndef DWM_REGS_H
#define DWM_REGS_H

#include <array>
#include <cstdint>

// DW1000 register addresses, sub-register offsets, control/status bits, and the
// default channel configuration. Values from the DW1000 User Manual and the
// thotro/arduino-dw1000 reference (see docs/references.md). The DEFAULT_CONFIG
// table targets channel 5, 16 MHz PRF, preamble length 128, preamble code 4,
// 6.8 Mbps -- one known-good mode; other modes need different tuning values.
namespace dw1000 {

// register file ids
inline constexpr uint8_t TX_BUFFER = 0x09;
inline constexpr uint8_t SYS_CFG = 0x04;
inline constexpr uint8_t DX_TIME = 0x0A;
inline constexpr uint8_t SYS_CTRL = 0x0D;
inline constexpr uint8_t SYS_STATUS = 0x0F;
inline constexpr uint8_t RX_FINFO = 0x10;
inline constexpr uint8_t RX_BUFFER = 0x11;
inline constexpr uint8_t TX_ANTD = 0x18;
inline constexpr uint8_t CHAN_CTRL = 0x1F;
inline constexpr uint8_t OTP_IF = 0x2D;
inline constexpr uint8_t PMSC = 0x36;

// SYS_CTRL (0x0D) bits
inline constexpr uint32_t TXSTRT = 1u << 1;
inline constexpr uint32_t TRXOFF = 1u << 6;
inline constexpr uint32_t RXENAB = 1u << 8;

// SYS_STATUS (0x0F) bits
inline constexpr uint32_t TXFRS = 1u << 7;    // transmit frame sent
inline constexpr uint32_t LDEDONE = 1u << 10; // leading-edge processing done
inline constexpr uint32_t RXPHE = 1u << 12;   // rx phy header error
inline constexpr uint32_t RXDFR = 1u << 13;   // rx data frame ready
inline constexpr uint32_t RXFCG = 1u << 14;   // rx fcs good
inline constexpr uint32_t RXFCE = 1u << 15;   // rx fcs error
inline constexpr uint32_t RXRFSL = 1u << 16;  // rx reed-solomon frame sync loss
inline constexpr uint32_t RXRFTO = 1u << 17;  // rx frame wait timeout
inline constexpr uint32_t RXPTO = 1u << 21;   // preamble detection timeout
inline constexpr uint32_t RXSFDTO = 1u << 26; // sfd timeout

inline constexpr uint32_t RX_ERROR =
    RXPHE | RXFCE | RXRFSL | RXRFTO | RXPTO | RXSFDTO;

// default antenna delay (per manual; needs per-unit calibration for accuracy)
inline constexpr uint16_t ANTENNA_DELAY = 16436;

// one sub-register write: value packed little-endian into `size` bytes
struct ConfigWrite {
  uint8_t reg;
  uint16_t offset;
  uint32_t value;
  uint8_t size;
};

// channel 5, PRF 16 MHz, preamble 128 / code 4, 6.8 Mbps
inline constexpr std::array<ConfigWrite, 16> DEFAULT_CONFIG{{
    {0x23, 0x04, 0x8870, 2},     // AGC_TUNE1 (16 MHz)
    {0x23, 0x0C, 0x2502A907, 4}, // AGC_TUNE2
    {0x23, 0x12, 0x0035, 2},     // AGC_TUNE3
    {0x27, 0x02, 0x0001, 2},     // DRX_TUNE0b (6.8 Mbps)
    {0x27, 0x04, 0x0087, 2},     // DRX_TUNE1a (16 MHz)
    {0x27, 0x06, 0x0020, 2},     // DRX_TUNE1b (preamble > 64)
    {0x27, 0x08, 0x331A0052, 4}, // DRX_TUNE2 (16 MHz, PAC 8)
    {0x27, 0x26, 0x0028, 2},     // DRX_TUNE4H (preamble != 64)
    {0x28, 0x0B, 0xD8, 1},       // RF_RXCTRLH (channel 5)
    {0x28, 0x0C, 0x001E3FE0, 4}, // RF_TXCTRL (channel 5)
    {0x2A, 0x0B, 0xC0, 1},       // TC_PGDELAY (channel 5)
    {0x2B, 0x07, 0x0800041D, 4}, // FS_PLLCFG (channel 5)
    {0x2B, 0x0B, 0xBE, 1},       // FS_PLLTUNE (channel 5)
    {0x2E, 0x0806, 0x0D, 1},     // LDE_CFG1
    {0x2E, 0x1806, 0x1607, 2},   // LDE_CFG2 (16 MHz)
    {0x2E, 0x2804, 0x428E, 2},   // LDE_REPC (preamble code 4)
}};

// CHAN_CTRL for channel 5 TX/RX, RXPRF 16 MHz, TX/RX preamble code 4, std SFD:
// chan(5) | chan(5)<<4 | RXPRF(1)<<18 | TXCODE(4)<<22 | RXCODE(4)<<27
inline constexpr uint32_t CHAN_CTRL_VALUE = 0x21040055;

} // namespace dw1000

#endif
