#ifndef DWM_REGS_H
#define DWM_REGS_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <utility>

#include "bits.h"
#include "registers.h"

// TODO: add this, and other related classes, to some sort of DWM namespace
enum class PRF : uint8_t { MHZ_4 = 0b00, MHZ_16 = 0b01, MHZ_64 = 0b10 };

constexpr std::string_view to_string_view(PRF prf) noexcept {
  using namespace std::string_view_literals;

  switch (prf) {
  case PRF::MHZ_4:
    return "4 MHz"sv;
  case PRF::MHZ_16:
    return "16 MHz"sv;
  case PRF::MHZ_64:
    return "64 MHz"sv;
  }

  // the value comes off the wire, so a reserved encoding is reachable
  return "UNKNOWN PRF"sv;
}

enum class BitRate : uint8_t { KBPS_100 = 0b00, KBPS_850 = 0b01, MBPS_68 = 0b10 };

constexpr std::string_view to_string_view(BitRate br) noexcept {
  using namespace std::string_view_literals; // Allows for ""sv suffix

  switch (br) {
  case BitRate::KBPS_100:
    return "110 kbps"sv;
  case BitRate::KBPS_850:
    return "850 kbps"sv;
  case BitRate::MBPS_68:
    return "6.8 Mbps"sv;
  }

  // the value comes off the wire, so a reserved encoding is reachable
  return "UNKNOWN BITRATE"sv;
}

// DW1000 register addresses, sub-register offsets, control/status bits, and the
// default channel configuration. Values from the DW1000 User Manual and the
// thotro/arduino-dw1000 reference (see docs/references.md). The DEFAULT_CONFIG
// table targets channel 5, 16 MHz PRF, preamble length 128, preamble code 4,
// 6.8 Mbps -- one known-good mode; other modes need different tuning values.
namespace dw1000 {
using Registers::Access;

// a register file and the octet within it. Structural, so it can be a
// register's template argument, and its own type, so Registers::at keeps
// another device's registers out
struct Address {
  uint8_t file;
  uint16_t sub = 0;
};

// sub-addressed SPI header (manual 2.2.1.2): 1 octet non-indexed, 2 for an
// offset <= 0x7F, 3 with the extended-address flag for larger offsets
template <Address A> constexpr size_t header_size() {
  return A.sub == 0 ? 1 : A.sub <= 0x7F ? 2 : 3;
}

template <Address A>
constexpr std::array<std::byte, header_size<A>()> header(bool write) {
  const auto first = static_cast<std::byte>((write ? 0x80 : 0x00) |
                                            (A.file & 0x3F) |
                                            (A.sub ? 0x40 : 0x00));

  if constexpr (header_size<A>() == 1) {
    return {first};
  } else if constexpr (header_size<A>() == 2) {
    return {first, static_cast<std::byte>(A.sub & 0x7F)};
  } else {
    return {first, static_cast<std::byte>(0x80 | (A.sub & 0x7F)),
            static_cast<std::byte>((A.sub >> 7) & 0xFF)};
  }
}

// a tuning register the manual gives as one opaque number: the whole width is
// a single field, named for the register, and the size comes from it
template <Strings::fixed_string Name, size_t Bytes>
using whole = Registers::field<Name, Registers::word_for<Bytes>,
                               BITS_PER_BYTE * Bytes - 1, 0>;

template <Address A, typename F>
using tuning =
    Registers::reg<A, {.bytes = F::width / BITS_PER_BYTE}, F>;

enum class PanId : uint16_t {};
enum class ShortAddress : uint16_t {};

// 0xFFFF in either is what every node accepts (manual 5.2.1)
inline constexpr PanId ANY_PAN{0xFFFF};
inline constexpr ShortAddress BROADCAST{0xFFFF};

struct Identity {
  PanId pan;
  ShortAddress address;
};

using REV = Registers::field<"REV", uint8_t, 3, 0>;
using VER = Registers::field<"VER", uint8_t, 7, 4>;
using MODEL = Registers::field<"MODEL", uint8_t, 15, 8>;
using RIDTAG = Registers::field<"RIDTAG", uint16_t, 31, 16>;

using SHORT_ADDR = Registers::field<"SHORT_ADDR", ShortAddress, 15, 0>;
using PAN_ID = Registers::field<"PAN_ID", PanId, 31, 16>;

enum class PhrMode : uint8_t { Standard = 0b00, Long = 0b11 };

using FFEN = Registers::field<"FFEN", bool, 0, 0>;
using FFBC = Registers::field<"FFBC", bool, 1, 1>;
using FFAB = Registers::field<"FFAB", bool, 2, 2>;
using FFAD = Registers::field<"FFAD", bool, 3, 3>;
using FFAA = Registers::field<"FFAA", bool, 4, 4>;
using FFAM = Registers::field<"FFAM", bool, 5, 5>;
using FFAR = Registers::field<"FFAR", bool, 6, 6>;
using FFA4 = Registers::field<"FFA4", bool, 7, 7>;
using FFA5 = Registers::field<"FFA5", bool, 8, 8>;
using HIRQ_POL = Registers::field<"HIRQ_POL", bool, 9, 9>;
using SPI_EDGE = Registers::field<"SPI_EDGE", bool, 10, 10>;
using DIS_FCE = Registers::field<"DIS_FCE", bool, 11, 11>;
using DIS_DRXB = Registers::field<"DIS_DRXB", bool, 12, 12>;
using DIS_PHE = Registers::field<"DIS_PHE", bool, 13, 13>;
using DIS_RSDE = Registers::field<"DIS_RSDE", bool, 14, 14>;
using FCS_INIT2F = Registers::field<"FCS_INIT2F", bool, 15, 15>;
using PHR_MODE = Registers::field<"PHR_MODE", PhrMode, 17, 16>;
using DIS_STXP = Registers::field<"DIS_STXP", bool, 18, 18>;
using RXM110K = Registers::field<"RXM110K", bool, 22, 22>;
using RXWTOE = Registers::field<"RXWTOE", bool, 28, 28>;
// without it a rejected frame leaves the receiver idle, so with FFEN on one
// stray frame ends the listen (manual 7.2.6)
using RXAUTR = Registers::field<"RXAUTR", bool, 29, 29>;
using AUTOACK = Registers::field<"AUTOACK", bool, 30, 30>;
using AACKPEND = Registers::field<"AACKPEND", bool, 31, 31>;

using TFLEN = Registers::field<"TFLEN", uint8_t, 6, 0>;
using TFLE = Registers::field<"TFLE", uint8_t, 9, 7>;
using TXBR = Registers::field<"TXBR", BitRate, 14, 13>;
using TR = Registers::field<"TR", bool, 15, 15>;
using TXPRF = Registers::field<"TXPRF", PRF, 17, 16>;
using TXPSR = Registers::field<"TXPSR", uint8_t, 19, 18>;
using PE = Registers::field<"PE", uint8_t, 21, 20>;
using TXBOFFS = Registers::field<"TXBOFFS", uint16_t, 31, 22>;
using IFSDELAY = Registers::field<"IFSDELAY", uint8_t, 39, 32>;

// every control bit starts an action on a 1 and the part clears it as the
// action begins, so the zeros a write leaves beside it are no-ops
using SFCST = Registers::field<"SFCST", bool, 0, 0>;
using TXSTRT = Registers::field<"TXSTRT", bool, 1, 1>;
using TXDLYS = Registers::field<"TXDLYS", bool, 2, 2>;
using CANSFCS = Registers::field<"CANSFCS", bool, 3, 3>;
using TRXOFF = Registers::field<"TRXOFF", bool, 6, 6>;
using WAIT4RESP = Registers::field<"WAIT4RESP", bool, 7, 7>;
using RXENAB = Registers::field<"RXENAB", bool, 8, 8>;
using RXDLYE = Registers::field<"RXDLYE", bool, 9, 9>;
using HRBPT = Registers::field<"HRBPT", bool, 24, 24>;

using IRQS = Registers::field<"IRQS", bool, 0, 0, Access::ReadOnly>;
using TXFRS = Registers::field<"TXFRS", bool, 7, 7>;           // transmit frame sent
using LDEDONE = Registers::field<"LDEDONE", bool, 10, 10>;     // leading-edge processing done
using RXPHE = Registers::field<"RXPHE", bool, 12, 12>;         // rx phy header error
using RXDFR = Registers::field<"RXDFR", bool, 13, 13>;         // rx data frame ready
using RXFCG = Registers::field<"RXFCG", bool, 14, 14>;         // rx fcs good
using RXFCE = Registers::field<"RXFCE", bool, 15, 15>;         // rx fcs error
using RXRFSL = Registers::field<"RXRFSL", bool, 16, 16>;       // rx reed-solomon frame sync loss
using RXRFTO = Registers::field<"RXRFTO", bool, 17, 17>;       // rx frame wait timeout
using RXOVRR = Registers::field<"RXOVRR", bool, 20, 20, Access::ReadOnly>;
using RXPTO = Registers::field<"RXPTO", bool, 21, 21>;         // preamble detection timeout
using RXSFDTO = Registers::field<"RXSFDTO", bool, 26, 26>;     // sfd timeout
using AFFREJ = Registers::field<"AFFREJ", bool, 29, 29>;       // frame filter rejection
using HPDWARN = Registers::field<"HPDWARN", bool, 27, 27, Access::ReadOnly>;
using HSRBP = Registers::field<"HSRBP", bool, 30, 30, Access::ReadOnly>;
using ICRBP = Registers::field<"ICRBP", bool, 31, 31, Access::ReadOnly>;
using TXPUTE = Registers::field<"TXPUTE", bool, 34, 34, Access::ReadOnly>;

using RxErrors = std::tuple<RXPHE, RXFCE, RXRFSL, RXRFTO, RXPTO, RXSFDTO>;

using RXFLEN = Registers::field<"RXFLEN", uint8_t, 6, 0>;
using RXFLE = Registers::field<"RXFLE", uint8_t, 9, 7>;

// full 40-bit counter. do NOT drop the low bits: the sub-nanosecond flight
// time that ranging measures lives in them (1 m ~= 213 of these ~15.65 ps
// ticks, and the low 9 bits span ~8 ns). masking them zeroes the measurement.
enum class Timestamp : uint64_t {};

using RX_STAMP = Registers::field<"RX_STAMP", Timestamp, 39, 0>;
using TX_STAMP = Registers::field<"TX_STAMP", Timestamp, 39, 0>;

static_assert(RX_STAMP::mask == TX_STAMP::mask);

using Ticks = std::chrono::duration<uint64_t, std::ratio<1, 63'897'600'000>>; // each bit = ~15.65 ps

// the counter wraps every ~17 s, so a difference is taken mod 2^40
constexpr Ticks operator-(Timestamp later, Timestamp earlier) {
  return Ticks{(std::to_underlying(later) - std::to_underlying(earlier)) &
               RX_STAMP::mask};
}

using TX_CHAN = Registers::field<"TX_CHAN", uint8_t, 3, 0>;
using RX_CHAN = Registers::field<"RX_CHAN", uint8_t, 7, 4>;
using RXPRF = Registers::field<"RXPRF", PRF, 19, 18>;
using TX_PCODE = Registers::field<"TX_PCODE", uint8_t, 26, 22>;
using RX_PCODE = Registers::field<"RX_PCODE", uint8_t, 31, 27>;

enum class SysClock : uint8_t { Auto = 0b00, Xti = 0b01, Pll = 0b10 };

using SYSCLKS = Registers::field<"SYSCLKS", SysClock, 1, 0>;
// reserved in the field table, but step L-1 of the LDE load (manual 2.5.5.10,
// Table 4) writes 0x0301 -- SYSCLKS on XTI with both of these set -- and L-3
// writes 0x0200, their reset value
using LDE_LOAD_CLKS = Registers::field<"LDE_LOAD_CLKS", uint8_t, 9, 8>;

using LDELOAD = Registers::field<"LDELOAD", bool, 15, 15>;

using TX_ANTD = whole<"TX_ANTD", 2>;
using LDE_RXANTD = whole<"LDE_RXANTD", 2>;

using AGC_TUNE1 = whole<"AGC_TUNE1", 2>;
using AGC_TUNE2 = whole<"AGC_TUNE2", 4>;
using AGC_TUNE3 = whole<"AGC_TUNE3", 2>;
using DRX_TUNE0b = whole<"DRX_TUNE0b", 2>;
using DRX_TUNE1a = whole<"DRX_TUNE1a", 2>;
using DRX_TUNE1b = whole<"DRX_TUNE1b", 2>;
using DRX_TUNE2 = whole<"DRX_TUNE2", 4>;
using DRX_TUNE4H = whole<"DRX_TUNE4H", 2>;
using RF_RXCTRLH = whole<"RF_RXCTRLH", 1>;
using RF_TXCTRL = whole<"RF_TXCTRL", 4>;
using TC_PGDELAY = whole<"TC_PGDELAY", 1>;
using FS_PLLCFG = whole<"FS_PLLCFG", 4>;
using FS_PLLTUNE = whole<"FS_PLLTUNE", 1>;
using LDE_CFG1 = whole<"LDE_CFG1", 1>;
using LDE_CFG2 = whole<"LDE_CFG2", 2>;
using LDE_REPC = whole<"LDE_REPC", 2>;

namespace Map {
using DEV_ID = Registers::reg<
    Address{0x00}, {.bytes = 4, .access = Access::ReadOnly, .reset = 0xDECA0130},
    REV, VER, MODEL, RIDTAG>;
// manual 2.02's bit table shows only TFLEN = 12, but the default mode it
// describes in 2.5 (16 MHz, 6.8 Mbps, preamble 128) is what the part resets to,
// and the device test reads 16 MHz back after a reset
using PANADR = Registers::reg<Address{0x03}, {.bytes = 4, .reset = 0xFFFFFFFF},
                              SHORT_ADDR, PAN_ID>;
using SYS_CFG = Registers::reg<
    Address{0x04}, {.bytes = 4, .reset = 0x00001200}, FFEN, FFBC, FFAB, FFAD,
    FFAA, FFAM, FFAR, FFA4, FFA5, HIRQ_POL, SPI_EDGE, DIS_FCE, DIS_DRXB,
    DIS_PHE, DIS_RSDE, FCS_INIT2F, PHR_MODE, DIS_STXP, RXM110K, RXWTOE,
    RXAUTR, AUTOACK, AACKPEND>;
using TX_FCTRL = Registers::reg<Address{0x08}, {.bytes = 5, .reset = 0x0015400C},
                                TFLEN, TFLE, TXBR, TR, TXPRF, TXPSR, PE,
                                TXBOFFS, IFSDELAY>;
using TX_BUFFER = Registers::reg<Address{0x09}, {.bytes = 1024}>;
using SYS_CTRL = Registers::reg<Address{0x0D}, {.bytes = 4}, SFCST, TXSTRT,
                                TXDLYS, CANSFCS, TRXOFF, WAIT4RESP, RXENAB,
                                RXDLYE, HRBPT>;
using SYS_STATUS = Registers::reg<
    Address{0x0F}, {.bytes = 5, .access = Access::WriteOneClear}, IRQS, TXFRS,
    LDEDONE, RXPHE, RXDFR, RXFCG, RXFCE, RXRFSL, RXRFTO, RXOVRR, RXPTO,
    RXSFDTO, AFFREJ, HPDWARN, HSRBP, ICRBP, TXPUTE>;
using RX_FINFO =
    Registers::reg<Address{0x10}, {.bytes = 4, .access = Access::ReadOnly},
                   RXFLEN, RXFLE>;
using RX_BUFFER =
    Registers::reg<Address{0x11}, {.bytes = 1024, .access = Access::ReadOnly}>;
// 5 = the RX_STAMP subfield at offset 0, not the full 14-octet register
using RX_TIME =
    Registers::reg<Address{0x15}, {.bytes = 5, .access = Access::ReadOnly},
                   RX_STAMP>;
// 5 = the TX_STAMP subfield at offset 0, not the full 10-octet register
using TX_TIME =
    Registers::reg<Address{0x17}, {.bytes = 5, .access = Access::ReadOnly},
                   TX_STAMP>;
using TX_ANTD = tuning<Address{0x18}, dw1000::TX_ANTD>;
using CHAN_CTRL = Registers::reg<Address{0x1F}, {.bytes = 4, .reset = 0x55},
                                 TX_CHAN, RX_CHAN, RXPRF, TX_PCODE, RX_PCODE>;
using AGC_TUNE1 = tuning<Address{0x23, 0x04}, dw1000::AGC_TUNE1>;
using AGC_TUNE2 = tuning<Address{0x23, 0x0C}, dw1000::AGC_TUNE2>;
using AGC_TUNE3 = tuning<Address{0x23, 0x12}, dw1000::AGC_TUNE3>;
using DRX_TUNE0b = tuning<Address{0x27, 0x02}, dw1000::DRX_TUNE0b>;
using DRX_TUNE1a = tuning<Address{0x27, 0x04}, dw1000::DRX_TUNE1a>;
using DRX_TUNE1b = tuning<Address{0x27, 0x06}, dw1000::DRX_TUNE1b>;
using DRX_TUNE2 = tuning<Address{0x27, 0x08}, dw1000::DRX_TUNE2>;
using DRX_TUNE4H = tuning<Address{0x27, 0x26}, dw1000::DRX_TUNE4H>;
using RF_RXCTRLH = tuning<Address{0x28, 0x0B}, dw1000::RF_RXCTRLH>;
using RF_TXCTRL = tuning<Address{0x28, 0x0C}, dw1000::RF_TXCTRL>;
using TC_PGDELAY = tuning<Address{0x2A, 0x0B}, dw1000::TC_PGDELAY>;
using FS_PLLCFG = tuning<Address{0x2B, 0x07}, dw1000::FS_PLLCFG>;
using FS_PLLTUNE = tuning<Address{0x2B, 0x0B}, dw1000::FS_PLLTUNE>;
using OTP_CTRL = Registers::reg<Address{0x2D, 0x06}, {.bytes = 2}, LDELOAD>;
using LDE_CFG1 = tuning<Address{0x2E, 0x0806}, dw1000::LDE_CFG1>;
using LDE_RXANTD = tuning<Address{0x2E, 0x1804}, dw1000::LDE_RXANTD>;
using LDE_CFG2 = tuning<Address{0x2E, 0x1806}, dw1000::LDE_CFG2>;
using LDE_REPC = tuning<Address{0x2E, 0x2804}, dw1000::LDE_REPC>;
// the low half of a 32-bit register: the LDE load writes no more of it
using PMSC_CTRL0 = Registers::reg<Address{0x36}, {.bytes = 2, .reset = 0x0200},
                                  SYSCLKS, LDE_LOAD_CLKS>;
} // namespace Map

// default antenna delay (per manual; needs per-unit calibration for accuracy)
inline constexpr uint16_t ANTENNA_DELAY = 16436;

// channel 5, PRF 16 MHz, preamble 128 / code 4, 6.8 Mbps
inline constexpr std::tuple DEFAULT_CONFIG{
    Registers::set<Map::AGC_TUNE1>(AGC_TUNE1{0x8870}),      // 16 MHz
    Registers::set<Map::AGC_TUNE2>(AGC_TUNE2{0x2502A907}),
    Registers::set<Map::AGC_TUNE3>(AGC_TUNE3{0x0035}),
    Registers::set<Map::DRX_TUNE0b>(DRX_TUNE0b{0x0001}),    // 6.8 Mbps
    Registers::set<Map::DRX_TUNE1a>(DRX_TUNE1a{0x0087}),    // 16 MHz
    Registers::set<Map::DRX_TUNE1b>(DRX_TUNE1b{0x0020}),    // preamble > 64
    Registers::set<Map::DRX_TUNE2>(DRX_TUNE2{0x331A0052}),  // 16 MHz, PAC 8
    Registers::set<Map::DRX_TUNE4H>(DRX_TUNE4H{0x0028}),    // preamble != 64
    Registers::set<Map::RF_RXCTRLH>(RF_RXCTRLH{0xD8}),      // channel 5
    Registers::set<Map::RF_TXCTRL>(RF_TXCTRL{0x001E3FE0}),  // channel 5
    Registers::set<Map::TC_PGDELAY>(TC_PGDELAY{0xC0}),      // channel 5
    Registers::set<Map::FS_PLLCFG>(FS_PLLCFG{0x0800041D}),  // channel 5
    Registers::set<Map::FS_PLLTUNE>(FS_PLLTUNE{0xBE}),      // channel 5
    Registers::set<Map::LDE_CFG1>(LDE_CFG1{0x0D}),
    Registers::set<Map::LDE_CFG2>(LDE_CFG2{0x1607}),        // 16 MHz
    Registers::set<Map::LDE_REPC>(LDE_REPC{0x428E}),        // preamble code 4
    // CHAN_CTRL for channel 5 TX/RX, RXPRF 16 MHz, TX/RX preamble code 4, std SFD
    Registers::set<Map::CHAN_CTRL>(TX_CHAN{5}, RX_CHAN{5}, RXPRF{PRF::MHZ_16},
                                   TX_PCODE{4}, RX_PCODE{4}),
    Registers::set<Map::TX_ANTD>(TX_ANTD{ANTENNA_DELAY}),
    // RX antenna delay (LDE_RXANTD, 0x2E:1804) -- balances TX_ANTD, else
    // ~half the antenna delay stays uncompensated as a fixed offset
    Registers::set<Map::LDE_RXANTD>(LDE_RXANTD{ANTENNA_DELAY}),
};
} // namespace dw1000

#endif
