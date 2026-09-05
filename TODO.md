# TODO: DWM1000 Two-Way Ranging → Position Tracking

Goal: range between two DWM1000 devices via two-way ranging (time-of-flight),
then extend to multilateration for position tracking.

## Algorithm choice

Two-way ranging (TWR) is the right approach. One-way ToF would require sub-ns
clock sync between devices (light travels ~30cm/ns, target is ±10cm) — not
feasible. TWR measures round-trip time on a single device's clock, sidestepping
clock sync.

### Start with Single-Sided TWR (SS-TWR) — easiest, less accurate

Only 2 messages:

```
Initiator                          Responder
   |---------- Poll -------------->|  Responder notes T_RP (rx)
   |   (notes T_SP on tx)          |
   |                               |  waits / turns around
   |<-------- Response ------------|  Responder notes T_SR (tx),
   |  (notes T_RR on rx)           |  embeds (T_SR - T_RP) in msg
```

Math (on initiator):
```
T_round  = T_RR - T_SP            (measured on initiator clock)
T_reply  = T_SR - T_RP            (measured on responder clock, sent in payload)
ToF      = (T_round - T_reply) / 2
distance = ToF * c
```

- Easiest: no delayed-TX scheduling needed for a first version; responder
  reports its actual T_reply.
- Less accurate: the two crystals drift, and drift multiplies T_reply. Fine for
  a short-range bring-up demo.

### Upgrade later to Double-Sided TWR (DS-TWR) — manual Appendix 3

3 messages (Poll, Response, Final). Removes the clock-drift term by averaging two
round trips:
```
TOF = (2*T_RR - T_SP - 2*T_SR + T_RP + T_RF - T_SF) / 4
```
Works best when RX-to-TX response time (T_RSP) is the same at both ends.

## Current codebase status

`DWMRegisterView` / `DWM` today: register read/write, bit manipulation, TX config
(PRF, bitrate, preamble), hard_reset. Already have `DWMTimestamp` +
SYS_TIME/TX_TIME/RX_TIME register IDs.

Missing: transmit-a-frame / receive-a-frame / event-polling machinery.

## What to add

### 1. New register IDs (DWMRegisterID)

| Reg                 | Addr | Purpose                                                  |
|---------------------|------|----------------------------------------------------------|
| SYS_CTRL            | 0x0D | Start TX (TXSTRT), start RX (RXENAB), delayed TX (TXDLYS) |
| RX_FINFO            | 0x10 | Length of received frame                                  |
| RX_BUFFER           | 0x11 | Received frame payload                                    |
| DX_TIME             | 0x0A | Delayed send/receive time (for DS-TWR later)             |
| TX_ANTD             | 0x18 | Transmit antenna delay                                    |
| SYS_CFG             | 0x04 | System config (RXAUTR auto-reenable, etc.)              |
| SYS_MASK            | 0x0E | IRQ mask (optional; can poll instead)                   |

Note: already have SYSTEM_EVENT_STATUS (0x0F) = SYS_STATUS. Poll it for TXFRS
(TX frame sent) and RXDFR/RXFCG (RX data frame good).

### 2. New methods on DWM

- `send_frame(std::span<const std::byte>)` — write payload to TX_BUFFER, set frame
  length in TX_FCTRL, set TXSTRT in SYS_CTRL, poll SYS_STATUS for TXFRS.
- `receive_frame()` -> payload + RX_TIME timestamp — set RXENAB, poll SYS_STATUS
  for RXFCG, read RX_FINFO length, read RX_BUFFER, read RX_TIME.
- `get_tx_timestamp()` / `get_rx_timestamp()` — mostly have these via register
  views (TX_TIME/RX_TIME .value() returns DWMTimestamp).
- `set_antenna_delay(uint16_t)` — write TX_ANTD and LDE_RXANTD (sub-register
  0x2E:1804).

### 3. New Ranging layer (e.g. components/dwm/ranging.h)

Host-side state machine: `initiator_range()` and `responder_serve()` built on the
send_frame/receive_frame primitives. SS-TWR math lives here. Keep separate from
DWM (the chip driver) — the algorithm is host software, not a chip feature
(per manual).

### 4. CRITICAL: LDE microcode load on every power-up

Must load the LDE (leading-edge detection) algorithm from ROM into RAM before
timestamps are valid, else RX_TIME is garbage. Specific OTP/PMSC dance:
PMSC_CTRL0 clock config -> OTP_CTRL LDELOAD -> wait -> restore clocks.
Add to init sequence right after hard_reset().

### 5. Antenna delay calibration

For first demo: set TX antenna delay = 0, put combined delay in RX (manual says
fine when both ends identical). Expect a constant ~1m offset until calibrated.
Calibrate later (manual section 8.3): ~1000 ranges at a known separation, tune
delay until average matches. Reported range varies ~2.15 mm/degC and ~5.35 cm/VBATT.

## Minimal path to "it works"

1. Add LDE load to init + the registers above.
2. Implement send_frame / receive_frame with status polling (no IRQs yet).
3. Implement SS-TWR: one ESP32 runs initiator_range() in a loop, the other runs
   responder_serve().
4. Print computed distance. Expect a fixed offset (uncalibrated antenna delay) but
   distances that change correctly as you move them.
5. Once one link works: calibrate antenna delay -> upgrade to DS-TWR -> add
   multiple anchors -> multilateration for actual position.

## Reference

DW1000 User Manual v2.02 (FCC-ID 2AAXVTNTMOD1). Key sections:
- Appendix 3: Two-Way Ranging (algorithm + range calc)
- Section 8.3: IC Calibration – Antenna Delay
- Section 3.2: Transmission timestamp / 3.3: Delayed Transmission
- Section 4.1.6: RX Message timestamp
