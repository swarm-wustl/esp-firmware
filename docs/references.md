# References

- [dw1000.pdf](dw1000.pdf) — DW1000 User Manual (DecaWave, v2.02). Authoritative register map and semantics.
- [arduino-repo](https://github.com/thotro/arduino-dw1000) — thotro/arduino-dw1000, a mature DW1000 driver. Useful as a reference implementation for register sequences, config values, and timing workarounds.
- [rust-dw1000](https://github.com/braun-embedded/rust-dw1000) — the `dw1000` crate: a modern, hardware-agnostic (embedded-hal) DW1000 driver with a type-state API. Closest prior art to this driver's design; worth reading for the config/LDE-load ordering and SS/DS-TWR timestamp math before implementing ranging.
- [bitcraze/lps-node-firmware](https://github.com/bitcraze/lps-node-firmware) — Bitcraze Loco Positioning node firmware (LGPL-3), used to fly Crazyflie swarms. `src/uwb_twr_tag.c` / `src/uwb_twr_anchor.c` are the strongest reference for double-sided TWR and multi-node scheduling — read these before moving past single-sided TWR to a 3+ node swarm.
- [bitcraze/libdw1000](https://github.com/bitcraze/libdw1000) — Bitcraze's open (not Decawave's restrictively-licensed decadriver) DW1000 driver. Another config/sequence reference. NB: do not copy from Qorvo/Decawave's `decadriver`/DecaRanging — restrictive license; this driver is built only from the manual + the arduino/rust/bitcraze open refs.
