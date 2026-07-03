# References

- [dw1000.pdf](dw1000.pdf) — DW1000 User Manual (DecaWave, v2.02). Authoritative register map and semantics.
- [arduino-repo](https://github.com/thotro/arduino-dw1000) — thotro/arduino-dw1000, a mature DW1000 driver. Useful as a reference implementation for register sequences, config values, and timing workarounds.
- [rust-dw1000](https://github.com/braun-embedded/rust-dw1000) — the `dw1000` crate: a modern, hardware-agnostic (embedded-hal) DW1000 driver with a type-state API. Closest prior art to this driver's design; worth reading for the config/LDE-load ordering and SS/DS-TWR timestamp math before implementing ranging.
