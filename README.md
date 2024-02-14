# PineTime Firmware in Rust

This is a bare-metal firmware implementation for Pine64's PineTime smartwatch, fully written in Rust and based on the [pinetime-rtic](https://github.com/dbrgn/pinetime-rtic) project.

It sets out to expand the existing project, which is only a proof of concept.

## Features from the original project

The original project implemented all of the basic features necessary for a smartwatch:

- Bare-metal Rust with nrf52-hal
- RTIC for concurrency
- embedded-graphics for drawing onto the LCD
- Detect button presses
- Cycle through backlight brightness levels using button
- Show battery charge status and voltage
- Send BLE advertisement frames using the pure-Rust rubble stack

## Changes

Some major changes in the implementation were required because of changes in the embedded rust ecosystem:

- Move from the pretty much abandonned nrf-hal (see [this issue](https://github.com/nrf-rs/nrf-hal/issues/432) on GitHub) to the actively maintained [embassy-nrf](https://github.com/embassy-rs/embassy/tree/main/embassy-nrf) crate.
- With the prior change, the entire code was also ported to embassy, as it now works on stable rust.
- As rubble has not been maintained anymore for a while, this project now uses the [nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice) crate. Depending on the continuous development, the aim is to maybe switch to [bleps](https://github.com/bjoernQ/bleps) at some point, to use an open, rust-based implementation again.

## New features

This project sets out to add the following features to make a usable firmware out of the existing proof of concept:

- Display of (date and) time
- Access to all internal peripherals
- Battery percentage calculation
- Touch support
- Firmware updates without needing access the debugging pins

> **Warning**
> Be aware that right now the project is still in a very early development state. Most of these features are not yet fully or at all implemented.

### Current state

At the moment, the following features have been implemented (on top of the features from the original project):

- ~~A rudimentary UI showing the time and battery state~~
  - This has been removed for now in preparation for a more useful UI interface
- Control of the vibration motor (duration and vibration count can be set)
- A very rudimentary battery percentage calculation (will be replaced with a better implementation at some point)
- Basic touch detection
  - The driver crate used for this currently doesn’t support all touch gestures
  - Currently, just basic touch detection is implemented
- Bluetooth implementation
  - Battery service (does not yet display the actual battery state)
  - Fetch time from a current time service on a connected decive (e.g. smartphone)
- Heart rate measurement with fairly accurate heart rate calculation
- Accelerometer driver
  - Currently only logs accelerometer vectors

## Development

Target MCU: nRF52832 (xxAA)

### Flashing (probe-run)

Install cargo-embed:

    $ cargo install probe-run

Flash the target:

    $ cargo run

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  http://opensource.org/licenses/MIT) at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
