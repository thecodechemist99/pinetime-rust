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

## New features

This project sets out to add the following features to make a usable firmware out of the existing proof of concept:

- Display of (date and) time
- Access to all internal peripherals
- Battery percentage calculation
- Touch support
- Firmware updates without needing access the debugging pins

It also updated the existing code base to work with the current versions of all used external crates. For this it was necessary to use a custom fork of rubble, as the original project is no more actively maintained.

> **Warning**
> Be aware that right now the project is still in a very early development state. Most of these features are not yet implemented.

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
