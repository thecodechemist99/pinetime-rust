//! Backlight control
//!
//! Implementation based upon https://github.com/dbrgn/pinetime-rtic/blob/master/pinetime-rtic/src/backlight.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use embassy_nrf::{
    gpio::Output,
    peripherals::{P0_14, P0_22, P0_23},
};

/// Control the backlight.
///
/// There are three active-low backlight pins, each connected to a FET that
/// toggles backlight power through a resistor.
///
/// - Low: 2.2 kΩ
/// - Mid: 100 Ω
/// - High: 30 Ω
///
/// Through combinations of these pins, 7 brightness levels (+ off) can be
/// configured.
#[allow(unused)]
pub struct Backlight<'a> {
    low: Output<'a, P0_14>,
    mid: Output<'a, P0_22>,
    high: Output<'a, P0_23>,

    /// The current brightness level (value between 0 and 7).
    brightness: u8,
}

impl<'a> Backlight<'a> {
    /// Initialize the backlight with the specified level (0–7).
    #[allow(unused)]
    pub fn init(
        low: Output<'a, P0_14>,
        mid: Output<'a, P0_22>,
        high: Output<'a, P0_23>,
        brightness: u8,
    ) -> Self {
        let mut backlight = Self {
            low,
            mid,
            high,
            brightness,
        };
        backlight.set(brightness);
        backlight
    }

    /// Set the brightness level between 0 (off) and 7 (max brightness).
    pub fn set(&mut self, brightness: u8) -> Result<(), Error> {
        match brightness {
            0..=7 => {
                defmt::debug!("Setting backlight brightness to {}", brightness);

                if brightness & 0x01 > 0 {
                    self.low.set_low();
                } else {
                    self.low.set_high();
                }
                if brightness & 0x02 > 0 {
                    self.mid.set_low();
                } else {
                    self.mid.set_high();
                }
                if brightness & 0x04 > 0 {
                    self.high.set_low();
                } else {
                    self.high.set_high();
                }
                self.brightness = brightness;

                Ok(())
            }
            _ => Err(Error::OutOfBounds),
        }
    }

    /// Turn off the backlight.
    #[allow(unused)]
    pub fn off(&mut self) {
        self.set(0);
    }

    /// Increase backlight brightness.
    #[allow(unused)]
    pub fn brighter(&mut self) -> Result<(), Error> {
        self.set(self.brightness + 1)
    }

    /// Decrease backlight brightness.
    #[allow(unused)]
    pub fn darker(&mut self) -> Result<(), Error> {
        self.set(self.brightness - 1)
    }

    /// Return the current brightness level (value between 0 and 7).
    #[allow(unused)]
    pub fn get_brightness(&self) -> u8 {
        self.brightness
    }
}

#[derive(Debug)]
pub enum Error {
    OutOfBounds,
}
