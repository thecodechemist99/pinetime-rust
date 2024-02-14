//! Control the vibration motos
//!
//! Implementation based upon https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use embassy_nrf::{gpio::Output, peripherals::P0_16};
use embassy_time::Timer;

#[allow(unused)]
#[derive(Clone, Copy)]
/// Pulse length in milliseconds
pub enum PulseLength {
    /// 200ms pulse
    SHORT = 200,
    /// 400 ms pulse
    LONG = 400,
}

struct VibratorConfig<'a> {
    /// Motor enable pin (inverted)
    pin_enable: Output<'a, P0_16>,
}

#[allow(unused)]
pub struct Vibrator {
    /// Vibrator configuration
    config: VibratorConfig<'static>,
}

impl Vibrator {
    /// Configure vibrator on boot
    pub fn init(enable_pin: Output<'static, P0_16>) -> Self {
        Self {
            config: VibratorConfig {
                pin_enable: enable_pin,
            },
        }
    }
    #[allow(unused)]
    /// Pulse the vibrator for the set amount of times and
    /// the specified pulse length.
    pub async fn pulse(&mut self, length: PulseLength, times: Option<u8>) {
        let count = *times.clone().get_or_insert(1);
        for _ in 0..count {
            self.config.pin_enable.set_low();
            Timer::after_millis(length as u64).await;
        }
    }
}
