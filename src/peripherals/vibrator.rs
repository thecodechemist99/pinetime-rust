//! Control the vibration motos
//!
//! Implementation based upon https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use embassy_nrf::{gpio::Output, peripherals::P0_16};
use embassy_time::Timer;

// /// Controller for the internal vibration motor
// pub struct VibrationMotor<'a> {
//     /// Pin P0.16: High = off, Low = on
//     control_pin: Output<'a, P0_16>,
// }

// impl<'a> VibrationMotor<'a> {
//     /// Initialize vibration controller
//     pub fn init(control_pin: Output<'a, P0_16>) -> Self {
//         Self { control_pin }
//     }

//     /// Pulse the vibration motor once for the specified time in ms.
//     ///
//     /// The default pulse duration is 100ms.
//     #[allow(unused)]
//     pub fn pulse_once(&mut self, duration_ms: Option<u32>) {
//         self.on();
//         self.wait(duration_ms);
//         self.off();
//     }

//     /// Pulse the vibration motor a specified amount of times for
//     /// the specified time in ms.
//     ///
//     /// The default pulse duration is 100ms.
//     #[allow(unused)]
//     pub fn pulse_times(&mut self, duration_ms: Option<u32>, times: u8) {
//         for _ in 0..times {
//             self.pulse_once(duration_ms);
//             self.wait(duration_ms);
//         }
//     }

//     fn on(&mut self) {
//         let _ = self.control_pin.set_low();
//     }

//     fn off(&mut self) {
//         let _ = self.control_pin.set_high();
//     }

//     async fn wait(&mut self, duration: Option<u32>) {
//         Timer::after_micros(match duration {
//             Some(delay) => delay as u64 * 1_000,
//             None => 100_000,
//         })
//         .await;
//     }
// }

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
