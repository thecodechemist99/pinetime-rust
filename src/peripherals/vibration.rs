//! Control the vibration motos
//! 
//! Implementation based upon https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use nrf52832_hal::{
    gpio::{Output, Pin, PushPull},
    prelude::OutputPin,
};

use crate::system::delay::Delay;

/// Controller for the internal vibration motor
pub struct VibrationMotor {
    /// Pin P0.16: High = off, Low = on
    control_pin: Pin<Output<PushPull>>,
    delay_source: Delay,
}

impl VibrationMotor {
    /// Initialize vibration controller
    pub fn init(control_pin: Pin<Output<PushPull>>, delay_source: &mut Delay) -> Self {
        Self { control_pin, delay_source: *delay_source }
    }

    /// Pulse the vibration motor once for the specified time in ms.
    /// 
    /// The default pulse duration is 100ms.
    #[allow(unused)]
    pub fn pulse_once(&mut self, duration_ms: Option<u32>) {
        self.on();
        self.wait(duration_ms);
        self.off();
    }

    /// Pulse the vibration motor a specified amount of times for
    /// the specified time in ms.
    /// 
    /// The default pulse duration is 100ms.
    #[allow(unused)]
    pub fn pulse_times(&mut self, duration_ms: Option<u32>, times: u8) {
        for _ in 0..times {
            self.pulse_once(duration_ms);
            self.wait(duration_ms);
        }
    }

    fn on(&mut self) {
        let _ = self.control_pin.set_low();
    }

    fn off(&mut self) {
        let _ = self.control_pin.set_high();
    }

    fn wait(&mut self, duration: Option<u32>) {
        self.delay_source.delay_us(
            match duration {
                Some(delay) => delay * 1_000,
                None => 100_000,
            }
        );
    }
}