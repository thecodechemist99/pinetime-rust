//! Control the vibration motos
//! 
//! Implementation based upon https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use nrf52832_hal::{
    gpio::{Output, Pin, PushPull}, prelude::OutputPin, time,
};
use crate::delay::TimerDelay;

pub struct VibrationMotor {
    control_pin: Pin<Output<PushPull>>,
    delay_source: &'static mut TimerDelay,
}

impl VibrationMotor {
    pub fn init(control_pin: Pin<Output<PushPull>>, delay_source: &mut TimerDelay) -> Self {

        Self { control_pin, delay_source }
    }

    pub fn pulse_once(&mut self, duration: Option<u32>) {
        self.on();
        self.wait(duration);
        self.off();
    }

    pub fn pulse_times(&mut self, duration: Option<u32>, times: u8) {
        for _ in 0..times {
            self.pulse_once(duration);
            self.wait(duration);
        }
    }

    pub fn on(&mut self) {
        let _ = self.control_pin.set_low();
    }

    pub fn off(&mut self) {
        let _ = self.control_pin.set_high();
    }

    fn wait(&mut self, duration: Option<u32>) {
        self.delay_source.delay_us(
            match duration {
                Some(delay) => delay,
                None => 100_000,
            }
        );
    }
}