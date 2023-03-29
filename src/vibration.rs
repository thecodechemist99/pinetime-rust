//! Control the vibration motos
//! 
//! Implementation based upon https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use nrf52832_hal::{
    gpio::{Output, Pin, PushPull},
    prelude::OutputPin,
};

// use crate::timer_delay::TimerDelay;
use crate::delay::Delay;

pub struct VibrationMotor {
    control_pin: Pin<Output<PushPull>>,
    delay_source: Delay,
}

impl VibrationMotor {
    pub fn init(control_pin: Pin<Output<PushPull>>, delay_source: &mut Delay) -> Self {
        Self { control_pin, delay_source: *delay_source }
    }

    #[allow(unused)]
    pub fn pulse_once(&mut self, duration_ms: Option<u32>) {
        self.on();
        self.wait(duration_ms);
        self.off();
    }

    #[allow(unused)]
    pub fn pulse_times(&mut self, duration_ms: Option<u32>, times: u8) {
        for _ in 0..times {
            self.pulse_once(duration_ms);
            self.wait(duration_ms);
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
                Some(delay) => delay * 1_000,
                None => 100_000,
            }
        );
    }
}