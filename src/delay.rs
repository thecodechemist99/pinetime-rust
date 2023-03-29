//! A delay driver based on `asm::delay`.
//! 
//! Adaptation of the original implementation from https://docs.rs/cortex-m/0.7.7/src/cortex_m/delay.rs.html
//! with some inspiration from https://github.com/ChrisChrisLoLo/keezyboost40/blob/master/firmware/keezus/src/delay.rs.
//! 
//! This adaptation is necessary for projects using the `cortex-m-rtic` framework
//! because the SysTick-based `cortex_m::delay` cannot be used as SYST is moved
//! to the RTIC framework (https://github.com/rtic-rs/cortex-m-rtic/issues/523).

use embedded_hal::blocking::delay::{
    DelayMs,
    DelayUs
};

/// `asm::delay` as a delay provider
#[derive(Debug, Clone, Copy)]
pub struct Delay {
    frequency: u32,
}

impl Delay {
    /// Configures `asm::delay` as a delay provider.
    pub fn new(frequency: u32) -> Delay {
        Delay { frequency }
    }

    /// Delay using `asm::delay` for a certain duration, in µs.
    pub fn delay_us(&mut self, us: u32) {
        let ticks_per_second = self.frequency;
        let ticks_per_microsecond = ticks_per_second/1_000_000;

        // Iterate rather than multiply to prevent buffer overflow
        for _ in 0..us {
            cortex_m::asm::delay(ticks_per_microsecond);
        }
    }

    /// Delay using `asm::delay` for a certain duration, in ms.
    pub fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms);
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us);
    }
}
