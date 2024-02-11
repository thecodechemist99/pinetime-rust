//! Touch controler module for PineTime

pub use cst816s::TouchGesture;

use cst816s::CST816S;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_nrf::{
    gpio::{Input, Output},
    peripherals::{P0_10, P0_28},
    twim::{self, Twim},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

struct TouchControllerConfig<'a, TWI>
where
    TWI: twim::Instance,
{
    /// Touchpad instance
    touchpad:
        CST816S<I2cDevice<'a, NoopRawMutex, Twim<'a, TWI>>, Input<'a, P0_28>, Output<'a, P0_10>>,
}

pub struct TouchController<TWI>
where
    TWI: twim::Instance,
{
    /// Touch controller configuration
    config: TouchControllerConfig<'static, TWI>,
}

impl<TWI> TouchController<TWI>
where
    TWI: twim::Instance,
{
    /// Configure vibrator on boot
    pub fn init(
        twi: I2cDevice<'static, NoopRawMutex, Twim<'static, TWI>>,
        interrupt_pin: Input<'static, P0_28>,
        reset_pin: Output<'static, P0_10>,
    ) -> Self {
        Self {
            config: TouchControllerConfig {
                touchpad: CST816S::new(twi, interrupt_pin, reset_pin),
            },
        }
    }
    /// Check for new touch event
    pub fn try_event_detected(&mut self) -> Option<TouchGesture> {
        if let Some(event) = self.config.touchpad.read_one_touch_event(true) {
            return Some(event.gesture);
        }
        None
    }
}
