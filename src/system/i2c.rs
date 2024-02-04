//! Generic I2C peripheral trait

use embassy_nrf::{
    gpio::{Input, Output, Pin},
    twim::{self, Twim},
};

/// Generic I2C peripheral with interrupt pin
pub trait I2CPeripheral<'a, I, IRQ, RST>
where
    I: twim::Instance,
    IRQ: Pin,
    RST: Pin,
    Self: Sized,
{
    fn new(
        i2c: Twim<'a, I>,
        interrupt_pin: Input<'a, IRQ>,
        reset_pin: Option<Output<'a, RST>>,
    ) -> Result<Self, Error>;
}

#[derive(Debug)]
pub enum Error {
    ResetPinRequired,
}
