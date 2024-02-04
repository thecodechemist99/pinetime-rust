//! Accelerometer module for PineTime

use crate::system::i2c::{Error, I2CPeripheral};
use embassy_nrf::{
    gpio::{Input, Output, Pin},
    twim::{self, Twim},
};

#[allow(unused)]
pub struct Accelerometer {}

impl<'a, I, IRQ, RST> I2CPeripheral<'a, I, IRQ, RST> for Accelerometer
where
    I: twim::Instance,
    IRQ: Pin,
    RST: Pin,
{
    #[allow(unused)]
    fn new(
        i2c: Twim<'a, I>,
        interrupt_pin: Input<'a, IRQ>, // P0.08 : Interrupt
        reset_pin: Option<Output<'a, RST>>,
    ) -> Result<Self, Error> {
        Ok(Self {})
    }
}
