//! Generic I2C peripheral trait

use embedded_hal::blocking::i2c::{Write, Read, WriteRead};
use nrf52832_hal::prelude::{InputPin, StatefulOutputPin};

/// Generic I2C peripheral with interrupt pin
pub trait I2CPeripheral<I2C, PINT, RST>
where
    I2C: Write + Read + WriteRead,
    PINT: InputPin,
    RST: StatefulOutputPin,
    Self: Sized,
{
    fn new(i2c: I2C, interrupt_pin: PINT, reset_pin: Option<RST>) -> Result<Self, Error>;
}

#[derive(Debug)]
pub enum Error {
    ResetPinRequired,
}