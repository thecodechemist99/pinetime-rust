//! Accelerometer module for PineTime

use crate::system::i2c::{Error, I2CPeripheral};
use embedded_hal::blocking::i2c::{Write, Read, WriteRead};
use nrf52832_hal::prelude::{InputPin, StatefulOutputPin};

#[allow(unused)]
pub struct Accelerometer {
    
}

impl<I2C, PINT, RST> I2CPeripheral<I2C, PINT, RST> for Accelerometer
where
    I2C: Write + Read + WriteRead,
    PINT: InputPin,
    RST: StatefulOutputPin,
{
    #[allow(unused)]
    fn new(i2c: I2C, interrupt_pin: PINT, reset_pin: Option<RST>) -> Result<Self, Error> {
        Ok(Self {  })
    }
}
