//! Touch panel module for PineTime

pub use cst816s::TouchGesture;

use crate::system::i2c::{Error, I2CPeripheral};
use core::fmt::Debug;
use cst816s::CST816S;
use embedded_hal::{delay::DelayNs, i2c::I2c};
use nrf52832_hal::prelude::{InputPin, StatefulOutputPin};

#[allow(unused)]
pub struct TouchController<I2C, PINT, RST>
where
    I2C: I2c,
    PINT: InputPin,
    RST: StatefulOutputPin,
{
    touchpad: CST816S<I2C, PINT, RST>,
}

impl<I2C, PINT, RST> I2CPeripheral<I2C, PINT, RST> for TouchController<I2C, PINT, RST>
where
    I2C: I2c,
    PINT: InputPin,
    RST: StatefulOutputPin,
{
    #[allow(unused)]
    fn new(i2c: I2C, interrupt_pin: PINT, reset_pin: Option<RST>) -> Result<Self, Error> {
        match reset_pin {
            Some(reset) => {
                let mut touchpad = CST816S::new(i2c, interrupt_pin, reset);

                Ok(Self { touchpad })
            }
            None => Err(Error::ResetPinRequired),
        }
    }
}

impl<I2C, PINT, RST> TouchController<I2C, PINT, RST>
where
    I2C: I2c,
    PINT: InputPin,
    RST: StatefulOutputPin,
    RST::Error: Debug,
{
    #[allow(unused)]
    pub fn init(mut self, delay: &mut impl DelayNs) -> Self {
        self.touchpad.setup(delay).unwrap();
        self
    }

    pub fn try_event_detected(&mut self) -> Option<TouchGesture> {
        if let Some(event) = self.touchpad.read_one_touch_event(true) {
            defmt::info!("Touch event detected");
            Some(event.gesture)
        } else {
            None
        }
    }
}
