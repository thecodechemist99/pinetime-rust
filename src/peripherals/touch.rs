//! Touch panel module for PineTime

pub use cst816s::TouchGesture;

use crate::system::i2c::{Error, I2CPeripheral};
use cst816s::CST816S;
use embassy_nrf::{
    gpio::{Input, Output, Pin},
    twim::{self, Twim},
};
use embassy_time::Delay;

#[allow(unused)]
pub struct TouchController<'a, I, IRQ, RST>
where
    I: twim::Instance,
    IRQ: Pin,
    RST: Pin,
{
    touchpad: CST816S<Twim<'a, I>, Input<'a, IRQ>, Output<'a, RST>>,
}

impl<I, IRQ, RST> TouchController<'_, I, IRQ, RST>
where
    I: twim::Instance,
    IRQ: Pin,
    RST: Pin,
{
    #[allow(unused)]
    pub fn init(mut self, delay: &mut Delay) -> Self {
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

impl<'a, I, IRQ, RST> I2CPeripheral<'a, I, IRQ, RST> for TouchController<'a, I, IRQ, RST>
where
    I: twim::Instance,
    IRQ: Pin,
    RST: Pin,
{
    #[allow(unused)]
    fn new(
        i2c: Twim<'a, I>,
        interrupt_pin: Input<'a, IRQ>, // P0.28 : Interrupt (signal to the CPU when a touch event is detected)
        reset_pin: Option<Output<'a, RST>>,
    ) -> Result<Self, Error> {
        match reset_pin {
            Some(reset) => {
                let mut touchpad = CST816S::new(i2c, interrupt_pin, reset);

                Ok(Self { touchpad })
            }
            None => Err(Error::ResetPinRequired),
        }
    }
}
