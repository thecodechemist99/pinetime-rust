//! Touch controler module for PineTime

pub use cst816s::TouchGesture;

use cst816s::CST816S;
use embassy_nrf::{
    gpio::{Input, Output},
    peripherals::{P0_10, P0_28},
    twim::{self, Twim},
};

// #[allow(unused)]
// pub struct TouchController<'a, I, IRQ, RST>
// where
//     I: twim::Instance,
//     IRQ: Pin,
//     RST: Pin,
// {
//     touchpad: CST816S<Twim<'a, I>, Input<'a, IRQ>, Output<'a, RST>>,
// }

// impl<I, IRQ, RST> TouchController<'_, I, IRQ, RST>
// where
//     I: twim::Instance,
//     IRQ: Pin,
//     RST: Pin,
// {
//     #[allow(unused)]
//     pub fn init(mut self, delay: &mut Delay) -> Self {
//         self.touchpad.setup(delay).unwrap();
//         self
//     }

//     pub fn try_event_detected(&mut self) -> Option<TouchGesture> {
//         if let Some(event) = self.touchpad.read_one_touch_event(true) {
//             defmt::info!("Touch event detected");
//             Some(event.gesture)
//         } else {
//             None
//         }
//     }
// }

// impl<'a, I, IRQ, RST> I2CPeripheral<'a, I, IRQ, RST> for TouchController<'a, I, IRQ, RST>
// where
//     I: twim::Instance,
//     IRQ: Pin,
//     RST: Pin,
// {
//     #[allow(unused)]
//     fn new(
//         i2c: Twim<'a, I>,
//         interrupt_pin: Input<'a, IRQ>, // P0.28 : Interrupt (signal to the CPU when a touch event is detected)
//         reset_pin: Option<Output<'a, RST>>,
//     ) -> Result<Self, Error> {
//         match reset_pin {
//             Some(reset) => {
//                 let mut touchpad = CST816S::new(i2c, interrupt_pin, reset);

//                 Ok(Self { touchpad })
//             }
//             None => Err(Error::ResetPinRequired),
//         }
//     }
// }

struct TouchControllerConfig<'a, I>
where
    I: twim::Instance,
{
    /// Touchpad instance
    touchpad: CST816S<Twim<'a, I>, Input<'a, P0_28>, Output<'a, P0_10>>,
}

pub struct TouchController<I>
where
    I: twim::Instance,
{
    /// Touch controller configuration
    config: TouchControllerConfig<'static, I>,
}

impl<I> TouchController<I>
where
    I: twim::Instance,
{
    /// Configure vibrator on boot
    pub fn init(
        twi: Twim<'static, I>,
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
