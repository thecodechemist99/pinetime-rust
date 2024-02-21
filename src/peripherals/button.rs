//! Button control module for PineTime

use debouncr::{debounce_2, Debouncer, Edge, Repeat2};
use embassy_nrf::{
    gpio::{Input, Output},
    peripherals::{P0_13, P0_15},
};
use embassy_time::Timer;

struct ButtonConfig<'a> {
    /// Button detection pin (high/low)
    pin_in: Input<'a, P0_13>,
    /// Button enable pin
    pin_out: Output<'a, P0_15>,
    /// Debouncer for button
    debouncer: Debouncer<u8, Repeat2>,
}

pub struct Button {
    /// Button configuration
    config: ButtonConfig<'static>,
}

impl Button {
    /// Configure button on boot
    pub fn init(in_pin: Input<'static, P0_13>, out_pin: Output<'static, P0_15>) -> Self {
        Self {
            config: ButtonConfig {
                pin_in: in_pin,
                pin_out: out_pin,
                debouncer: debounce_2(false),
            },
        }
    }
    /// Check if button is pressed
    pub async fn pressed(&mut self) -> bool {
        let mut pressed = false;

        // Enable button
        self.config.pin_out.set_high();
        // The button needs a short time to give stable outputs
        Timer::after_nanos(1).await;

        // Check for edge event
        if self.config.debouncer.update(self.config.pin_in.is_high()) == Some(Edge::Rising) {
            pressed = true;
        }

        // Button consumes around 34µA when P0.15 is left high.
        // To reduce current consumption, set it low most of the time.
        self.config.pin_out.set_low();

        pressed
    }
}
