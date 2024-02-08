//! Button control module for PineTime

use debouncr::{debounce_2, Debouncer, Edge, Repeat2};
use embassy_nrf::{
    gpio::{Input, Output},
    peripherals::{P0_13, P0_15},
};
use embassy_time::{Duration, Timer};

struct ButtonConfig<'a> {
    /// Button detection pin (high/low)
    pin_button: Input<'a, P0_13>,
    /// Button enable pin
    pin_enable: Output<'a, P0_15>,
    /// Debouncer for button
    debouncer: Debouncer<u8, Repeat2>,
}

pub struct Button {
    /// Button configuration
    config: ButtonConfig<'static>,
}

impl Button {
    /// Configure button on boot
    pub fn init(button_pin: Input<'static, P0_13>, enable_pin: Output<'static, P0_15>) -> Self {
        Self {
            config: ButtonConfig {
                pin_button: button_pin,
                pin_enable: enable_pin,
                debouncer: debounce_2(false),
            },
        }
    }
    /// Check if button is pressed
    pub async fn pressed(&mut self) -> bool {
        let config = &mut self.config;
        let mut pressed = false;

        // Enable button
        config.pin_enable.set_high();
        // The button needs a short time to give stable outputs
        Timer::after(Duration::from_nanos(1)).await;

        // Check for edge event
        if config.debouncer.update(config.pin_button.is_high()) == Some(Edge::Rising) {
            pressed = true;
        }

        // Button consumes around 34µA when P0.15 is left high.
        // To reduce current consumption, set it low most of the time.
        config.pin_enable.set_low();

        pressed
    }
}
