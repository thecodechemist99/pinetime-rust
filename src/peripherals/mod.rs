use embassy_nrf::spim;

pub(super) mod accelerometer;
pub(super) mod backlight;
pub(super) mod battery;
pub(super) mod button;
pub(super) mod display;
pub(super) mod heartrate;
pub(super) mod touch;
pub(super) mod vibrator;

/// API for accessing peripherals
pub struct Peripherals<SPI>
where
    SPI: spim::Instance,
{
    battery: battery::Battery,
    button: button::Button,
    display: display::Display<SPI>,
    vibrator: vibrator::Vibrator,
}
