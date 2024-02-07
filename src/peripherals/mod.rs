pub(super) mod accelerometer;
pub(super) mod backlight;
pub(super) mod battery;
pub(super) mod display;
pub(super) mod heartrate;
pub(super) mod touch;
pub(super) mod vibration;

/// API for accessing peripherals
struct Peripherals {
    battery: battery::Battery,
    // button: ,
    display: display::DisplayAPI,
}
