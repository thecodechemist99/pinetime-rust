#![no_std]
#![no_main]

mod peripherals;
mod system;

// Panic handler and debugging
use defmt::unwrap;

use defmt_rtt as _;
use panic_probe as _;

// Device
use debouncr::{debounce_2, Edge};
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    peripherals::{P0_10, P0_13, P0_28, SPI2, TWISPI1},
    saadc::{self, ChannelConfig, Resolution, Saadc},
    spim,
    twim::{self, Twim},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<TWISPI1>;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<SPI2>;
});

// Crate
use peripherals::{
    backlight::Backlight,
    battery::{BatteryInfo, BatteryStatus},
    display::Display,
    touch::{TouchController, TouchGesture},
    vibration::VibrationMotor,
};
use system::{bluetooth::Bluetooth, i2c::I2CPeripheral};

// Others
use chrono::NaiveDateTime;
// use fugit::{ExtU32, TimerInstantU32 as InstantU32};

// Include current UTC epoch at compile time
include!(concat!(env!("OUT_DIR"), "/utc.rs"));
const TIMEZONE: i32 = 2 * 3_600;

// PineTime has a 32 MHz HSE (HFXO) and a 32.768 kHz LSE (LFXO)
#[allow(dead_code)]
const HFXO_FREQ_HZ: u32 = 32_000_000u32;
#[allow(dead_code)]
const LFXO_FREQ_HZ: u32 = 32_768u32;

// Communication channels
static BATTERY_STATUS: Signal<ThreadModeRawMutex, BatteryInfo> = Signal::new();
static INCREASE_BRIGHTNESS: Signal<ThreadModeRawMutex, bool> = Signal::new();
static NOTIFY: Signal<ThreadModeRawMutex, u8> = Signal::new();
static TIME: Signal<ThreadModeRawMutex, NaiveDateTime> = Signal::new();
static TOUCH_EVENT: Signal<ThreadModeRawMutex, TouchGesture> = Signal::new();

/// Called when button is pressed without bouncing for 10 (5 * 2) ms.
#[embassy_executor::task(pool_size = 1)]
async fn button_pressed() {
    INCREASE_BRIGHTNESS.signal(true);
}

/// Check for notifications every 100ms
#[embassy_executor::task(pool_size = 1)]
async fn notify(mut motor: VibrationMotor<'static>) {
    loop {
        if NOTIFY.signaled() {
            // Vibrate signaled amount of times
            let count = NOTIFY.wait().await;
            match count {
                1 => motor.pulse_once(Some(200)),
                _ => motor.pulse_times(Some(200), count),
            }
        }

        // Re-schedule the timer interrupt in 100ms
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Fetch the battery status from the hardware.
#[embassy_executor::task(pool_size = 1)]
async fn update_battery_status(mut battery: BatteryStatus<'static>) {
    loop {
        if battery.update().unwrap() {
            // Battery status changed
            defmt::info!("Battery status updated");
            BATTERY_STATUS.signal(battery.info());
        };

        // Re-schedule the timer interrupt in 1s
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Update backlight brightness
#[embassy_executor::task(pool_size = 1)]
async fn update_brightness(mut backlight: Backlight<'static>) {
    loop {
        if INCREASE_BRIGHTNESS.wait().await {
            if backlight.get_brightness() < 7 {
                backlight.brighter().unwrap();
            } else {
                backlight.off();
            }
        }
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn update_lcd(mut display: Display<'static, SPI2>) {
    loop {
        if BATTERY_STATUS.signaled() {
            display.update_battery_status(BATTERY_STATUS.wait().await);
        }

        if TIME.signaled() {
            display.update_time(TIME.wait().await, TIMEZONE);
        }

        // Re-schedule the timer interrupt in 1s
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Get the current time.
#[embassy_executor::task(pool_size = 1)]
async fn update_time() {
    let mut tick = Ticker::every(Duration::from_secs(1));
    loop {
        // Calculate current time
        let now = Instant::now();
        let utc = NaiveDateTime::from_timestamp_opt(UTC_EPOCH + now.elapsed().as_secs() as i64, 0)
            .unwrap();

        // Send time to channel
        TIME.signal(utc);

        // Re-schedule the timer interrupt
        tick.next().await;
    }
}

/// Polls the button state every 10ms
#[embassy_executor::task(pool_size = 1)]
async fn poll_button(pin: Input<'static, P0_13>) {
    let mut debounce = debounce_2(false);
    loop {
        // Poll button
        let edge = debounce.update(pin.is_high());
        if edge == Some(Edge::Rising) {
            defmt::info!("Button pressed!");
            unwrap!(Spawner::for_current_executor()
                .await
                .spawn(button_pressed()));
        }

        // pin.wait_for_rising_edge().await;
        // defmt::info!("Button pressed!");
        // pin.wait_for_falling_edge().await;
        // defmt::info!("Button released!");

        // Re-schedule the timer interrupt in 10ms
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Polls the touch interrupt pin every 2ms
#[embassy_executor::task(pool_size = 1)]
async fn poll_touch(mut touch: TouchController<'static, TWISPI1, P0_28, P0_10>) {
    loop {
        // Check for touch event
        if let Some(gesture) = touch.try_event_detected() {
            TOUCH_EVENT.signal(gesture);
        }

        // Re-schedule the timer interrupt in 2ms
        Timer::after(Duration::from_millis(2)).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nrf::init(Default::default());
    defmt::info!("Initializing");

    // Initialize SAADC
    let mut saadc_config = saadc::Config::default();
    // Set resolution to 12bit, necessary for correct battery status calculation
    saadc_config.resolution = Resolution::_12BIT;
    // Pin P0.31: Voltage level
    let channel_config = ChannelConfig::single_ended(&mut p.P0_31);
    let saadc = Saadc::new(p.SAADC, Irqs, saadc_config, [channel_config]);
    saadc.calibrate().await;

    // Initialize Backlight
    let mut backlight = Backlight::init(
        Output::new(p.P0_14, Level::High, OutputDrive::Standard),
        Output::new(p.P0_22, Level::High, OutputDrive::Standard),
        Output::new(p.P0_23, Level::High, OutputDrive::Standard),
        0,
    );

    // Initalize Battery
    let battery = BatteryStatus::init(Input::new(p.P0_12, Pull::None), saadc)
        .await
        .unwrap();
    BATTERY_STATUS.signal(battery.info());

    // Initialize Button
    let _ = Output::new(p.P0_15, Level::High, OutputDrive::Standard);
    let button = Input::new(p.P0_13, Pull::None);

    // Initialize vibration motor
    let vibration = VibrationMotor::init(Output::new(p.P0_16, Level::High, OutputDrive::Standard));

    // Initialize Bluetooth

    // Initialize I2C
    let mut i2c_config = twim::Config::default();
    // Use I2C at 400KHz (the fastest clock available on the nRF52832),
    i2c_config.frequency = twim::Frequency::K400;

    let i2c = Twim::new(p.TWISPI1, Irqs, p.P0_06, p.P0_07, i2c_config);

    // Initialize SPI
    let mut spim_config = spim::Config::default();
    // Use SPI at 8MHz (the fastest clock available on the nRF52832),
    // otherwise refreshing will be super slow.
    spim_config.frequency = spim::Frequency::M8;
    // SPI must be used in mode 3. Mode 0 (the default) won't work.
    spim_config.mode = spim::MODE_3;

    let spim = spim::Spim::new(p.SPI2, Irqs, p.P0_02, p.P0_04, p.P0_03, spim_config);

    // Initialize LCD
    let display = Display::init(
        spim,
        Output::new(p.P0_25, Level::Low, OutputDrive::Standard),
        Output::new(p.P0_18, Level::Low, OutputDrive::Standard),
        Output::new(p.P0_26, Level::Low, OutputDrive::Standard),
        &mut Delay,
    );
    backlight.set(2).unwrap();

    // Initialize touch controller
    let touch = TouchController::new(
        i2c,
        Input::new(p.P0_28, Pull::Up), // Touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
        Some(Output::new(p.P0_10, Level::High, OutputDrive::Standard)), // Touchpad reset pin: P0.10/NFC2 (TP_RESET)
    )
    .unwrap()
    .init(&mut Delay);

    defmt::info!("Initialization finished");

    // Schedule tasks
    unwrap!(_spawner.spawn(poll_button(button)));
    unwrap!(_spawner.spawn(poll_touch(touch)));
    unwrap!(_spawner.spawn(update_battery_status(battery)));
    unwrap!(_spawner.spawn(update_brightness(backlight)));
    unwrap!(_spawner.spawn(update_lcd(display)));
    unwrap!(_spawner.spawn(update_time()));
    unwrap!(_spawner.spawn(notify(vibration)));
}
