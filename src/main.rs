#![no_std]
#![no_main]

mod peripherals;
mod system;

// Panic handler and debugging
use defmt::unwrap;

use defmt_rtt as _;
use panic_probe as _;

// Core
use core::cell::RefCell;
use static_cell::StaticCell;

// System
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    interrupt::{self, InterruptExt, Priority},
    peripherals::{SPI2, TWISPI1},
    saadc::{self, ChannelConfig, Resolution, Saadc},
    spim,
    twim::{self, Twim},
};
use embassy_sync::{
    blocking_mutex::{raw::ThreadModeRawMutex, NoopMutex},
    signal::Signal,
};
use embassy_time::{Duration, Ticker, Timer};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<TWISPI1>;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<SPI2>;
});

// BLE
use nrf_softdevice::Softdevice;

// Crate
use peripherals::{
    accelerometer::Accelerometer,
    battery::Battery,
    button::Button,
    display::{BacklightPins, Brightness, Display},
    heartrate::HeartRateMonitor,
    touch::{TouchController, TouchGesture},
    vibrator::{PulseLength, Vibrator},
};
use system::{
    bluetooth::Bluetooth,
    time::{TimeManager, TimeReference},
};

// Others
use chrono::{NaiveDateTime, Timelike};

/// Shared I2C bus
static I2C_BUS: StaticCell<NoopMutex<RefCell<Twim<TWISPI1>>>> = StaticCell::new();

// Communication channels
static BATTERY_STATUS: Signal<ThreadModeRawMutex, (u8, bool)> = Signal::new();
static CTS_TIME: Signal<ThreadModeRawMutex, TimeReference> = Signal::new();
static INCREASE_BRIGHTNESS: Signal<ThreadModeRawMutex, bool> = Signal::new();
static NOTIFY: Signal<ThreadModeRawMutex, u8> = Signal::new();
static TIME: Signal<ThreadModeRawMutex, NaiveDateTime> = Signal::new();
static TOUCH_EVENT: Signal<ThreadModeRawMutex, TouchGesture> = Signal::new();

/// BLE runner task
#[embassy_executor::task(pool_size = 1)]
async fn ble_runner(mut ble: Bluetooth) {
    // Start softdevice
    unwrap!(Spawner::for_current_executor()
        .await
        .spawn(softdevice_task(ble.get_sd())));

    // Run GATT Server
    loop {
        let e = ble.run_gatt_server(&CTS_TIME).await;
        defmt::info!(
            "gatt_server run exited with error: {:?}",
            defmt::Debug2Format(&e)
        );
    }
}

/// Called when button is pressed without bouncing for 10 (5 * 2) ms.
#[embassy_executor::task(pool_size = 1)]
async fn button_pressed() {
    INCREASE_BRIGHTNESS.signal(true);
}

/// Check for notifications every 100ms
#[embassy_executor::task(pool_size = 1)]
async fn notify(mut vibrator: Vibrator) {
    loop {
        if NOTIFY.signaled() {
            // Vibrate signaled amount of times
            let count = NOTIFY.wait().await;
            match count {
                1 => vibrator.pulse(PulseLength::SHORT, None).await,
                _ => vibrator.pulse(PulseLength::SHORT, Some(count)).await,
            }
        }

        // Re-schedule the timer interrupt in 100ms
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Fetch the battery status from the hardware.
#[embassy_executor::task(pool_size = 1)]
async fn update_battery_status(mut battery: Battery) {
    loop {
        let status = (battery.get_percent().await, battery.is_charging());
        BATTERY_STATUS.signal(status);

        // Re-schedule the timer interrupt in 1s
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Fetch the heart rate measurement from the hardware.
#[embassy_executor::task(pool_size = 1)]
async fn update_heart_rate(mut hrm: HeartRateMonitor<TWISPI1>) {
    let mut tick = Ticker::every(Duration::from_millis(100));
    let mut last_bpm = 0;
    hrm.enable();
    loop {
        let heart_rate = hrm.start_measurement().await;

        if let Some(hr) = heart_rate {
            last_bpm = hr;
            defmt::debug!("Heart rate: {}", hr);
        } else {
            defmt::debug!("Not enough data.");
        }
        defmt::info!("Heart rate: {}", last_bpm);

        // Re-schedule the timer interrupt in 100ms
        tick.next().await;
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn update_lcd(mut display: Display<SPI2>) {
    let mut tick = Ticker::every(Duration::from_secs(1));
    loop {
        if INCREASE_BRIGHTNESS.signaled() {
            display.set_brightness(match display.get_brightness() {
                Brightness::LEVEL0 => Brightness::LEVEL1,
                Brightness::LEVEL1 => Brightness::LEVEL2,
                Brightness::LEVEL2 => Brightness::LEVEL3,
                Brightness::LEVEL3 => Brightness::LEVEL4,
                Brightness::LEVEL4 => Brightness::LEVEL5,
                Brightness::LEVEL5 => Brightness::LEVEL6,
                Brightness::LEVEL6 => Brightness::LEVEL7,
                Brightness::LEVEL7 => Brightness::LEVEL0,
            });
        }

        if BATTERY_STATUS.signaled() {
            let (percent, charging) = BATTERY_STATUS.wait().await;
            defmt::info!(
                "Battery status: {} ({})",
                percent,
                if charging { "charging" } else { "discharging" }
            );
            // display.update_battery_status(battery);
        }

        if TIME.signaled() {
            let time = TIME.wait().await;
            defmt::info!(
                "Current time: {}:{}:{}",
                time.time().hour(),
                time.time().minute(),
                time.time().second(),
            );
            // display.update_time(time);
        }

        // Re-schedule the timer interrupt in 1s
        tick.next().await;
    }
}

/// Update the current time.
#[embassy_executor::task(pool_size = 1)]
async fn update_time(mut time_manager: TimeManager) {
    let mut tick = Ticker::every(Duration::from_secs(1));
    loop {
        // Update time from CTS
        if CTS_TIME.signaled() {
            time_manager.set_time(CTS_TIME.wait().await);
        }

        // Send current time to channel
        TIME.signal(time_manager.get_time());

        // Re-schedule the timer interrupt
        tick.next().await;
    }
}

/// Poll the button state every 10ms
#[embassy_executor::task(pool_size = 1)]
async fn poll_button(mut button: Button) {
    loop {
        INCREASE_BRIGHTNESS.signal(button.pressed().await);

        // Re-schedule the timer interrupt in 10ms
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Check for new touch event every 2ms
#[embassy_executor::task(pool_size = 1)]
async fn poll_touch(mut touch: TouchController<TWISPI1>) {
    loop {
        // Check for touch event
        if let Some(gesture) = touch.try_event_detected() {
            TOUCH_EVENT.signal(gesture);
        }

        // Re-schedule the timer interrupt in 2ms
        Timer::after(Duration::from_millis(2)).await;
    }
}

/// Softdevice runner task
#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    // Configure interrupt priorities to exclude 0 (default), 1, and 4,
    // which are reserved for the SoftDevice
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let mut p = embassy_nrf::init(config);

    defmt::info!("Initializing system ...");

    // == Initialize Timekeeping ==
    let time_manager = TimeManager::init();
    unwrap!(_spawner.spawn(update_time(time_manager)));

    // == Initialize I2C/TWI ==
    let mut twi_config = twim::Config::default();
    // Use TWI at 400KHz (fastest clock available on the nRF52832)
    twi_config.frequency = twim::Frequency::K400;
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(interrupt::Priority::P3);
    let twi = Twim::new(p.TWISPI1, Irqs, p.P0_06, p.P0_07, twi_config);
    let i2c_bus = NoopMutex::new(RefCell::new(twi));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    // Initialize SPI
    let mut spim_config = spim::Config::default();
    // Use SPI at 8MHz (fastest clock available on the nRF52832)
    spim_config.frequency = spim::Frequency::M8;
    // SPI must be used in mode 3, node 0 (the default) won't work.
    spim_config.mode = spim::MODE_3;
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SPIM2_SPIS2_SPI2.set_priority(interrupt::Priority::P3);

    // == Initialize Bluetooth ==
    let ble = Bluetooth::init("PineTime");
    unwrap!(_spawner.spawn(ble_runner(ble)));

    defmt::info!("Initializing peripherals ...");

    // == Initialize Accelerometer ==
    let accelerometer = Accelerometer::init(I2cDevice::new(i2c_bus));

    // == Initalize ADC ==
    let mut adc_config = saadc::Config::default();
    // Ensure correct battery status calculation
    adc_config.resolution = Resolution::_12BIT;
    let channel_config = ChannelConfig::single_ended(&mut p.P0_31);
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SAADC.set_priority(interrupt::Priority::P3);
    let adc = Saadc::new(p.SAADC, Irqs, adc_config, [channel_config]);
    adc.calibrate().await;

    defmt::debug!("SAADC initialized.");

    // == Initialize Battery ==
    let charge_indicator = Input::new(p.P0_12, Pull::None);
    let battery = Battery::init(adc, charge_indicator);
    unwrap!(_spawner.spawn(update_battery_status(battery)));

    defmt::debug!("Battery initialized.");

    // == Initialize Button ==
    let button_pin = Input::new(p.P0_13, Pull::None);
    let enable_pin = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    let button = Button::init(button_pin, enable_pin);
    unwrap!(_spawner.spawn(poll_button(button)));

    defmt::debug!("Button initialized.");

    // // == Initialize Heart Rate Monitor ==
    let hrm = HeartRateMonitor::init(I2cDevice::new(i2c_bus));
    unwrap!(_spawner.spawn(update_heart_rate(hrm)));

    defmt::debug!("Heart rate monitor initialized.");

    // == Initialize LCD ==
    let spim = spim::Spim::new(p.SPI2, Irqs, p.P0_02, p.P0_04, p.P0_03, spim_config);
    let cs_pin = Output::new(p.P0_25, Level::Low, OutputDrive::Standard);
    let dc_pin = Output::new(p.P0_18, Level::Low, OutputDrive::Standard);
    let reset_pin = Output::new(p.P0_26, Level::Low, OutputDrive::Standard);
    let backlight = BacklightPins::init(
        Output::new(p.P0_14, Level::High, OutputDrive::Standard),
        Output::new(p.P0_22, Level::High, OutputDrive::Standard),
        Output::new(p.P0_23, Level::High, OutputDrive::Standard),
    );
    let display = Display::init(spim, cs_pin, dc_pin, reset_pin, backlight);
    unwrap!(_spawner.spawn(update_lcd(display)));

    defmt::debug!("Display initialized.");

    // == Initialize Touch Controller ==
    let interrupt_pin = Input::new(p.P0_28, Pull::Up);
    let reset_pin = Output::new(p.P0_10, Level::High, OutputDrive::Standard);
    let touch = TouchController::init(I2cDevice::new(i2c_bus), interrupt_pin, reset_pin);
    unwrap!(_spawner.spawn(poll_touch(touch)));

    defmt::debug!("Touch controller initialized.");

    // == Initialize Vibrator ==
    let enable_pin = Output::new(p.P0_16, Level::High, OutputDrive::Standard);
    let vibrator = Vibrator::init(enable_pin);
    unwrap!(_spawner.spawn(notify(vibrator)));

    defmt::debug!("Vibrator initialized.");

    defmt::info!("Initialization finished");
}
