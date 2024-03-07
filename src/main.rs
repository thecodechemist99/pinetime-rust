#![no_std]
#![no_main]

mod peripherals;
mod system;
mod ui;

// Panic handler and debugging
use defmt::unwrap;

use defmt_rtt as _;
use panic_probe as _;

// Core
use core::cell::RefCell;
use static_cell::StaticCell;

// System
use embassy_embedded_hal::shared_bus::blocking::{i2c::I2cDevice, spi::SpiDevice};
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    interrupt::{self, InterruptExt},
    peripherals::{SPI2, TWISPI1},
    saadc::{self, ChannelConfig, Resolution, Saadc},
    spim::{self, Spim},
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
    display::{BacklightPins, Brightness, Display, DisplayCanvas},
    heartrate::HeartRateMonitor,
    spi_flash::Flash,
    touch::{TouchController, TouchGesture},
    vibrator::Vibrator,
};
use system::{
    bluetooth::Bluetooth,
    config::SystemConfig,
    time::{TimeManager, TimeReference},
};
use ui::{DefaultWatchface, WatchFace, WatchFaceState};

// Others
use chrono::{NaiveDateTime, Timelike};

/// Shared I2C bus
static I2C_BUS: StaticCell<NoopMutex<RefCell<Twim<TWISPI1>>>> = StaticCell::new();
/// Shared SPI bus
static SPI_BUS: StaticCell<NoopMutex<RefCell<Spim<SPI2>>>> = StaticCell::new();

// Communication channels
static BATTERY_STATUS: Signal<ThreadModeRawMutex, (u8, bool)> = Signal::new();
static BAS_LEVEL: Signal<ThreadModeRawMutex, u8> = Signal::new();
static CTS_TIME: Signal<ThreadModeRawMutex, TimeReference> = Signal::new();
static HRS_MEASUREMENT: Signal<ThreadModeRawMutex, u8> = Signal::new();
static INCREASE_BRIGHTNESS: Signal<ThreadModeRawMutex, bool> = Signal::new();
static TIME: Signal<ThreadModeRawMutex, NaiveDateTime> = Signal::new();
static TOUCH_EVENT: Signal<ThreadModeRawMutex, TouchGesture> = Signal::new();
static UI_CANVAS: Signal<ThreadModeRawMutex, DisplayCanvas> = Signal::new();

/// BLE runner task
#[embassy_executor::task]
async fn ble_runner(mut ble: Bluetooth) {
    // Start softdevice
    unwrap!(Spawner::for_current_executor()
        .await
        .spawn(softdevice_task(ble.get_sd())));

    // Run GATT Server
    loop {
        // Start server
        if let Some(e) = ble
            .run_gatt_server(&CTS_TIME, &BAS_LEVEL, &HRS_MEASUREMENT)
            .await
            .err()
        {
            #[cfg(debug_assertions)]
            defmt::info!(
                "gatt_server run exited with error: {:?}",
                defmt::Debug2Format(&e)
            );
        }
    }
}

/// Fetch the battery status from the hardware.
#[embassy_executor::task]
async fn update_battery_status(mut battery: Battery) {
    let mut battery_level = 0;
    loop {
        // Update battery status
        let status = (battery.get_percent().await, battery.is_charging());
        BATTERY_STATUS.signal(status);

        // Update BLE battery service if battery level changed
        let (level, _) = status;
        if battery_level != level {
            battery_level = level;
            BAS_LEVEL.signal(battery_level);
        }

        // Re-schedule task in 10s
        Timer::after(Duration::from_secs(10)).await;
    }
}

/// Fetch the heart rate measurement from the hardware.
#[embassy_executor::task]
async fn update_heart_rate(mut hrm: HeartRateMonitor<TWISPI1>) {
    let mut tick = Ticker::every(Duration::from_millis(100));
    let mut last_bpm = 0;
    hrm.enable();
    loop {
        // Measure heart rate
        let heart_rate = hrm.start_measurement().await;

        if let Some(hr) = heart_rate {
            // Update heart rate measurement service if heart rate changed
            if last_bpm != hr {
                last_bpm = hr;
                HRS_MEASUREMENT.signal(hr);
            }
            // defmt::info!("Heart rate: {}", last_bpm);
        } else {
            #[cfg(debug_assertions)]
            defmt::debug!("Not enough data.");
        }

        // Re-schedule the timer interrupt in 100ms
        tick.next().await;
    }
}

#[embassy_executor::task]
async fn update_lcd(mut display: Display<SPI2>) {
    // TODO: Tick necessary or is wait at a higher rate just fine?
    let mut tick = Ticker::every(Duration::from_millis(100));
    loop {
        // Update backlight brightness
        if INCREASE_BRIGHTNESS.signaled() {
            if INCREASE_BRIGHTNESS.wait().await {
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
        }

        // Update UI
        if UI_CANVAS.signaled() {
            display.update(UI_CANVAS.wait().await);
        }

        // Re-schedule the timer interrupt in 1s
        tick.next().await;
    }
}

/// Update the current time.
#[embassy_executor::task]
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

#[embassy_executor::task(pool_size = 1)]
async fn update_ui() {
    let mut watch_face = DefaultWatchface::new();
    let mut watch_face_state = WatchFaceState::default();
    loop {
        if BATTERY_STATUS.signaled() {
            let (percent, charging) = BATTERY_STATUS.wait().await;
            // defmt::info!(
            //     "Battery status: {} ({})",
            //     percent,
            //     if charging { "charging" } else { "discharging" }
            // );
            watch_face_state.percent = percent;
            watch_face_state.charging = charging;
        }

        if TIME.signaled() {
            let time = TIME.wait().await;
            // defmt::info!(
            //     "Current time: {}:{}:{}",
            //     time.time().hour(),
            //     time.time().minute(),
            //     time.time().second(),
            // );
            watch_face_state.time = time;
        }

        let canvas = watch_face.update(watch_face_state);
        UI_CANVAS.signal(canvas);

        // Re-schedule the timer interrupt in 100ms
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Poll the accelerometer state every 10ms
#[embassy_executor::task]
async fn poll_accelerometer(mut accelerometer: Accelerometer<TWISPI1>) {
    loop {
        // Read from sensor
        let (x, y, z) = accelerometer.get_acceleration_vector();
        // defmt::info!("Acceleration vector: ({}, {}, {})", x, y, z);

        // Re-schedule the timer interrupt in 10ms
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Poll the button state every 10ms
#[embassy_executor::task]
async fn poll_button(mut button: Button) {
    loop {
        let pressed = button.pressed().await;
        if pressed {
            INCREASE_BRIGHTNESS.signal(pressed);
        }

        // Re-schedule the timer interrupt in 20ms
        Timer::after(Duration::from_millis(20)).await;
    }
}

/// Check for new touch event every 2ms
#[embassy_executor::task]
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
    let config = SystemConfig::new();
    let mut p = embassy_nrf::init(config);

    #[cfg(debug_assertions)]
    defmt::debug!("Initializing system ...");

    // == Initialize Timekeeping ==
    let time_manager = TimeManager::init();
    unwrap!(_spawner.spawn(update_time(time_manager)));

    #[cfg(debug_assertions)]
    defmt::debug!("Time manager initialized.");

    // == Initialize SPI ==
    let mut spim_config = spim::Config::default();
    // Use SPI at 8MHz (fastest clock available on the nRF52832)
    spim_config.frequency = spim::Frequency::M8;
    // SPI must be used in mode 3, node 0 (the default) won't work.
    spim_config.mode = spim::MODE_3;
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SPIM2_SPIS2_SPI2.set_priority(interrupt::Priority::P3);
    let spi = Spim::new(p.SPI2, Irqs, p.P0_02, p.P0_04, p.P0_03, spim_config);
    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_BUS.init(spi_bus);

    #[cfg(debug_assertions)]
    defmt::debug!("SPI initialized.");

    // == Initialize TWI/I2C ==
    let mut twi_config = twim::Config::default();
    // Use TWI at 400KHz (fastest clock available on the nRF52832)
    twi_config.frequency = twim::Frequency::K400;
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(interrupt::Priority::P3);
    let twi = Twim::new(p.TWISPI1, Irqs, p.P0_06, p.P0_07, twi_config);
    let i2c_bus = NoopMutex::new(RefCell::new(twi));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    #[cfg(debug_assertions)]
    defmt::debug!("TWI/I2C initialized.");

    // == Initialize Bluetooth Low Energy ==
    let ble = Bluetooth::init("PineTime");
    unwrap!(_spawner.spawn(ble_runner(ble)));

    #[cfg(debug_assertions)]
    defmt::debug!("BLE initialized.");

    #[cfg(debug_assertions)]
    defmt::debug!("Initializing peripherals ...");

    // == Initialize Accelerometer ==
    let accelerometer = Accelerometer::init(I2cDevice::new(i2c_bus));
    unwrap!(_spawner.spawn(poll_accelerometer(accelerometer)));

    #[cfg(debug_assertions)]
    defmt::debug!("Accelerometer initialized.");

    // == Initalize ADC ==
    let mut adc_config = saadc::Config::default();
    // Ensure correct battery status calculation
    adc_config.resolution = Resolution::_12BIT;
    let channel_config = ChannelConfig::single_ended(&mut p.P0_31);
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SAADC.set_priority(interrupt::Priority::P3);
    let adc = Saadc::new(p.SAADC, Irqs, adc_config, [channel_config]);
    adc.calibrate().await;

    #[cfg(debug_assertions)]
    defmt::debug!("SAADC initialized.");

    // == Initialize Battery ==
    let charge_indicator = Input::new(p.P0_12, Pull::Up);
    let battery = Battery::init(adc, charge_indicator);
    unwrap!(_spawner.spawn(update_battery_status(battery)));

    #[cfg(debug_assertions)]
    defmt::debug!("Battery initialized.");

    // == Initialize Button ==
    let button_in_pin = Input::new(p.P0_13, Pull::Down);
    let button_out_pin = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    let button = Button::init(button_in_pin, button_out_pin);
    unwrap!(_spawner.spawn(poll_button(button)));

    #[cfg(debug_assertions)]
    defmt::debug!("Button initialized.");

    // == Initialize Heart Rate Monitor ==
    let hrm = HeartRateMonitor::init(I2cDevice::new(i2c_bus));
    unwrap!(_spawner.spawn(update_heart_rate(hrm)));

    #[cfg(debug_assertions)]
    defmt::debug!("Heart rate monitor initialized.");

    // == Initialize LCD ==
    let cs_pin = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
    let dc_pin = Output::new(p.P0_18, Level::Low, OutputDrive::Standard);
    let reset_pin = Output::new(p.P0_26, Level::Low, OutputDrive::Standard);
    let backlight = BacklightPins::init(
        Output::new(p.P0_14, Level::High, OutputDrive::Standard),
        Output::new(p.P0_22, Level::High, OutputDrive::Standard),
        Output::new(p.P0_23, Level::High, OutputDrive::Standard),
    );
    let display = Display::init(
        SpiDevice::new(spi_bus, cs_pin),
        dc_pin,
        reset_pin,
        backlight,
    );
    unwrap!(_spawner.spawn(update_lcd(display)));

    #[cfg(debug_assertions)]
    defmt::debug!("Display initialized.");

    // == Initialize SPI Flash ==
    let cs_pin = Output::new(p.P0_05, Level::High, OutputDrive::Standard);
    let mut flash = Flash::init(SpiDevice::new(spi_bus, cs_pin));
    // Put flash in deep power down mode to reduce power consumption
    // defmt::debug!("Device ID: {}", flash.read_id());
    flash.into_power_down().await;

    #[cfg(debug_assertions)]
    defmt::debug!("SPI flash initialized.");

    // == Initialize Touch Controller ==
    let interrupt_pin = Input::new(p.P0_28, Pull::Up);
    let reset_pin = Output::new(p.P0_10, Level::High, OutputDrive::Standard);
    let touch = TouchController::init(I2cDevice::new(i2c_bus), interrupt_pin, reset_pin);
    unwrap!(_spawner.spawn(poll_touch(touch)));

    #[cfg(debug_assertions)]
    defmt::debug!("Touch controller initialized.");

    // == Initialize Vibrator ==
    let enable_pin = Output::new(p.P0_16, Level::High, OutputDrive::Standard);
    let _vibrator = Vibrator::init(enable_pin);

    #[cfg(debug_assertions)]
    defmt::debug!("Vibrator initialized.");

    #[cfg(debug_assertions)]
    defmt::info!("Initializing UI ...");

    // == Initialize UI ==
    unwrap!(_spawner.spawn(update_ui()));

    #[cfg(debug_assertions)]
    defmt::debug!("Initialization finished");
}
