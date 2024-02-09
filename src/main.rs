#![no_std]
#![no_main]

mod peripherals;
mod system;

// Panic handler and debugging
use defmt::unwrap;

use defmt_rtt as _;
use panic_probe as _;

// Core
use core::mem;

// Device
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
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<TWISPI1>;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<SPI2>;
});

// BLE
use nrf_softdevice::{
    ble::{gatt_client, gatt_server, peripheral},
    raw, Config, Softdevice,
};

// Crate
use peripherals::{
    battery::Battery,
    button::Button,
    display::{BacklightPins, Brightness, Display},
    touch::{TouchController, TouchGesture},
    vibrator::Vibrator,
};
use system::bluetooth::{cts_get_epoch, BatteryServiceEvent, Bluetooth, Server, ServerEvent};

// Others
use chrono::{NaiveDateTime, Timelike};

// Communication channels
static BATTERY_STATUS: Signal<ThreadModeRawMutex, (u8, bool)> = Signal::new();
static CTS_TIME: Signal<ThreadModeRawMutex, NaiveDateTime> = Signal::new();
static INCREASE_BRIGHTNESS: Signal<ThreadModeRawMutex, bool> = Signal::new();
// static NOTIFY: Signal<ThreadModeRawMutex, u8> = Signal::new();
static TIME: Signal<ThreadModeRawMutex, NaiveDateTime> = Signal::new();
static TOUCH_EVENT: Signal<ThreadModeRawMutex, TouchGesture> = Signal::new();

/// BLE runner task
#[embassy_executor::task(pool_size = 1)]
async fn ble_runner(mut ble: Bluetooth) {
    loop {
        let e = ble.run_gatt().await;

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

// /// Check for notifications every 100ms
// #[embassy_executor::task(pool_size = 1)]
// async fn notify(mut vibrator: Vibrator) {
//     loop {
//         if NOTIFY.signaled() {
//             // Vibrate signaled amount of times
//             let count = NOTIFY.wait().await;
//             match count {
//                 1 => vibrator.pulse(PulseLength::SHORT, None).await,
//                 _ => vibrator.pulse(PulseLength::SHORT, Some(count)).await,
//             }
//         }

//         // Re-schedule the timer interrupt in 100ms
//         Timer::after(Duration::from_millis(100)).await;
//     }
// }

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

/// Get the current time.
#[embassy_executor::task(pool_size = 1)]
async fn update_time() {
    let mut ref_time = NaiveDateTime::UNIX_EPOCH;
    let mut instant = Instant::now();
    let mut tick = Ticker::every(Duration::from_secs(1));
    loop {
        // Update time from CTS
        if CTS_TIME.signaled() {
            ref_time = CTS_TIME.wait().await;
            instant = Instant::now();
        }

        // Calculate current time
        let now = Instant::now();
        let time = NaiveDateTime::from_timestamp_micros(
            ref_time.timestamp_micros() + now.duration_since(instant).as_micros() as i64,
        )
        .unwrap();

        // Send time to channel
        TIME.signal(time);

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

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     // 0 is Highest. Lower priority number can preempt higher priority number
//     // SoftDevice has reserved priorities 0 (default), 1, and 4
//     let mut config = embassy_nrf::config::Config::default();
//     config.gpiote_interrupt_priority = Priority::P2;
//     config.time_interrupt_priority = Priority::P2;
//     let mut p = embassy_nrf::init(config);
//     defmt::info!("Initializing");

//     // Initialize SAADC
//     let mut saadc_config = saadc::Config::default();
//     // Set resolution to 12bit, necessary for correct battery status calculation
//     saadc_config.resolution = Resolution::_12BIT;
//     // Pin P0.31: Voltage level
//     let channel_config = ChannelConfig::single_ended(&mut p.P0_31);
//     // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
//     interrupt::SAADC.set_priority(interrupt::Priority::P3);

//     let saadc = Saadc::new(p.SAADC, Irqs, saadc_config, [channel_config]);
//     saadc.calibrate().await;

//     // Initialize Backlight
//     let mut backlight = Backlight::init(
//         Output::new(p.P0_14, Level::High, OutputDrive::Standard),
//         Output::new(p.P0_22, Level::High, OutputDrive::Standard),
//         Output::new(p.P0_23, Level::High, OutputDrive::Standard),
//         0,
//     );

//     // Initalize Battery
//     let battery = Battery::init(saadc, Input::new(p.P0_12, Pull::None));

//     // Initialize Button
//     let button = Input::new(p.P0_13, Pull::None);
//     let btn_enable = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);

//     // Initialize vibration motor
//     let vibration = VibrationMotor::init(Output::new(p.P0_16, Level::High, OutputDrive::Standard));

//     // Initialize Bluetooth
//     let ble_config = generate_config().await;
//     let sd = Softdevice::enable(&ble_config);
//     let server = Server::new(sd).unwrap();

//     // Initialize I2C
//     let mut i2c_config = twim::Config::default();
//     // Use I2C at 400KHz (the fastest clock available on the nRF52832),
//     i2c_config.frequency = twim::Frequency::K400;
//     // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
//     interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(interrupt::Priority::P3);

//     let i2c = Twim::new(p.TWISPI1, Irqs, p.P0_06, p.P0_07, i2c_config);

//     // Initialize SPI
//     let mut spim_config = spim::Config::default();
//     // Use SPI at 8MHz (the fastest clock available on the nRF52832),
//     // otherwise refreshing will be super slow.
//     spim_config.frequency = spim::Frequency::M8;
//     // SPI must be used in mode 3. Mode 0 (the default) won't work.
//     spim_config.mode = spim::MODE_3;
//     // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
//     interrupt::SPIM2_SPIS2_SPI2.set_priority(interrupt::Priority::P3);

//     let spim = spim::Spim::new(p.SPI2, Irqs, p.P0_02, p.P0_04, p.P0_03, spim_config);

//     // Initialize LCD
//     let display = Display::init(
//         spim,
//         Output::new(p.P0_25, Level::Low, OutputDrive::Standard),
//         Output::new(p.P0_18, Level::Low, OutputDrive::Standard),
//         Output::new(p.P0_26, Level::Low, OutputDrive::Standard),
//     );
//     backlight.set(2).unwrap();

//     // Initialize touch controller
//     let touch = TouchController::new(
//         i2c,
//         Input::new(p.P0_28, Pull::Up), // Touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
//         Some(Output::new(p.P0_10, Level::High, OutputDrive::Standard)), // Touchpad reset pin: P0.10/NFC2 (TP_RESET)
//     )
//     .unwrap()
//     .init(&mut Delay);

//     defmt::info!("Initialization finished");

//     // Schedule tasks
//     unwrap!(_spawner.spawn(ble_runner(sd, server)));
//     unwrap!(_spawner.spawn(poll_button(btn_enable, button)));
//     unwrap!(_spawner.spawn(poll_touch(touch)));
//     unwrap!(_spawner.spawn(softdevice_task(sd)));
//     // unwrap!(_spawner.spawn(update_battery_status(battery)));
//     unwrap!(_spawner.spawn(update_brightness(backlight)));
//     unwrap!(_spawner.spawn(update_lcd(display)));
//     unwrap!(_spawner.spawn(update_time()));
//     unwrap!(_spawner.spawn(notify(vibration)));
// }

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    // Configure interrupt priorities to exclude 0 (default), 1, and 4,
    // which are reserved for the SoftDevice
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let mut p = embassy_nrf::init(config);

    defmt::info!("Initializing system ...");

    // == Initialize TWI/I2C ==
    let mut twi_config = twim::Config::default();
    // Use TWI at 400KHz (fastest clock available on the nRF52832)
    twi_config.frequency = twim::Frequency::K400;
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(interrupt::Priority::P3);
    let twi = Twim::new(p.TWISPI1, Irqs, p.P0_06, p.P0_07, twi_config);

    // Initialize SPI
    let mut spim_config = spim::Config::default();
    // Use SPI at 8MHz (fastest clock available on the nRF52832)
    spim_config.frequency = spim::Frequency::M8;
    // SPI must be used in mode 3, node 0 (the default) won't work.
    spim_config.mode = spim::MODE_3;
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SPIM2_SPIS2_SPI2.set_priority(interrupt::Priority::P3);

    // == Initialize Bluetooth ==
    let device_name = "PineTime";
    let sd_config = Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: device_name.as_bytes().as_ptr() as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };
    let sd = Softdevice::enable(&sd_config);
    let server = Server::new(sd).unwrap();
    let ble = Bluetooth::init(device_name, sd, server);
    unwrap!(_spawner.spawn(softdevice_task(sd)));

    // let ble = Bluetooth::init("PineTime");
    // let ble_config = generate_config().await;
    // let sd = Softdevice::enable(&ble_config);
    // let server = Server::new(sd).unwrap();

    defmt::info!("Initializing peripherals ...");

    // == Initalize ADC ==
    let mut adc_config = saadc::Config::default();
    // Ensure correct battery status calculation
    adc_config.resolution = Resolution::_12BIT;
    let channel_config = ChannelConfig::single_ended(&mut p.P0_31);
    // Priority levels 0 (default), 1, and 4 are reserved for SoftDevice
    interrupt::SAADC.set_priority(interrupt::Priority::P3);
    let adc = Saadc::new(p.SAADC, Irqs, adc_config, [channel_config]);
    adc.calibrate().await;

    // == Initialize Battery ==
    let charge_indicator = Input::new(p.P0_12, Pull::None);
    let battery = Battery::init(adc, charge_indicator);

    // == Initialize Button ==
    let button_pin = Input::new(p.P0_13, Pull::None);
    let enable_pin = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    let button = Button::init(button_pin, enable_pin);

    // == Initialize Vibrator ==
    let enable_pin = Output::new(p.P0_16, Level::High, OutputDrive::Standard);
    let _vibrator = Vibrator::init(enable_pin);

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

    // == Initialize Touch Controller ==
    let interrupt_pin = Input::new(p.P0_28, Pull::Up);
    let reset_pin = Output::new(p.P0_10, Level::High, OutputDrive::Standard);
    let touch = TouchController::init(twi, interrupt_pin, reset_pin);

    defmt::info!("Initialization finished");

    // Schedule tasks
    unwrap!(_spawner.spawn(ble_runner(ble)));
    // unwrap!(_spawner.spawn(softdevice_task(&mut ble)));
    unwrap!(_spawner.spawn(poll_button(button)));
    unwrap!(_spawner.spawn(poll_touch(touch)));
    unwrap!(_spawner.spawn(update_battery_status(battery)));
    unwrap!(_spawner.spawn(update_lcd(display)));
    unwrap!(_spawner.spawn(update_time()));
    // unwrap!(_spawner.spawn(notify(vibrator)));
}

// #[nrf_softdevice::gatt_server]
// pub struct Server {
//     pub bas: BatteryService,
// }
// struct BluetoothConfig {
//     /// GATT Server
//     server: Server,
//     /// Softdevice
//     sd: &'static Softdevice,
// }
