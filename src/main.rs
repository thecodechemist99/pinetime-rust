#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod backlight;
mod battery;
// mod bluetooth;
mod display;

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin as _, Pull},
    peripherals::SPI2,
    saadc::{self, ChannelConfig, Saadc},
    spim,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};

use panic_probe as _;
use defmt_rtt as _;

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<SPI2>;
});

use battery::BatteryStatus;
use backlight::Backlight;
use display::Display;

struct UserInterface {
    time: i64,
    battery: u8,
    charging: bool,
}

// Include current local time timestamp at compile time
include!(concat!(env!("OUT_DIR"), "/utc.rs"));

static INCREASE_BRIGHTNESS: Signal<ThreadModeRawMutex, bool> = Signal::new();
static UI: Mutex<ThreadModeRawMutex, UserInterface> = Mutex::new(UserInterface { time: UTC_TIME, battery: 0, charging: false });

#[embassy_executor::task(pool_size = 1)]
async fn update_lcd(mut display: Display) {
    loop {
        {
            let ui = UI.lock().await;
            display.update(ui.time, ui.battery, ui.charging);
        }

        // Re-schedule the timer interrupt in 1s
        Timer::after(Duration::from_millis_floor(1000)).await;
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn update_brightness(mut backlight: Backlight) {
    loop {
        let update = INCREASE_BRIGHTNESS.wait().await;
        if update {
            if backlight.get_brightness() < 7 {
                backlight.brighter();
            } else {
                backlight.off();
            }
        }
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn button_pressed() {
    INCREASE_BRIGHTNESS.signal(true);
}

#[embassy_executor::task(pool_size = 1)]
async fn poll_button(pin: Input<'static, AnyPin>) {
    loop {
        // Poll button
        if pin.is_high() {
            defmt::info!("Button pressed!");
            unwrap!(Spawner::for_current_executor().await.spawn(button_pressed()));
        }

        // pin.wait_for_rising_edge().await;
        // defmt::info!("Button pressed!");
        // pin.wait_for_falling_edge().await;
        // defmt::info!("Button released!");

        // Re-schedule the timer interrupt in 2ms
        Timer::after(Duration::from_millis(2)).await;
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn update_battery_status(mut battery: BatteryStatus) {
    loop {
        defmt::info!("Updating battery status");

        let changed = battery.update().await;
        if changed {
            // battery status changed
            defmt::info!("Battery status changed");
            {
                let mut ui = UI.lock().await;
                ui.battery = battery.voltage();
                ui.charging = battery.is_charging();
            }
        };

        // Re-schedule the timer interrupt in 1s
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nrf::init(Default::default());
    defmt::info!("Initializing");

    // Initialize SAADC
    let saadc_config = saadc::Config::default();
    let channel_config = ChannelConfig::single_ended(&mut p.P0_31);
    let saadc = Saadc::new(p.SAADC, Irqs, saadc_config, [channel_config]);
    saadc.calibrate().await;

    // Initialize SPI
    let mut spim_config = spim::Config::default();
    spim_config.frequency = spim::Frequency::M8;
    spim_config.mode = spim::MODE_3;

    let spim = spim::Spim::new(p.SPI2, Irqs, p.P0_02, p.P0_04, p.P0_03, spim_config);

    // Initialize Backlight
    let mut backlight = Backlight::init(
        Output::new(p.P0_14.degrade(), Level::High, OutputDrive::Standard),
        Output::new(p.P0_22.degrade(), Level::High, OutputDrive::Standard),
        Output::new(p.P0_23.degrade(), Level::High, OutputDrive::Standard),
        0,
    );

    // Initialize LCD
    let display = Display::init(
        spim,
        Output::new(p.P0_25.degrade(), Level::Low, OutputDrive::Standard),
        Output::new(p.P0_18.degrade(), Level::Low, OutputDrive::Standard),
        Output::new(p.P0_26.degrade(), Level::Low, OutputDrive::Standard),
    );
    backlight.set(2);

    // Initialize Button
    let button = Input::new(p.P0_13.degrade(), Pull::None);
    let mut btn_enable = Output::new(p.P0_15, Level::High, OutputDrive::Standard);
    btn_enable.set_high();

    // Initalize Battery
    let battery = BatteryStatus::init(Input::new(p.P0_12.degrade(), Pull::None), saadc).await;
    {
        let mut ui = UI.lock().await;
        ui.battery = battery.voltage();
        ui.charging = battery.is_charging();
    }

    defmt::info!("Initialization finished");

    unwrap!(_spawner.spawn(poll_button(button)));
    unwrap!(_spawner.spawn(update_battery_status(battery)));
    unwrap!(_spawner.spawn(update_brightness(backlight)));
    unwrap!(_spawner.spawn(update_lcd(display)));
}