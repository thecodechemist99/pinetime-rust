#![no_std]
#![no_main]

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
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};

use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<SPI2>;
});

use backlight::Backlight;
use battery::BatteryStatus;
use display::Display;

struct UserInterface {
    time: i64,
    battery: u8,
    charging: bool,
}

// Include current local time timestamp at compile time
include!(concat!(env!("OUT_DIR"), "/utc.rs"));

static INCREASE_BRIGHTNESS: Signal<ThreadModeRawMutex, bool> = Signal::new();
static UI: Mutex<ThreadModeRawMutex, UserInterface> = Mutex::new(UserInterface {
    time: UTC_TIME,
    battery: 0,
    charging: false,
});

#[embassy_executor::task(pool_size = 1)]
async fn update_lcd(mut display: Display) {
    loop {
        {
            let ui = UI.lock().await;
            display.update(ui.time, ui.battery, ui.charging);
        }

        // Initialize touch controller
        let touch = TouchController::new(
            i2c,
            gpio.p0_28.into_pullup_input().degrade(), // Touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
            Some(gpio.p0_10.into_push_pull_output(Level::High).degrade()), // Touchpad reset pin: P0.10/NFC2 (TP_RESET)
        )
        .unwrap()
        .init(&mut delay);

        // Schedule tasks immediately
        poll_button::spawn().unwrap();
        poll_touch::spawn().unwrap();
        update_battery_status::spawn().unwrap();
        notify::spawn().unwrap();

        // Schedule time measurement task to start exactly 1s after boot
        let since_boot = InstantU32::duration_since_epoch(monotonics::now());
        update_time::spawn_at(monotonics::now() + (1.secs() - since_boot)).unwrap();

        (
            Shared {
                bluetooth,
                // ble_ll,
                display,
                draw_ui: true,
                // radio,
            },
            Local {
                backlight,
                battery,
                // ble_r,
                button,
                button_debouncer: debounce_2(false),
                touch,
                vibration,
            },
            init::Monotonics(mono),
        )
    }

    /// Hook up the RADIO interrupt to the Rubble BLE stack.
    #[task(binds = RADIO, shared = [bluetooth], priority = 4)]
    fn radio(mut c: radio::Context) {
        let mut queued_work = false;

        c.shared.bluetooth.lock(|ble| {
            queued_work = ble.handle_radio_interrupt();
        });

        if queued_work {
            // If there's any lower-priority work to be done, ensure that happens.
            // If we fail to spawn the task, it's already scheduled.
            ble_worker::spawn().ok();
        }
    }

    /// Hook up the TIMER1 interrupt to the Rubble BLE stack.
    #[task(binds = TIMER1, shared = [bluetooth], priority = 4)]
    fn ble_timer(mut c: ble_timer::Context) {
        let mut queued_work = false;

        c.shared.bluetooth.lock(|ble| {
            queued_work = ble.handle_timer_interrupt();
        });

        if queued_work {
            // If there's any lower-priority work to be done, ensure that happens.
            // If we fail to spawn the task, it's already scheduled.
            ble_worker::spawn().ok();
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
            unwrap!(Spawner::for_current_executor()
                .await
                .spawn(button_pressed()));
        }

        // pin.wait_for_rising_edge().await;
        // defmt::info!("Button pressed!");
        // pin.wait_for_falling_edge().await;
        // defmt::info!("Button released!");

        // Re-schedule the timer interrupt in 2ms
        poll_touch::spawn_after(2.millis()).unwrap();
    }

    /// Called when a touch event is detected.
    #[task(priority = 2)]
    fn touch_event_detected(_c: touch_event_detected::Context, gesture: TouchGesture) {
        match gesture {
            TouchGesture::SingleClick => {
                defmt::info!("Touch event detected: single click");
            }
            TouchGesture::DoubleClick => {
                defmt::info!("Touch event detected: double click");
                // update_backlight::spawn().unwrap();
            }
            _ => {
                defmt::info!("Touch event detected: other touch event");
            }
        };
    }

    /// Enable or disable UI.
    #[task(shared = [display, draw_ui], priority = 3)]
    fn enable_ui(mut c: enable_ui::Context, enable: bool) {
        if enable {
            *c.shared.draw_ui = true;
        } else {
            *c.shared.draw_ui = false;
            c.shared.display.lock(|d| {
                d.clear();
            });
        }
    }

    /// Fetch the battery status from the hardware. Update the text if
    /// ui is enabled and something changed.
    #[task(local = [battery], shared = [draw_ui], priority = 3)]
    fn update_battery_status(c: update_battery_status::Context) {
        let changed = c.local.battery.update().unwrap();
        if changed && *c.shared.draw_ui {
            show_battery_status::spawn(c.local.battery.percent(), c.local.battery.is_charging())
                .unwrap();
        }

        // Re-schedule the timer interrupt in 1s
        update_battery_status::spawn_after(1.secs()).unwrap();
    }

    /// Show the battery status on the LCD.
    #[task(shared = [display], priority = 2)]
    fn show_battery_status(mut c: show_battery_status::Context, percentage: u8, charging: bool) {
        defmt::info!(
            "Battery status: {} ({})",
            percentage,
            if charging { "charging" } else { "discharging" },
        );

        // Update UI
        c.shared.display.lock(|d| {
            d.update_battery_status(percentage, charging);
        });
    }

    /// Get the current time Instant. Update the text if
    /// ui is enabled and something changed.
    #[task(shared = [draw_ui], priority = 3)]
    fn update_time(c: update_time::Context) {
        let now = monotonics::now();

        if *c.shared.draw_ui {
            let utc = NaiveDateTime::from_timestamp_opt(
                UTC_EPOCH + InstantU32::duration_since_epoch(now).to_secs() as i64,
                0,
            )
            .unwrap();
            show_time::spawn(utc).unwrap();
        }

        // Re-schedule the timer interrupt
        update_time::spawn_at(now + 1.secs()).unwrap();
    }

    /// Show the current time on the LCD.
    #[task(shared = [display], priority = 2)]
    fn show_time(mut c: show_time::Context, utc: NaiveDateTime) {
        // defmt::debug!("UTC time: {}:{}:{}", utc.hour(), utc.minute(), utc.second());

        // Update UI
        c.shared.display.lock(|display| {
            display.update_time(utc, TIMEZONE);
        });
    }

    defmt::info!("Initialization finished");

    unwrap!(_spawner.spawn(poll_button(button)));
    unwrap!(_spawner.spawn(update_battery_status(battery)));
    unwrap!(_spawner.spawn(update_brightness(backlight)));
    unwrap!(_spawner.spawn(update_lcd(display)));
}
