#![no_std]
#![no_main]

mod backlight;
mod battery;
mod bluetooth;
mod delay;
mod display;
mod monotonic_nrf52;

#[rtic::app(device = nrf52832_hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4, SWI5_EGU5])]
mod app {
    // Panic handler and debugging
    use defmt_rtt as _;
    use panic_rtt_target as _;

    /// Terminates the application and makes `probe-run` exit with exit-code = 0
    pub fn exit() -> ! {
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // Device
    use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
    use nrf52832_hal::{
        self as hal,
        gpio::{Floating, Input, Level, Pin},
        pac,
        prelude::InputPin,
        saadc::{Saadc, SaadcConfig, Resolution},
    };
    use rtic::Monotonic;

    // Crate
    use crate::backlight::Backlight;
    use crate::battery::BatteryStatus;
    use crate::bluetooth::BLE;
    use crate::delay::TimerDelay;
    use crate::display::Display;
    use crate::monotonic_nrf52::MonoTimer;

    // Others
    use chrono::{NaiveDateTime, Timelike};
    use fugit::{ExtU32, TimerInstantU32 as InstantU32};

    // Include current utc time timestamp at compile time
    include!(concat!(env!("OUT_DIR"), "/utc.rs"));

    #[shared]
    struct Shared {
        backlight: Backlight,
        #[lock_free]
        battery: BatteryStatus,
        display: Display,
    }

    #[local]
    struct Local {
        button: Pin<Input<Floating>>,
        button_debouncer: Debouncer<u8, Repeat6>,
    }

    #[monotonic(binds = TIMER1, default = true)]
    type Mono = MonoTimer<hal::pac::TIMER1>;

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Destructure device peripherals
        let pac::Peripherals {
            CLOCK,
            FICR,
            P0,
            RADIO,
            SAADC,
            SPIM1,
            TIMER0,
            TIMER1,
            TIMER2,
            ..
        } = c.device;

        // Initialize Clocks
        // On reset, the high frequency clock is already used, but we also need to
        // switch to the external HF oscillator. This is needed for Bluetooth to work.
        let _clocks = hal::clocks::Clocks::new(CLOCK).enable_ext_hfosc();

        // Initialize Delay on TIMER0
        let delay = TimerDelay::new(TIMER0);

        // Initialize monotonic timer on TIMER0 (for RTIC)
        let mono = MonoTimer::new(TIMER1);

        // Initialize BLE
        let ble = BLE::init(&FICR, RADIO, TIMER2);

        // Initialize GPIO peripheral
        let gpio = hal::gpio::p0::Parts::new(P0);

        // Initialize SAADC
        let mut saadc_config = SaadcConfig::default();
        // Set resolution to 12bit, necessary for correct battery status calculation
        saadc_config.resolution = Resolution::_12BIT;
        let saadc = Saadc::new(SAADC, saadc_config);

        // Initialize Backlight
        let mut backlight = Backlight::init(
            gpio.p0_14.into_push_pull_output(Level::High).degrade(),
            gpio.p0_22.into_push_pull_output(Level::High).degrade(),
            gpio.p0_23.into_push_pull_output(Level::High).degrade(),
            0,
        );

        // Initalize Battery
        let battery = BatteryStatus::init(
            gpio.p0_12.into_floating_input().degrade(),
            gpio.p0_31.into_floating_input(),
            saadc,
        ).unwrap();

        // Initialize Button
        let _ = gpio.p0_15.into_push_pull_output(Level::High);
        let button = gpio.p0_13.into_floating_input().degrade();

        // Initialize SPI
        let spi_pins = hal::spim::Pins {
            sck: Some(gpio.p0_02.into_push_pull_output(Level::Low).degrade()),
            miso: Some(gpio.p0_04.into_floating_input().degrade()),
            mosi: Some(gpio.p0_03.into_push_pull_output(Level::Low).degrade()),
        };

        let spi = hal::Spim::new(
            SPIM1,
            spi_pins,
            // Use SPI at 8MHz (the fastest clock available on the nRF52832),
            // otherwise refreshing will be super slow.
            hal::spim::Frequency::M8,
            // SPI must be used in mode 3. Mode 0 (the default) won't work.
            hal::spim::MODE_3,
            0,
        );

        // Initialize LCD
        let display = Display::init(
            spi,
            gpio.p0_25.into_push_pull_output(Level::Low).degrade(),
            gpio.p0_18.into_push_pull_output(Level::Low).degrade(),
            gpio.p0_26.into_push_pull_output(Level::Low).degrade(),
            delay,
        );
        #[cfg(debug_assertions)]
        {
            backlight.set(2).unwrap();
        }

        // Schedule tasks immediately
        poll_button::spawn().unwrap();
        update_battery_status::spawn().unwrap();

        // Schedule time measurement task to start exactly 1s after boot
        let since_boot = InstantU32::duration_since_epoch(monotonics::now());
        update_time::spawn_at(monotonics::now() + (1.secs() - since_boot)).unwrap();

        (
            Shared {
                backlight,
                battery,
                display,
            },
            Local {
                button,
                button_debouncer: debounce_6(false),
            },
            init::Monotonics(mono),
        )
    }

    /// Polls the button state every 2ms
    #[task(local = [button, button_debouncer], priority = 2)]
    fn poll_button(c: poll_button::Context) {
        let poll_button::LocalResources {
            button,
            button_debouncer,
        } = c.local;

        // Poll button
        let pressed = button.is_high().unwrap();
        let edge = button_debouncer.update(pressed);

        if edge == Some(Edge::Rising) {
            defmt::info!("Button pressed");
            button_pressed::spawn().unwrap();
        }

        // Re-schedule the timer interrupt in 2ms
        poll_button::spawn_after(2.millis()).unwrap();
    }

    /// Called when button is pressed without bouncing for 12 (6 * 2) ms.
    #[task(shared = [backlight], priority = 2)]
    fn button_pressed(mut c: button_pressed::Context) {
        c.shared.backlight.lock(|bl| {
            if bl.get_brightness() < 7 {
                bl.brighter().unwrap();
            } else {
                bl.off();
            }
        });
    }

    /// Fetch the battery status from the hardware. Update the text if
    /// something changed.
    #[task(shared = [backlight, battery], priority = 3)]
    fn update_battery_status(mut c: update_battery_status::Context) {
        let mut inhibit = false;
        c.shared.backlight.lock(|bl| {
            if bl.get_brightness() == 0 {
                inhibit = true;
            }
        });

        if c.shared.battery.update().unwrap() && !inhibit {
            show_battery_status::spawn().unwrap();
        }

        // Re-schedule the timer interrupt in 1s
        update_battery_status::spawn_after(1.secs()).unwrap();
    }

    /// Show the battery status on the LCD.
    #[task(shared = [battery, display], priority = 3)]
    fn show_battery_status(mut c: show_battery_status::Context) {
        let voltage = c.shared.battery.voltage();
        let charging = c.shared.battery.is_charging();

        defmt::info!(
            "Battery status: {} ({})",
            voltage,
            if charging { "charging" } else { "discharging" },
        );

        // Show battery status in top right corner
        c.shared.display.lock(|display| {
            Display::update_battery_status(display, voltage, charging);
        });
    }

    /// Show the current time on the LCD.
    #[task(shared = [backlight, display], priority = 1)]
    fn update_time(mut c: update_time::Context) {
        let now = monotonics::now();

        // let mut inhibit = false;
        // c.shared.backlight.lock(|bl| {
        //     if bl.get_brightness() == 0 {
        //         inhibit = true;
        //     }
        // });

        // if !inhibit {
            let time = NaiveDateTime::from_timestamp_opt(
                UTC_EPOCH + InstantU32::duration_since_epoch(now).to_secs() as i64,
                0).unwrap();
    
            defmt::debug!("Time: {}:{}:{}", time.hour() + 1, time.minute(), time.second());
    
            // Show battery status in top right corner
            c.shared.display.lock(|display| {
                Display::update_time(display, time);
            });
        // }

        // Re-schedule the timer interrupt
        update_time::spawn_at(now + 1.secs()).unwrap();
    }
}