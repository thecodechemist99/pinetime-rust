#![no_std]
#![no_main]

mod peripherals;
mod system;

#[rtic::app(device = nrf52832_hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4, SWI5_EGU5])]
mod app {
    // Panic handler and debugging
    #[cfg(not(test))]
    use {defmt_rtt as _, panic_probe as _};

    /// Terminates the application and makes `probe-run` exit with exit-code = 0
    #[cfg(debug_assertions)]
    pub fn exit() -> ! {
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // Device
    use debouncr::{debounce_2, Debouncer, Edge, Repeat2};
    use hal::{
        gpio::{Floating, Input, Level, Output, Pin, PullUp, PushPull},
        pac,
        prelude::InputPin,
        saadc::{Resolution, Saadc, SaadcConfig},
    };
    use nrf52832_hal as hal;
    use peripherals::touch::TouchController;
    use rtic::Monotonic;
    use rubble::link::{queue::SimpleQueue, MIN_PDU_BUF};
    use rubble_nrf5x::{radio::PacketBuffer, timer::BleTimer};

    // Crate
    use crate::peripherals::{
        backlight::Backlight,
        battery::BatteryStatus,
        display::Display,
        touch::{TouchController, TouchGesture},
        vibration::VibrationMotor,
    };
    use crate::system::{
        bluetooth::Bluetooth, delay::Delay, i2c::I2CPeripheral, monotonics::MonoTimer,
    };

    // Others
    use chrono::NaiveDateTime;
    use fugit::{ExtU32, TimerInstantU32 as InstantU32};

    // Include current UTC epoch at compile time
    include!(concat!(env!("OUT_DIR"), "/utc.rs"));
    const TIMEZONE: i32 = 2 * 3_600;

    // PineTime has a 32 MHz HSE (HFXO) and a 32.768 kHz LSE (LFXO)
    #[allow(dead_code)]
    const HFXO_FREQ_HZ: u32 = 32_000_000u32;
    #[allow(dead_code)]
    const LFXO_FREQ_HZ: u32 = 32_768u32;

    #[shared]
    struct Shared {
        bluetooth: Bluetooth,
        display: Display,
        #[lock_free]
        draw_ui: bool,
    }

    #[local]
    struct Local {
        backlight: Backlight,
        battery: BatteryStatus,
        button: Pin<Input<Floating>>,
        button_debouncer: Debouncer<u8, Repeat2>,
        touch: TouchController<hal::Twim<pac::TWIM1>, Pin<Input<PullUp>>, Pin<Output<PushPull>>>,
        vibration: VibrationMotor,
    }

    #[monotonic(binds = TIMER0, default = true)]
    type Mono = MonoTimer<hal::pac::TIMER0>;

    #[init(
        local = [
            // BLE
            ble_tx_buf: PacketBuffer = [0; MIN_PDU_BUF],
            ble_rx_buf: PacketBuffer = [0; MIN_PDU_BUF],
            tx_queue: SimpleQueue = SimpleQueue::new(),
            rx_queue: SimpleQueue = SimpleQueue::new()
        ]
    )]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Destructure device peripherals
        let pac::Peripherals {
            CLOCK,
            FICR,
            P0,
            RADIO,
            SAADC,
            SPIM0,
            TIMER0,
            TIMER1,
            TWIM1,
            ..
        } = c.device;

        // Initialize Clocks
        // On reset, the high frequency clock is already used, but we also need to
        // switch to the external HF oscillator. This is needed for Bluetooth to work.
        let _clocks = hal::clocks::Clocks::new(CLOCK).enable_ext_hfosc();

        // Initialize Delay
        let mut delay = Delay::new(HFXO_FREQ_HZ);

        // Initialize monotonic timer on TIMER0 (for RTIC)
        let mono = MonoTimer::new(TIMER0);

        // Initialize BLE timer on TIMER1
        let ble_timer = BleTimer::init(TIMER1);

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
        )
        .unwrap();

        // Initialize Button
        let _ = gpio.p0_15.into_push_pull_output(Level::High);
        let button = gpio.p0_13.into_floating_input().degrade();

        // Initialize vibration motor
        let vibration = VibrationMotor::init(
            gpio.p0_16.into_push_pull_output(Level::High).degrade(),
            &mut delay,
        );

        // Initialize Bluetooth
        let bluetooth = Bluetooth::init(
            ble_timer,
            RADIO,
            &FICR,
            c.local.ble_tx_buf,
            c.local.ble_rx_buf,
            c.local.tx_queue,
            c.local.rx_queue,
        );

        // Initialize I2C
        let i2c_pins = hal::twim::Pins {
            scl: gpio.p0_07.into_floating_input().degrade(),
            sda: gpio.p0_06.into_floating_input().degrade(),
        };

        let i2c = hal::Twim::new(TWIM1, i2c_pins, hal::twim::Frequency::K400);

        // Initialize SPI
        let spi_pins = hal::spim::Pins {
            sck: Some(gpio.p0_02.into_push_pull_output(Level::Low).degrade()),
            miso: Some(gpio.p0_04.into_floating_input().degrade()),
            mosi: Some(gpio.p0_03.into_push_pull_output(Level::Low).degrade()),
        };

        let spi = hal::Spim::new(
            SPIM0,
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
            &mut delay,
        );
        #[cfg(debug_assertions)]
        {
            backlight.set(2).unwrap();
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

    /// Lower-priority task spawned from RADIO and TIMER1 interrupts.
    #[task(shared = [bluetooth], priority = 2)]
    fn ble_worker(mut c: ble_worker::Context) {
        // Fully drain the packet queue
        c.shared.bluetooth.lock(|ble| {
            ble.drain_packets();
        });
    }

    /// Polls the button state every 10ms
    #[task(local = [button, button_debouncer], priority = 3)]
    fn poll_button(c: poll_button::Context) {
        let pressed = c.local.button.is_high().unwrap();
        let edge = c.local.button_debouncer.update(pressed);

        if edge == Some(Edge::Rising) {
            defmt::info!("Button pressed");
            button_pressed::spawn().unwrap();
        }

        // Re-schedule the timer interrupt in 10ms
        poll_button::spawn_after(10.millis()).unwrap();
    }

    /// Called when button is pressed without bouncing for 20 (5 * 2) ms.
    #[task(priority = 2)]
    fn button_pressed(_c: button_pressed::Context) {
        update_backlight::spawn().unwrap();
    }

    /// Update backlight upon button pressed
    #[task(local = [backlight], priority = 2)]
    fn update_backlight(c: update_backlight::Context) {
        let bl = c.local.backlight;
        match bl.get_brightness() {
            0 => {
                bl.brighter().unwrap();
                enable_ui::spawn(true).unwrap();
            }
            1..=6 => bl.brighter().unwrap(),
            _ => {
                bl.off();
                enable_ui::spawn(false).unwrap();
            }
        };
    }

    /// Polls the touch interrupt pin every 2ms
    #[task(local = [touch], priority = 3)]
    fn poll_touch(c: poll_touch::Context) {
        if let Some(gesture) = c.local.touch.try_event_detected() {
            touch_event_detected::spawn(gesture).unwrap();
        }

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

    #[task(local = [vibration], priority = 2)]
    fn notify(c: notify::Context) {
        c.local.vibration.pulse_once(Some(200));
    }
}
