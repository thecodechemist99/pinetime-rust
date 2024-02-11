//! Heartrate monitor module for PineTime

mod ppg_processing;

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_nrf::twim::{self, Twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use hrs3300::Hrs3300;
use ppg_processing::PPG;

#[allow(unused)]
/// Heart rate monitor configuration
struct HeartRateMonitorConfig<'a, TWI>
where
    TWI: twim::Instance,
{
    /// Heart rate sensor
    sensor: Hrs3300<I2cDevice<'a, NoopRawMutex, Twim<'a, TWI>>>,
    /// PPG processor
    ppg: PPG,
}

#[allow(unused)]
pub struct HeartRateMonitor<TWI>
where
    TWI: twim::Instance,
{
    /// Heart rate monitor configuration
    config: HeartRateMonitorConfig<'static, TWI>,
    /// Timer index for heart rate calculation
    timer_index: u16,
}

impl<TWI> HeartRateMonitor<TWI>
where
    TWI: twim::Instance,
{
    /// Configure heart rate monitor settings on boot
    pub fn init(interface: I2cDevice<'static, NoopRawMutex, Twim<'static, TWI>>) -> Self {
        let mut sensor = Hrs3300::new(interface);
        sensor.init().unwrap();

        Self {
            config: HeartRateMonitorConfig {
                sensor,
                ppg: PPG::new(),
            },
            timer_index: 0,
        }
    }
    #[allow(unused)]
    /// Start measurement
    pub async fn start_measurement(&mut self) -> Option<u8> {
        // self.enable();
        // self.config.ppg.reset();

        // Measure
        let ambient = self.config.ppg.preprocess(
            self.config.sensor.read_hrs().unwrap(),
            self.config.sensor.read_als().unwrap(),
        );
        let mut bpm = self.config.ppg.get_heart_rate();
        // defmt::debug!("Ambient light detected: {}, BPM: {}", ambient, bpm);

        // If ambient light detected or a reset requested (bpm < 0)
        if ambient {
            // Reset all DAQ buffers
            self.config.ppg.reset();
            // Force state to None (not enough data)
            bpm = None;
        }

        // Stop measurement
        // self.disable();
        // self.config.ppg.reset();

        bpm
    }
    /// Enable heart rate monitor
    pub fn enable(&mut self) {
        self.config.sensor.enable_hrs().unwrap();
        self.config.sensor.enable_oscillator().unwrap();
    }
    /// Disable heart rate monitor
    pub fn disable(&mut self) {
        self.config.sensor.disable_hrs().unwrap();
        self.config.sensor.disable_oscillator().unwrap();
        self.config.ppg.reset();
    }
    #[allow(unused)]
    /// Read ambient light sensor
    pub fn get_ambient_light(&mut self) -> u32 {
        self.config.sensor.read_als().unwrap()
    }
}
