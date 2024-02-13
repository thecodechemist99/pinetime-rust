//! Accelerometer module for PineTime

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_nrf::twim::{self, Twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;

use bma423::{self, Bma423, FullPower};

#[allow(unused)]
/// Accelerometer configuration
struct AccelerometerConfig<'a, TWI>
where
    TWI: twim::Instance,
{
    /// Accelerometer sensor
    sensor: Bma423<I2cDevice<'a, NoopRawMutex, Twim<'a, TWI>>, FullPower>,
}

#[allow(unused)]
pub struct Accelerometer<TWI>
where
    TWI: twim::Instance,
{
    /// Accelerometer configuration
    config: AccelerometerConfig<'static, TWI>,
}

impl<TWI> Accelerometer<TWI>
where
    TWI: twim::Instance,
{
    /// Configure accelerometer settings on boot
    pub fn init(interface: I2cDevice<'static, NoopRawMutex, Twim<'static, TWI>>) -> Self {
        let config = bma423::Config::default();
        let sensor = Bma423::new(interface, config);
        let mut delay = Delay;

        Self {
            config: AccelerometerConfig {
                sensor: sensor.init(&mut delay).unwrap(),
            },
        }
    }
}
