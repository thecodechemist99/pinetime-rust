//! Battery status check
//!
//! Implementation based upon https://github.com/dbrgn/pinetime-rtic/blob/master/pinetime-rtic/src/battery.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use embassy_nrf::{gpio::Input, peripherals::P0_12, saadc::Saadc};

/// Battery configuration
struct BatteryConfig<'a> {
    /// ADC instance for battery voltage measurement
    adc: Saadc<'a, 1>,
    /// Charge indication pin (low = charging)
    pin_charge_indication: Input<'a, P0_12>,
}

/// Battery API
pub struct Battery {
    /// Battery configuration
    config: BatteryConfig<'static>,
}

impl Battery {
    /// Configure battery settings on boot
    pub fn init(adc: Saadc<'static, 1>, charge_pin: Input<'static, P0_12>) -> Self {
        Self {
            config: BatteryConfig {
                adc,
                pin_charge_indication: charge_pin,
            },
        }
    }
    /// Carging state of the battery
    pub fn is_charging(&self) -> bool {
        self.config.pin_charge_indication.is_low()
    }
    /// Battery capacity in percent
    pub async fn get_percent(&mut self) -> u8 {
        let voltage = self.get_voltage().await;

        // Use fixed data points and linear interpolation in between
        // to estimate battery capacity.
        // No idea where I originally got these data points from, though.
        (match voltage {
            0..=3449 => 0,
            3450..=3699 => (voltage - 3450) / 5,
            3700..=4199 => 50 + (voltage - 3700) / 10,
            _ => 100,
        }) as u8
    }
    /// Battery voltage in millivolts
    async fn get_voltage(&mut self) -> u16 {
        let mut buf = [0; 1];
        self.config.adc.sample(&mut buf).await;
        // For detailed explanation of formula check https://wiki.pine64.org/wiki/PineTime
        // Use u32 during calculation to prevent overflow
        (buf[0] as u32 * 2000 / 1241) as u16
    }
}
