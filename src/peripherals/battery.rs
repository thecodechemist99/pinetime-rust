//! Battery status check
//!
//! Implementation based upon https://github.com/dbrgn/pinetime-rtic/blob/master/pinetime-rtic/src/battery.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use embassy_nrf::{gpio::Input, peripherals::P0_12, saadc::Saadc};

/// Battery configuration
struct BatteryConfig<'a> {
    /// ADC instance for battery voltage measurement
    adc: Saadc<'a, 1>,
    /// Charge indication pin:
    /// high = battery, low = charging
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

// ====================================
// pub struct BatteryInfo {
//     /// Charging state
//     pub charging: bool,

//     /// Battery percentage
//     pub percent: u8,
// }

// #[allow(unused)]
// pub struct BatteryStatus<'a> {
//     /// Pin P0.12: High = battery, Low = charging
//     pin_charge_indication: Input<'a, P0_12>,

//     /// Saadc instance
//     saadc: Saadc<'a, 1>,

//     /// Charging state
//     charging: bool,

//     /// Battery voltage in 0.1 volt steps
//     voltage: u8,
// }

// impl<'a> BatteryStatus<'_> {
//     /// Initialize battery status
//     pub async fn init(
//         pin_charge_indication: Input<'static, P0_12>,
//         mut saadc: Saadc<'static, 1>,
//     ) -> Result<Self, Error> {
//         // Get initial charging state
//         let charging = pin_charge_indication.is_low();

//         // Get initial voltage
//         let mut buf = [0; 1];
//         saadc.sample(&mut buf).await;
//         let voltage = Self::convert_adc_measurement(&buf[0])?;

//         Ok(Self {
//             pin_charge_indication,
//             saadc,
//             charging,
//             voltage,
//         })
//     }

//     // Return current state of the battery.
//     ///
//     /// This returns the stored value. To fetch current data, call `update()` first.
//     pub fn info(&self) -> BatteryInfo {
//         BatteryInfo {
//             charging: self.is_charging(),
//             percent: self.percent(),
//         }
//     }

//     /// Return whether the watch is currently charging.
//     ///
//     /// This returns the stored value. To fetch current data, call `update()` first.
//     #[allow(unused)]
//     pub fn is_charging(&self) -> bool {
//         self.charging
//     }

//     /// Return the current battery charge in percent (0–100).
//     ///
//     /// This returns the stored value. To fetch current data, call `update()` first.
//     #[allow(unused)]
//     pub fn percent(&self) -> u8 {
//         match self.voltage {
//             42 => 100,
//             41 => 90,
//             40 => 80,
//             39 => 70,
//             38 => 60,
//             37 => 50,
//             36 => 30,
//             35 => 10,
//             _ => 0,
//         }
//     }

//     /// Update the current battery status by reading information from the
//     /// hardware. Return whether or not the values changed.
//     #[allow(unused)]
//     pub fn update(&mut self) -> Result<bool, Error> {
//         let mut changed = false;

//         // Check charging status
//         let charging = self.pin_charge_indication.is_low();
//         if charging != self.charging {
//             self.charging = charging;
//             changed = true;
//         }

//         // Check voltage
//         let mut buf = [0; 1];
//         self.saadc.sample(&mut buf);
//         let voltage = Self::convert_adc_measurement(&buf[0])?;

//         if voltage != self.voltage {
//             self.voltage = voltage;
//             changed = true;
//         }

//         Ok(changed)
//     }

//     /// Return the current battery voltage in 0.1 volts.
//     ///
//     /// This returns the stored value. To fetch current data, call `update()` first.
//     #[allow(unused)]
//     pub fn voltage(&self) -> u8 {
//         self.voltage
//     }

//     /// Convert an ADC measurement into a battery voltage in volts.
//     fn convert_adc_measurement(raw_measurement: &i16) -> Result<u8, Error> {
//         match raw_measurement {
//             0..=4095 => {
//                 let adc_val: u32 = (*raw_measurement as u16).into(); // keep as 32 bit for multiplication
//                 let battery_voltage: u32 = (adc_val * 2000) / 1241; // multiply the ADC value by 2 * 1000 for mV and divide by (2 ^ 12 / 3.3V reference)
//                 Ok((battery_voltage / 100) as u8)
//             }
//             _ => Err(Error::InvalidMeasurement),
//         }
//     }
// }

// #[allow(unused)]
// #[derive(Debug)]
// pub enum Error {
//     InvalidMeasurement,
//     SaadcError,
// }
