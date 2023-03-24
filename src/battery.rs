//! Battery status check
//! 
//! Implementation based upon https://github.com/dbrgn/pinetime-rtic/blob/master/pinetime-rtic/src/battery.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use embassy_nrf::{
    gpio::{Input, AnyPin},
    saadc::Saadc,
};

#[allow(unused)]
pub struct BatteryStatus {
    /// Pin P0.12: High = battery, Low = charging
    pin_charge_indication: Input<'static, AnyPin>,

    /// Saadc instance
    saadc: Saadc<'static, 1>,

    /// Charging state
    charging: bool,

    /// Battery voltage in 0.1 volt steps
    voltage: u8,
}

impl BatteryStatus {
    /// Initialize battery status
    #[allow(unused)]
    pub async fn init(pin_charge_indication: Input<'static, AnyPin>, mut saadc: Saadc<'static, 1>) -> Self {
        // Get initial charging state
        let charging = pin_charge_indication.is_low();

        // Get initial voltage
        let mut buf = [0; 1];
        saadc.sample(&mut buf).await;
        let voltage = Self::convert_adc_measurement(&buf[0]).unwrap_or(0);

        Self { pin_charge_indication, saadc, charging, voltage }
    }

    /// Convert an ADC measurement into a battery voltage in volts.
    #[allow(unused)]
    fn convert_adc_measurement(raw_measurement: &i16) -> Result<u8, Error> {
        match raw_measurement {
            0 ..= 4095 => {
                let adc_val: u32 = (*raw_measurement as u16).into(); // keep as 32 bit for multiplication
                let battery_voltage: u32 = (adc_val * 2000) / 1241; // multiply the ADC value by 2 * 1000 for mV and divide by (2 ^ 12 / 3.3V reference)
                Ok((battery_voltage / 100) as u8)
            },
            _ => Err(Error::InvalidMeasurement),
        }
    }

    /// Return whether the watch is currently charging.
    ///
    /// This returns the stored value. To fetch current data, call `update()` first.
    #[allow(unused)]
    pub fn is_charging(&self) -> bool {
        self.charging
    }

    /// Return the current battery charge in percent (0–100).
    ///
    /// This returns the stored value. To fetch current data, call `update()` first.
    #[allow(unused)]
    pub fn percent(&self) -> u8 {
        unimplemented!();
    }

    /// Return the current battery voltage in 0.1 volts.
    ///
    /// This returns the stored value. To fetch current data, call `update()` first.
    #[allow(unused)]
    pub fn voltage(&self) -> u8 {
        self.voltage
    }

    /// Update the current battery status by reading information from the
    /// hardware. Return whether or not the values changed.
    #[allow(unused)]
    pub async fn update(&mut self) -> bool {
        let mut changed = false;

        // Check charging status
        let charging = self.pin_charge_indication.is_low();
        if charging != self.charging {
            self.charging = charging;
            changed = true;
        }

        // Check voltage
        let mut buf = [0; 1];
        self.saadc.sample(&mut buf).await;
        let voltage = Self::convert_adc_measurement(&buf[0]).unwrap_or(0);
        if voltage != self.voltage {
            self.voltage = voltage;
            changed = true;
        }

        changed
    }
}

#[derive(Debug)]
pub enum Error {
    InvalidMeasurement,
}