//! Battery status check
//! 
//! Implementation based upon https://github.com/dbrgn/pinetime-rtic/blob/master/pinetime-rtic/src/battery.rs
//! and https://wiki.pine64.org/wiki/PineTime.

use nrf52832_hal::{
    gpio::{Floating, Input, p0, Pin},
    prelude::*,
    saadc::Saadc,
};

#[allow(unused)]
pub struct BatteryStatus {
    /// Pin P0.12: High = battery, Low = charging
    pin_charge_indication: Pin<Input<Floating>>,

    /// Pin P0.31: Voltage level
    pin_voltage: p0::P0_31<Input<Floating>>,

    /// Saadc instance
    saadc: Saadc,

    /// Charging state
    charging: bool,

    /// Battery voltage in 0.1 volt steps
    voltage: u8,
}

impl BatteryStatus {
    /// Initialize battery status
    pub fn init(pin_charge_indication: Pin<Input<Floating>>, mut pin_voltage: p0::P0_31<Input<Floating>>, mut saadc: Saadc) -> Result<Self, Error> {
        // Get initial charging state
        let charging = pin_charge_indication.is_low().unwrap();

        // Get initial voltage
        let reading = match saadc.read(&mut pin_voltage) {
            Ok(res) => Ok(res),
            Err(_) => Err(Error::SaadcError),
        }?;
        let voltage = Self::convert_adc_measurement(&reading)?;
        // let voltage = Self::convert_adc_measurement(&saadc.read(&mut pin_voltage).unwrap()).unwrap_or(0);

        Ok(Self { pin_charge_indication, pin_voltage, saadc, charging, voltage })
    }

    /// Convert an ADC measurement into a battery voltage in volts.
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
        match self.voltage {
            42 => 100,
            41 => 90,
            40 => 80,
            39 => 70,
            38 => 60,
            37 => 50,
            36 => 30,
            35 => 10,
            _ => 0,
        }
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
    pub fn update(&mut self) -> Result<bool, Error> {
        let mut changed = false;

        // Check charging status
        let charging = self.pin_charge_indication.is_low().unwrap();
        if charging != self.charging {
            self.charging = charging;
            changed = true;
        }

        // Check voltage
        let reading = match self.saadc.read(&mut self.pin_voltage) {
            Ok(res) => Ok(res),
            Err(_) => Err(Error::SaadcError),
        }?;
        let voltage = Self::convert_adc_measurement(&reading)?;

        if voltage != self.voltage {
            self.voltage = voltage;
            changed = true;
        }

        Ok(changed)
    }
}

#[derive(Debug)]
pub enum Error {
    InvalidMeasurement,
    SaadcError,
    // SaadcError(<Saadc as _embedded_hal_adc_OneShot<Saadc, i16, p0::P0_31<Input<Floating>>>>::Error),
}