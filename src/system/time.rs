//! Time keeping module for PineTime

use chrono::{NaiveDate, NaiveDateTime};
use embassy_time::Instant;

#[allow(unused)]
pub struct TimeReference {
    /// Clock time
    time: NaiveDateTime,
    /// Related system time
    instant: Instant,
}

impl Default for TimeReference {
    fn default() -> Self {
        Self {
            time: NaiveDateTime::UNIX_EPOCH,
            instant: Instant::from_ticks(0),
        }
    }
}

impl TimeReference {
    #[allow(unused)]
    /// Create new time reference from NaiveDateTime
    pub fn from_datetime(time: NaiveDateTime) -> Self {
        Self {
            time,
            instant: Instant::now(),
        }
    }
    /// Create new time reference from Current Time Service data
    pub fn from_cts_bytes(bytes: &[u8]) -> Self {
        // Convert CTS bytes to NaiveDateTime
        let year = u16::from_le_bytes(bytes[..2].try_into().ok().unwrap()) as i32;
        let month = bytes[2] as u32;
        let day = bytes[3] as u32;
        let hour = bytes[4] as u32;
        let min = bytes[5] as u32;
        let sec = bytes[6] as u32;
        // let day_of_week = self.cts_data[7] as u32;
        let milli = bytes[8] as u32 * 1000 / 256; // Convert fractions_256 to milliseconds

        let time = NaiveDate::from_ymd_opt(year, month, day)
            .unwrap()
            .and_hms_milli_opt(hour, min, sec, milli)
            .unwrap();

        Self {
            time,
            instant: Instant::now(),
        }
    }
}

#[allow(unused)]
pub struct TimeManager {
    reference: TimeReference,
}

impl TimeManager {
    /// Initialize time measurement on boot
    pub fn init() -> Self {
        Self {
            reference: TimeReference::default(),
        }
    }
    /// Get current time
    pub fn get_time(&self) -> NaiveDateTime {
        let now = Instant::now();
        NaiveDateTime::from_timestamp_micros(
            self.reference.time.timestamp_micros()
                + now.duration_since(self.reference.instant).as_micros() as i64,
        )
        .unwrap()
    }
    /// Update time reference
    pub fn set_time(&mut self, reference: TimeReference) {
        self.reference = reference;
    }
}
