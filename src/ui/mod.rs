//! UI definitions module
//! Based on: https://github.com/lupyuen/pinetime-watchface/blob/master/src/lib.rs

use chrono::NaiveDateTime;

mod default_watchface;

pub trait WatchFace<'a> {
    /// Create new watchface
    fn new() -> Self;

    /// Update watchface with state
    fn update(&'a mut self, state: WatchFaceState);
}

/// State for the watch face
pub struct WatchFaceState {
    pub time: NaiveDateTime,
    // pub ble: ,
    pub percent: u8,
    pub charging: bool,
}
