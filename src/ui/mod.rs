//! UI definitions module
//! Based on: https://github.com/lupyuen/pinetime-watchface/blob/master/src/lib.rs

use chrono::NaiveDateTime;
use embedded_canvas::CCanvas;

use crate::peripherals::display::{ColorMode, DisplayCanvas};

mod default_watchface;
pub use default_watchface::DefaultWatchface;

pub trait WatchFace {
    /// Create new watchface
    fn new() -> Self;

    /// Update watchface with state
    fn update(&'a mut self, state: WatchFaceState) -> DisplayCanvas;
}

/// State for the watch face
#[derive(Clone, Copy, Debug)]
pub struct WatchFaceState {
    pub time: NaiveDateTime,
    // pub ble: ,
    pub percent: u8,
    pub charging: bool,
}

impl Default for WatchFaceState {
    /// Set default values
    fn default() -> Self {
        Self {
            time: NaiveDateTime::UNIX_EPOCH,
            percent: 0,
            charging: false,
        }
    }
}
