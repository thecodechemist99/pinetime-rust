mod monotonic_nrf52;

pub(crate) mod bluetooth;
pub(crate) mod delay;
pub(crate) mod i2c;
pub(crate) mod monotonics {
    pub use super::monotonic_nrf52::MonoTimer;
}
