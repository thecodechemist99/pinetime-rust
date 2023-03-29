mod monotonic_nrf52;

pub(crate) mod delay;
pub(crate) mod monotonics {
    pub use super::monotonic_nrf52::MonoTimer;
}
