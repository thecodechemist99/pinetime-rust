//! General system configuration

use embassy_nrf::{
    config::{Config, Debug, HfclkSource, LfclkSource},
    interrupt::Priority,
};

pub struct SystemConfig {}

impl SystemConfig {
    /// Create new system configuration
    pub fn new() -> Config {
        // Generate default config, required because Config is set as
        // `non_exhaustive`
        let mut config = Config::default();

        // Set high-frequency and low-frequency clock sources to external
        config.hfclk_source = HfclkSource::ExternalXtal;
        config.lfclk_source = LfclkSource::ExternalXtal;

        // Enable DC/DC regulator to massively reduce runtime current consumption
        config.dcdc.reg1 = true;

        // Configure interrupt priorities to exclude 0 (default), 1, and 4,
        // which are reserved for the nrf SoftDevice
        config.gpiote_interrupt_priority = Priority::P2;
        config.time_interrupt_priority = Priority::P2;

        // Allow debugging
        config.debug = Debug::Allowed;

        config
    }
}
