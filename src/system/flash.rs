//! SPI flash module

use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_nrf::{
    gpio::Output,
    peripherals::P0_05,
    spim::{self, Spim},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

#[allow(unused)]
struct FlashConfig<SPI>
where
    SPI: spim::Instance,
{
    /// Flash instance
    interface: SpiDevice<'static, NoopRawMutex, Spim<'static, SPI>, Output<'static, P0_05>>,
}

#[allow(unused)]
pub struct Flash<SPI>
where
    SPI: spim::Instance,
{
    /// Flash configuration
    config: FlashConfig<SPI>,
}

impl<SPI> Flash<SPI>
where
    SPI: spim::Instance,
{
    /// Configure flash settings on boot
    pub fn init(
        spim: SpiDevice<'static, NoopRawMutex, Spim<'static, SPI>, Output<'static, P0_05>>,
    ) -> Self {
        Self {
            config: FlashConfig { interface: spim },
        }
    }
    /// Enable deep power down state
    pub fn into_deep_power_down(&mut self) {
        // Set CS high, then low, then write 0xb9 on the SPI bus, then set cs high again
        self.config.interface.write(&[0xb9]).unwrap();
    }
}
