//! SPI flash module

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as EmbassySpiDevice;
use embassy_nrf::{
    gpio::Output,
    peripherals::P0_05,
    spim::{self, Spim},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embedded_hal::spi::SpiDevice;

#[allow(unused)]
struct FlashConfig<SPI>
where
    SPI: spim::Instance,
{
    /// Flash SPI interface
    interface: EmbassySpiDevice<'static, NoopRawMutex, Spim<'static, SPI>, Output<'static, P0_05>>,
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
        spi: EmbassySpiDevice<'static, NoopRawMutex, Spim<'static, SPI>, Output<'static, P0_05>>,
    ) -> Self {
        Self {
            config: FlashConfig { interface: spi },
        }
    }
    /// Enable deep power down state
    pub async fn into_power_down(&mut self) {
        // Set CS high, then low, then write 0xb9 on the SPI bus, then set CS high again
        // and keep it high for 0.1μs
        self.config.interface.write(&[0xb9]).unwrap();
        Timer::after_nanos(100).await;
    }
    /// Read device ID
    pub fn read_id(&mut self) -> u8 {
        let buf: &mut [u8] = &mut [0xab, 0x00, 0x00, 0x00];
        // Set CS low, shift instruction code `ABH` followed by 3 dummy bytes, then set CS high again
        self.config.interface.transfer_in_place(buf).unwrap();
        // defmt::debug!("{}", buf);
        buf[0]
    }
    /// Wake from deep power down state
    #[allow(unused)]
    pub async fn wake(&mut self) {
        // Set CS low, shift instruction code `ABH`, then set CS high again
        self.config.interface.write(&[0xb9]).unwrap();
        Timer::after_micros(20).await;
    }
    /// Wake from deep power down state and read device ID
    #[allow(unused)]
    pub async fn wake_and_read_id(&mut self) -> u8 {
        let buf: &mut [u8] = &mut [0xab, 0x00, 0x00, 0x00];
        // Set CS low, shift instruction code `ABH` followed by 3 dummy bytes, then set CS high again
        // and keep it high for 20μs
        self.config.interface.transfer_in_place(buf).unwrap();
        Timer::after_micros(20).await;
        buf[0]
    }
}
