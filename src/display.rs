//! Display control

use embassy_nrf::{
    gpio::{Output, AnyPin},
    peripherals::SPI2,
    spim::Spim,
};
use embassy_time::{Delay, Instant};

use chrono::{Timelike, NaiveDateTime};
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    prelude::*,
    pixelcolor::Rgb565,
    mono_font::{
        iso_8859_1::{
            FONT_10X20,
        },
        MonoTextStyle
    },
    text::Text,
};
use mipidsi::{
    Builder,
    models::ST7789,
    Orientation,
};

#[allow(unused)]
pub struct Display {
    /// The current brightness level (value between 0 and 7).
    lcd: mipidsi::Display<SPIInterface<Spim<'static, SPI2>, Output<'static, AnyPin>, Output<'static, AnyPin>>, ST7789, Output<'static, AnyPin>>,
}

impl Display {
    /// Initialize the backlight with the specified level (0–7).
    #[allow(unused)]
    pub fn init(
        spim: Spim<'static, SPI2>,
        cs: Output<'static, AnyPin>,
        dc: Output<'static, AnyPin>,
        rst: Output<'static, AnyPin>,
    ) -> Self {
        let lcd = Builder::st7789(SPIInterface::new(spim, dc, cs))
            .with_display_size(240, 240)
            .with_orientation(Orientation::Portrait(false))
            .init(&mut Delay, Some(rst)).unwrap();

       let mut display = Self {
            lcd,
        };
        display.clear();
        display
    }

    /// Clear the display.
    pub fn clear(&mut self) {
        self.lcd.clear(Rgb565::WHITE).unwrap();
    }

    /// Update the display contents.
    pub fn update(&mut self, init_time: i64, battery: u8, charging: bool) {
        self.clear();

        // Set text style
        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);

        // Draw time
        let secs_since_boot = Instant::as_secs(&Instant::now());
        let now = NaiveDateTime::from_timestamp_opt(init_time + secs_since_boot as i64, 0).unwrap();

        let mut buf = [0u8; 8];
        let time_str = format_no_std::show(&mut buf, format_args!("{:02}:{:02}:{:02}", now.hour() + 1, now.minute(), now.second())).unwrap();
        Text::new(time_str, Point::new(80, 70), text_style).draw(&mut self.lcd).unwrap();

        // Draw battery status
        let mut buf = [0u8; 16];
        let battery_str = format_no_std::show(&mut buf, format_args!("Battery: {}mV", battery as u16 * 100_u16)).unwrap();
        Text::new(battery_str, Point::new(20, 200), text_style).draw(&mut self.lcd).unwrap();

        if charging {
            Text::new("Charging...", Point::new(20, 220), text_style).draw(&mut self.lcd).unwrap();
        }
    }
}