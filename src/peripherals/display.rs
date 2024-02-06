//! Display control module for PineTime

use embassy_nrf::{
    gpio::Output,
    peripherals::{P0_18, P0_25, P0_26},
    spim::{self, Spim},
};

use chrono::{NaiveDateTime, Timelike};
use display_interface_spi::SPIInterface;
use embassy_time::Delay;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Ellipse, PrimitiveStyle, PrimitiveStyleBuilder},
    text::Text,
};
use mipidsi::{models::ST7789, Builder, Orientation};
use profont::PROFONT_24_POINT;

use super::battery::BatteryInfo;

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;

const MARGIN: u16 = 20;

const BACKGROUND_COLOR: Rgb565 = Rgb565::new(0, 0, 0);

#[allow(unused)]
pub struct Display<'a, SPI>
where
    SPI: spim::Instance,
{
    lcd: mipidsi::Display<
        SPIInterface<Spim<'a, SPI>, Output<'a, P0_18>, Output<'a, P0_25>>,
        ST7789,
        Output<'a, P0_26>,
    >,
}

impl<'a, SPI> Display<'a, SPI>
where
    SPI: spim::Instance,
{
    /// Initialize the display
    #[allow(unused)]
    pub fn init(
        spim: Spim<'a, SPI>,
        cs: Output<'a, P0_25>,
        dc: Output<'a, P0_18>,
        rst: Output<'a, P0_26>,
        delay: &mut Delay,
    ) -> Self {
        let lcd = Builder::st7789(SPIInterface::new(spim, dc, cs))
            .with_display_size(LCD_W, LCD_H)
            .with_orientation(Orientation::Portrait(false))
            .init(delay, Some(rst))
            .unwrap();

        let mut display = Self { lcd };
        display.clear();
        display
    }

    /// Clear the display.
    pub fn clear(&mut self) {
        self.lcd.clear(BACKGROUND_COLOR).unwrap();
    }

    /// Update the battery status
    pub fn update_battery_status(&mut self, status: BatteryInfo) {
        // Choose text style
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::new(0xee, 0xdd, 0xee))
            .background_color(BACKGROUND_COLOR);

        // Show battery status in bottom left corner
        let mut buf = [0u8; 6];
        let str = format_no_std::show(
            &mut buf,
            format_args!(
                "{}%/{}",
                status.percent,
                if status.charging { "C" } else { "D" }
            ),
        )
        .unwrap();
        let text = Text::new(str, Point::zero(), text_style.build());
        let translation = Point::new(MARGIN as i32, LCD_H as i32 - MARGIN as i32);
        text.translate(translation).draw(&mut self.lcd).unwrap();
    }

    /// Update the time display
    pub fn update_time(&mut self, date_time: NaiveDateTime) {
        // Choose text style
        let text_style = MonoTextStyleBuilder::new()
            .font(&PROFONT_24_POINT)
            .text_color(Rgb565::new(0xee, 0xdd, 0xee))
            .background_color(BACKGROUND_COLOR);

        // Show time in the center
        let mut buf = [0u8; 8];
        let str = format_no_std::show(
            &mut buf,
            format_args!("{:02}:{:02}", date_time.hour(), date_time.minute()),
        )
        .unwrap();
        let text = Text::new(str, Point::zero(), text_style.build());
        let translation = Point::new(LCD_W as i32 / 2 - 40, LCD_H as i32 / 2 + 10);
        text.translate(translation).draw(&mut self.lcd).unwrap();

        // Calculate seconds dot position
        let dot_d: u32 = 14;
        let dot_pos = calc_dot_pos(dot_d as i32, date_time.second() as i32);
        let prev_dot_pos = if date_time.second() > 0 {
            calc_dot_pos(dot_d as i32, date_time.second() as i32 - 1)
        } else {
            calc_dot_pos(dot_d as i32, 59)
        };

        // Overwrite old previous seconds dot
        Ellipse::new(prev_dot_pos, Size::new(dot_d, dot_d))
            .into_styled(PrimitiveStyle::with_fill(BACKGROUND_COLOR))
            .draw(&mut self.lcd)
            .unwrap();

        // Choose seconds dot style
        let style = PrimitiveStyleBuilder::new()
            .stroke_width(0)
            .fill_color(Rgb565::new(0xee, 0xdd, 0xee))
            .build();

        // Draw seconds dot
        Ellipse::new(dot_pos, Size::new(dot_d, dot_d))
            .into_styled(style)
            .draw(&mut self.lcd)
            .unwrap();
    }
}

// Calculate position for seconds dot
fn calc_dot_pos(diameter: i32, seconds: i32) -> Point {
    let sec_h: i32 = LCD_H as i32 / 17;
    let sec_w: i32 = LCD_W as i32 / 17;

    match seconds {
        0..=7 => Point::new((8 + seconds) * sec_w, 0),
        8..=22 => Point::new(LCD_W as i32 - diameter, (seconds - 7) * sec_h),
        23..=37 => Point::new(
            LCD_W as i32 - (seconds - 21) * sec_w,
            LCD_H as i32 - diameter,
        ),
        38..=52 => Point::new(0, LCD_H as i32 - (seconds - 36) * sec_h),
        _ => Point::new((seconds - 52) * sec_w, 0),
    }
}
