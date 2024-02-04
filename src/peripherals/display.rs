//! Display control module for PineTime

use nrf52832_hal::{
    gpio::{Output, Pin, PushPull},
    pac::SPIM0,
    spim::Spim,
};

use chrono::{FixedOffset, NaiveDateTime, TimeZone, Timelike};
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Ellipse, PrimitiveStyle, PrimitiveStyleBuilder},
    text::Text,
};
use embedded_hal::blocking::delay::DelayUs;
use mipidsi::{models::ST7789, Builder, Orientation};
use profont::PROFONT_24_POINT;

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;

const MARGIN: u16 = 20;

const BACKGROUND_COLOR: Rgb565 = Rgb565::new(0, 0, 0);

#[allow(unused)]
pub struct Display {
    lcd: mipidsi::Display<
        SPIInterface<Spim<SPIM0>, Pin<Output<PushPull>>, Pin<Output<PushPull>>>,
        ST7789,
        Pin<Output<PushPull>>,
    >,
}

impl Display {
    /// Initialize the display
    #[allow(unused)]
    pub fn init(
        spim: Spim<SPIM0>,
        cs: Pin<Output<PushPull>>,
        dc: Pin<Output<PushPull>>,
        rst: Pin<Output<PushPull>>,
        delay: &mut impl DelayUs<u32>,
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
    pub fn update_battery_status(&mut self, percent: u8, charging: bool) {
        // Choose text style
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::new(0xee, 0xdd, 0xee))
            .background_color(BACKGROUND_COLOR);

        // Show battery status in bottom left corner
        let mut buf = [0u8; 6];
        let str = format_no_std::show(
            &mut buf,
            format_args!("{}%/{}", percent, if charging { "C" } else { "D" }),
        )
        .unwrap();
        let text = Text::new(str, Point::zero(), text_style.build());
        let translation = Point::new(MARGIN as i32, LCD_H as i32 - MARGIN as i32);
        text.translate(translation).draw(&mut self.lcd).unwrap();
    }

    /// Update the time display
    pub fn update_time(&mut self, utc: NaiveDateTime, timezone: i32) {
        // Calculate local time
        let date_time = FixedOffset::east_opt(timezone)
            .unwrap()
            .from_utc_datetime(&utc);

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
