//! Display control module for PineTime

use embassy_nrf::{
    gpio::Output,
    peripherals::{P0_14, P0_18, P0_22, P0_23, P0_25, P0_26},
    spim::{self, Spim},
};

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

// const LCD_W: u16 = 240;
// const LCD_H: u16 = 240;

// const MARGIN: u16 = 20;

// const BACKGROUND_COLOR: Rgb565 = Rgb565::new(0, 0, 0);

// #[allow(unused)]
// pub struct Display<'a, SPI>
// where
//     SPI: spim::Instance,
// {
//     lcd: mipidsi::Display<
//         SPIInterface<Spim<'a, SPI>, Output<'a, P0_18>, Output<'a, P0_25>>,
//         ST7789,
//         Output<'a, P0_26>,
//     >,
// }

// impl<'a, SPI> Display<'a, SPI>
// where
//     SPI: spim::Instance,
// {
//     /// Initialize the display
//     #[allow(unused)]
//     pub fn init(
//         spim: Spim<'a, SPI>,
//         cs: Output<'a, P0_25>,
//         dc: Output<'a, P0_18>,
//         rst: Output<'a, P0_26>,
//         delay: &mut Delay,
//     ) -> Self {
//         let lcd = Builder::st7789(SPIInterface::new(spim, dc, cs))
//             .with_display_size(LCD_W, LCD_H)
//             .with_orientation(Orientation::Portrait(false))
//             .init(delay, Some(rst))
//             .unwrap();

//         let mut display = Self { lcd };
//         display.clear();
//         display
//     }

// /// Update the battery status
// pub async fn update_battery_status(&mut self, mut battery: Battery) {
//     // Choose text style
//     let text_style = MonoTextStyleBuilder::new()
//         .font(&FONT_10X20)
//         .text_color(Rgb565::new(0xee, 0xdd, 0xee))
//         .background_color(BACKGROUND_COLOR);

//     // Show battery status in bottom left corner
//     let mut buf = [0u8; 6];
//     let str = format_no_std::show(
//         &mut buf,
//         format_args!(
//             "{}%/{}",
//             battery.get_percent().await,
//             if battery.is_charging() { "C" } else { "D" }
//         ),
//     )
//     .unwrap();
//     let text = Text::new(str, Point::zero(), text_style.build());
//     let translation = Point::new(MARGIN as i32, LCD_H as i32 - MARGIN as i32);
//     text.translate(translation).draw(&mut self.lcd).unwrap();
// }

// /// Update the time display
// pub fn update_time(&mut self, date_time: NaiveDateTime) {
//     // Choose text style
//     let text_style = MonoTextStyleBuilder::new()
//         .font(&PROFONT_24_POINT)
//         .text_color(Rgb565::new(0xee, 0xdd, 0xee))
//         .background_color(BACKGROUND_COLOR);

//     // Show time in the center
//     let mut buf = [0u8; 8];
//     let str = format_no_std::show(
//         &mut buf,
//         format_args!("{:02}:{:02}", date_time.hour(), date_time.minute()),
//     )
//     .unwrap();
//     let text = Text::new(str, Point::zero(), text_style.build());
//     let translation = Point::new(LCD_W as i32 / 2 - 40, LCD_H as i32 / 2 + 10);
//     text.translate(translation).draw(&mut self.lcd).unwrap();

//     // Calculate seconds dot position
//     let dot_d: u32 = 14;
//     let dot_pos = calc_dot_pos(dot_d as i32, date_time.second() as i32);
//     let prev_dot_pos = if date_time.second() > 0 {
//         calc_dot_pos(dot_d as i32, date_time.second() as i32 - 1)
//     } else {
//         calc_dot_pos(dot_d as i32, 59)
//     };

//     // Overwrite old previous seconds dot
//     Ellipse::new(prev_dot_pos, Size::new(dot_d, dot_d))
//         .into_styled(PrimitiveStyle::with_fill(BACKGROUND_COLOR))
//         .draw(&mut self.lcd)
//         .unwrap();

//     // Choose seconds dot style
//     let style = PrimitiveStyleBuilder::new()
//         .stroke_width(0)
//         .fill_color(Rgb565::new(0xee, 0xdd, 0xee))
//         .build();

//     // Draw seconds dot
//     Ellipse::new(dot_pos, Size::new(dot_d, dot_d))
//         .into_styled(style)
//         .draw(&mut self.lcd)
//         .unwrap();
// }
// }

// // Calculate position for seconds dot
// fn calc_dot_pos(diameter: i32, seconds: i32) -> Point {
//     let sec_h: i32 = LCD_H as i32 / 17;
//     let sec_w: i32 = LCD_W as i32 / 17;

//     match seconds {
//         0..=7 => Point::new((8 + seconds) * sec_w, 0),
//         8..=22 => Point::new(LCD_W as i32 - diameter, (seconds - 7) * sec_h),
//         23..=37 => Point::new(
//             LCD_W as i32 - (seconds - 21) * sec_w,
//             LCD_H as i32 - diameter,
//         ),
//         38..=52 => Point::new(0, LCD_H as i32 - (seconds - 36) * sec_h),
//         _ => Point::new((seconds - 52) * sec_w, 0),
//     }
// }

// =============================================

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;

#[allow(unused)]
#[derive(Clone, Copy)]
pub enum Brightness {
    LEVEL0 = 0,
    LEVEL1 = 1,
    LEVEL2 = 2,
    LEVEL3 = 3,
    LEVEL4 = 4,
    LEVEL5 = 5,
    LEVEL6 = 6,
    LEVEL7 = 7,
}

pub struct BacklightPins<'a> {
    low: Output<'a, P0_14>,
    mid: Output<'a, P0_22>,
    high: Output<'a, P0_23>,
}

impl BacklightPins<'_> {
    /// Configure backlight pins on boot
    pub fn init(
        low: Output<'static, P0_14>,
        mid: Output<'static, P0_22>,
        high: Output<'static, P0_23>,
    ) -> Self {
        Self { low, mid, high }
    }
}

#[allow(unused)]
struct DisplayConfig<'a, SPI>
where
    SPI: spim::Instance,
{
    /// Display instance
    display: mipidsi::Display<
        SPIInterface<Spim<'a, SPI>, Output<'a, P0_18>, Output<'a, P0_25>>,
        ST7789,
        Output<'a, P0_26>,
    >,
    // Backlight pins
    pins_backlight: BacklightPins<'a>,
}

#[allow(unused)]
pub struct Display<SPI>
where
    SPI: spim::Instance,
{
    /// Display configuration
    config: DisplayConfig<'static, SPI>,
    /// Backlight brightness
    brightness: Brightness,
}

impl<SPI> Display<SPI>
where
    SPI: spim::Instance,
{
    /// Configure display settings on boot
    pub fn init(
        spim: Spim<'static, SPI>,
        cs_pin: Output<'static, P0_25>,
        dc_pin: Output<'static, P0_18>,
        rst_pin: Output<'static, P0_26>,
        backlight: BacklightPins<'static>,
    ) -> Self {
        Self {
            config: DisplayConfig {
                display: Builder::st7789(SPIInterface::new(spim, dc_pin, cs_pin))
                    .with_display_size(LCD_W, LCD_H)
                    .with_orientation(Orientation::Portrait(false))
                    .init(&mut Delay, Some(rst_pin))
                    .unwrap(),
                pins_backlight: BacklightPins {
                    low: backlight.low,
                    mid: backlight.mid,
                    high: backlight.high,
                },
            },
            brightness: Brightness::LEVEL0,
        }
    }
    #[allow(unused)]
    /// Clear the display
    pub fn clear(&mut self, color: Rgb565) -> Result<(), mipidsi::Error> {
        self.config.display.clear(color)
    }

    /// Brightness of the display backlight
    pub fn get_brightness(&self) -> Brightness {
        self.brightness
    }
    /// Set the backlight brightness
    pub fn set_brightness(&mut self, level: Brightness) {
        self.brightness = level;
    }
}
