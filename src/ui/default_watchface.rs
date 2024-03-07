//! Default watchface

use chrono::{Datelike, Timelike};
use embedded_graphics::{
    geometry::{Point, Size},
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::RgbColor,
    primitives::Rectangle,
    text::renderer::TextRenderer,
    Drawable,
};
use embedded_text::{alignment::HorizontalAlignment, style::TextBoxStyleBuilder, TextBox};

use super::{CCanvas, ColorMode, DisplayCanvas, WatchFace, WatchFaceState};

const BUF_LEN: usize = 64;

pub struct Label<'a, S>
where
    S: TextRenderer,
{
    str_buf: [u8; BUF_LEN],
    text_box: TextBox<'a, S>,
}

impl<'a, S> Label<'a, S>
where
    S: TextRenderer,
{
    /// Create new label
    fn new(text_box: TextBox<'a, S>) -> Self {
        Self {
            str_buf: [0; BUF_LEN],
            text_box,
        }
    }
}

/// Basic default watchface
pub struct DefaultWatchface<'a> {
    /// Time label
    pub time_label: Label<'a, MonoTextStyle<'a, ColorMode>>,
    /// Date label
    pub date_label: Label<'a, MonoTextStyle<'a, ColorMode>>,
    /// Power indicator label
    pub power_label: Label<'a, MonoTextStyle<'a, ColorMode>>,
}

impl<'a> WatchFace for DefaultWatchface<'a> {
    fn new() -> Self {
        let canvas: DisplayCanvas = CCanvas::new();

        Self {
            time_label: {
                // Styling
                let character_style = MonoTextStyle::new(&FONT_6X10, ColorMode::WHITE);
                let textbox_style = TextBoxStyleBuilder::new()
                    .height_mode(embedded_text::style::HeightMode::FitToText)
                    .alignment(HorizontalAlignment::Justified)
                    .build();

                // Bounding box
                let bounds = Rectangle::new(Point::zero(), Size::new(240, 0));

                // Create text box
                Label::new(TextBox::with_textbox_style(
                    "12:34",
                    bounds,
                    character_style,
                    textbox_style,
                ))
            },
            date_label: {
                // Styling
                let character_style = MonoTextStyle::new(&FONT_6X10, ColorMode::WHITE);
                let textbox_style = TextBoxStyleBuilder::new()
                    .height_mode(embedded_text::style::HeightMode::FitToText)
                    .alignment(HorizontalAlignment::Justified)
                    .build();

                // Bounding box
                let bounds = Rectangle::new(Point::zero(), Size::new(240, 0));

                // Create text box
                Label::new(TextBox::with_textbox_style(
                    "Sun 3, Mar 2024",
                    bounds,
                    character_style,
                    textbox_style,
                ))
            },
            power_label: {
                // Styling
                let character_style = MonoTextStyle::new(&FONT_6X10, ColorMode::WHITE);
                let textbox_style = TextBoxStyleBuilder::new()
                    .height_mode(embedded_text::style::HeightMode::FitToText)
                    .alignment(HorizontalAlignment::Justified)
                    .build();

                // Bounding box
                let bounds = Rectangle::new(Point::zero(), Size::new(240, 0));

                // Create text box
                Label::new(TextBox::with_textbox_style(
                    "64%",
                    bounds,
                    character_style,
                    textbox_style,
                ))
            },
        }
    }

    fn update(&mut self, state: WatchFaceState) -> DisplayCanvas {
        // Update time label
        // self.time_label.text_box.text = format_no_std::show(
        //     &mut self.time_label.str_buf,
        //     format_args!("{}:{}", state.time.hour(), state.time.minute()),
        // )
        // .unwrap();

        // // Update date label
        // self.date_label.text_box.text = format_no_std::show(
        //     &mut self.date_label.str_buf,
        //     format_args!(
        //         "{}, {} {}, {}",
        //         state.time.weekday(),
        //         state.time.date().day(),
        //         match state.time.month0() {
        //             0 => "Jan",
        //             1 => "Feb",
        //             2 => "Mar",
        //             3 => "Apr",
        //             4 => "May",
        //             5 => "Jun",
        //             6 => "Jul",
        //             7 => "Aug",
        //             8 => "Sep",
        //             9 => "Oct",
        //             10 => "Nov",
        //             11 => "Dec",
        //             _ => "", // TODO: Error handling
        //         },
        //         state.time.year()
        //     ),
        // )
        // .unwrap();

        // // Update power label
        // self.power_label.text_box.text = format_no_std::show(
        //     &mut self.power_label.str_buf,
        //     format_args!("{}%", state.percent),
        // )
        // .unwrap();

        // Draw to canvas
        let mut canvas: DisplayCanvas = CCanvas::new();
        self.time_label.text_box.draw(&mut canvas).unwrap();
        self.date_label.text_box.draw(&mut canvas).unwrap();
        self.power_label.text_box.draw(&mut canvas).unwrap();

        canvas
    }
}
