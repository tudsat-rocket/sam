use smart_leds::{RGB, RGB8};
// pub const GREEN: RGB8 = RGB8::new(0, 255, 0);
// pub const RED: RGB8 = RGB8::new(255, 0, 0);
// pub const BLUE: RGB8 = RGB8::new(0, 0, 255);
//
// pub const ORANGE: RGB8 = RGB8::new(255, 127, 0);
// pub const LILA: RGB8 = RGB8::new(125, 0, 125);
//
// pub const WHITE: RGB8 = RGB8::new(255, 255, 255);
// pub const BLACK: RGB8 = RGB8::new(0, 0, 0);

pub const GREEN: Color = Color::new_rgb(0, 255, 0);
pub const RED: Color = Color::new_rgb(255, 0, 0);
pub const BLUE: Color = Color::new_rgb(0, 0, 255);
pub const ORANGE: Color = Color::new_rgb(255, 127, 0);
pub const LILA: Color = Color::new_rgb(125, 0, 125);
pub const WHITE: Color = Color::new_rgb(255, 255, 255);
pub const BLACK: Color = Color::new_rgb(0, 0, 0);
pub const YELLOW: Color = Color::new_rgb(255, 255, 0);
#[derive(Clone, Copy, Debug)]
pub struct Color(RGB8);
impl From<Color> for RGB8 {
    fn from(value: Color) -> Self {
        value.0
    }
}
impl From<RGB8> for Color {
    fn from(value: RGB8) -> Self {
        Self(value)
    }
}
impl Color {
    pub const fn new_rgb(red: u8, green: u8, blue: u8) -> Self {
        Self(RGB {
            r: red,
            g: green,
            b: blue,
        })
    }
}
pub fn half(color: &RGB8) -> RGB8 {
    RGB8::new(color.r / 2, color.g / 2, color.b / 2)
}
