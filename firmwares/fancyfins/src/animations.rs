use defmt::info;
use smart_leds::RGB8;

/// This is unique to Wirth leds on the Hyacinth FancyFin hardware.
pub const GREEN: RGB8 = RGB8::new(0, 255, 0);
pub const RED: RGB8 = RGB8::new(255, 0, 0);
pub const BLUE: RGB8 = RGB8::new(0, 0, 255);

pub const ORANGE: RGB8 = RGB8::new(255, 127, 0);
pub const LILA: RGB8 = RGB8::new(125, 0, 125);

pub const WHITE: RGB8 = RGB8::new(255, 255, 255);
pub const BLACK: RGB8 = RGB8::new(0, 0, 0);

pub fn half(color: &RGB8) -> RGB8 {
    RGB8::new(color.r / 2, color.g / 2, color.b / 2)
}

/// Takes a percentage and lights up the leds up to (but not including) this part.
/// Imagined like a liquid filling up to that point
/// Other leds will be turned off.
pub fn mk_fill_percentage(percentage: u8, color: RGB8, buffer: &mut [RGB8], background: Option<RGB8>) {
    info!("mk_fill_percentage: {}", percentage);
    let percentage = percentage.clamp(0, 100);
    let top_position = (percentage as usize * buffer.len()) / 100;
    for i in 0..top_position {
        buffer[led_position_to_index(i)] = color;
    }
    if let Some(background) = background {
        for i in top_position..buffer.len() {
            buffer[led_position_to_index(i)] = background;
        }
    }
}
/// for every_nth = 3, every third led will be turned on
pub fn mk_every_nth(every_nth: usize, offset: usize, color: RGB8, buffer: &mut [RGB8], background: Option<RGB8>) {
    if let Some(background) = background {
        buffer.iter_mut().for_each(|l| *l = background);
    }
    for i in (offset..buffer.len()).step_by(every_nth) {
        buffer[led_position_to_index(i)] = color;
    }
}

/// Takes the position of the led counted from the bottom up, and returns the index in the chain.
/// This is unique to Hyacinth FancyFin hardware.
fn led_position_to_index(position: usize) -> usize {
    let half = position / 2;
    10 + half - (position % 2) * (2 * half + 1)
}
