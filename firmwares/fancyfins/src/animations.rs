use defmt::info;
use smart_leds::RGB8;

/// This is unique to Wirth leds on the Hyacinth FancyFin hardware.
pub const GREEN: RGB8 = RGB8::new(0, 255, 0);
pub const RED: RGB8 = RGB8::new(255, 0, 0);
pub const BLUE: RGB8 = RGB8::new(0, 0, 255);
pub const WHITE: RGB8 = RGB8::new(255, 255, 255);
pub const BLACK: RGB8 = RGB8::new(0, 0, 0);

///  Takes a percentage and lights up the leds up to (but not including) this part.
/// Other leds will be turned off.
pub fn mk_fill_percentage(percentage: u8, color: RGB8, buffer: &mut [RGB8], background: Option<RGB8>) {
    info!("mk_fill_percentage: {}", percentage);
    let percentage = percentage.clamp(0, 100);
    let top_position = (percentage as usize * buffer.len()) / 100;
    for i in 0..top_position {
        buffer[led_position_to_index(i as u32) as usize] = color;
    }
    if let Some(background) = background {
        for i in top_position..buffer.len() {
            buffer[led_position_to_index(i as u32) as usize] = background;
        }
    }
}

/// Takes the position of the led counted from the bottom up, and returns the index in the chain.
/// This is unique to Hyacinth FancyFin hardware.
fn led_position_to_index(position: u32) -> u32 {
    let half = position / 2;
    10 + half - (position % 2) * (2 * half + 1)
}
