use defmt::{error, info, warn};
use libm::{round, roundf};
use smart_leds::RGB8;

/// Takes a percentage and lights up the leds up to (but not including) this part.
/// Imagined like a liquid filling up to that point
/// Other leds will be turned off.
pub fn mk_fill_percentage(percentage: u8, color: RGB8, buffer: &mut [RGB8], background: Option<RGB8>) {
    mk_range_percentage(buffer, 0..percentage, color, background);
}
pub fn mk_fill_percentage_reverse(percentage: u8, color: RGB8, buffer: &mut [RGB8], background: Option<RGB8>) {
    let range = (100 - percentage)..100;
    mk_range_percentage(buffer, range, color, background);
}

pub fn mk_range_index(
    buffer: &mut [RGB8],
    range_index: core::ops::Range<usize>,
    color: RGB8,
    background: Option<RGB8>,
) {
    info!("fn mk_range_index: size: {}", range_index.size_hint());
    let safe_range = range_index.start.clamp(0, buffer.len())..range_index.end.clamp(0, buffer.len());
    if safe_range != range_index {
        warn!("fn mk_range_index has been used with unsafe input: range_index: {:?}", range_index);
    }
    safe_range.clone().for_each(|i| set_color_at_index(buffer, i, color));
    if let Some(background) = background {
        (0..(safe_range.start)).for_each(|i| set_color_at_index(buffer, i, background));
        (safe_range.end..buffer.len()).for_each(|i| set_color_at_index(buffer, i, background));
    }
}

pub fn mk_range_percentage(
    buffer: &mut [RGB8],
    range_percentage: core::ops::Range<u8>,
    color: RGB8,
    background: Option<RGB8>,
) {
    let safe_range = range_percentage.start.clamp(0, 100)..range_percentage.end.clamp(0, 100);
    if safe_range != range_percentage {
        warn!("fn mk_range_percentage has been used with unsafe input: range_percentage: {:?}", range_percentage);
    }
    let start_i = safe_range.start as usize * buffer.len() / 100;
    let end_i = safe_range.end as usize * buffer.len() / 100;

    mk_range_index(buffer, start_i..end_i, color, background);
}

/// for every_nth = 3, every third led will be turned on
pub fn mk_every_nth(
    every_nth: usize,
    offset: usize,
    color: RGB8,
    buffer: &mut [RGB8],
    background: Option<RGB8>,
) -> bool {
    if let Some(background) = background {
        mk_fill_percentage(100, background, buffer, None);
    }

    for i in (offset..buffer.len()).step_by(every_nth) {
        buffer[led_position_to_index(i)] = color;
    }
    true
}

/// The brightest spot will be at the top
/// Fade stretch percentage may be larger than 100%,
pub fn mk_brightness_fade_from_top(
    buffer: &mut [RGB8],
    color: RGB8,
    fade_stretch_percentage: u32,
    offset_from_top_percentage: u8,
    top_background: Option<RGB8>,
    bottom_background: Option<RGB8>,
) {
    if let Some(background) = top_background {
        mk_fill_percentage_reverse(offset_from_top_percentage, background, buffer, None);
    }

    let percentage = offset_from_top_percentage.clamp(0, 100);
    // exclusive
    let top_position = (percentage as usize * buffer.len()) / 100;

    let n_virtual_on = (fade_stretch_percentage as usize * buffer.len()) / 100;

    let dimm_per_step: f32 = match n_virtual_on {
        0 => 0.0,
        _ => 1.0 / n_virtual_on as f32,
    };
    let fade_index_range = usize::max(top_position - 1 - n_virtual_on, 0)..top_position;
    for (step, index_led) in fade_index_range.rev().enumerate() {
        let dimmed = set_logic_brightness(&color, 1.0 - dimm_per_step * step as f32);
        buffer[led_position_to_index(index_led)] = dimmed;
    }

    if let Some(background) = bottom_background {
        let background_index_range = 0..(usize::max(top_position - n_virtual_on, 0));
        background_index_range.for_each(|i| buffer[led_position_to_index(i)] = background);
    }
}

/// The brightest spot will be at the bottom.
/// Fade stretch percentage may be larger than 100%,
pub fn mk_brightness_fade_from_bottom(
    buffer: &mut [RGB8],
    color: RGB8,
    fade_stretch_percentage: u32,
    offset_from_bottom_percentage: u8,
    bottom_background: Option<RGB8>,
    top_background: Option<RGB8>,
) {
    if let Some(background) = bottom_background {
        mk_fill_percentage(offset_from_bottom_percentage, background, buffer, None);
    }

    let percentage = offset_from_bottom_percentage.clamp(0, 100);
    // inclusive
    let lowest_position = (percentage as usize * buffer.len()) / 100;

    let n_virtual_on = (fade_stretch_percentage as usize * buffer.len()) / 100;

    let dimm_per_step: f32 = match n_virtual_on {
        0 => 0.0,
        _ => 1.0 / n_virtual_on as f32,
    };
    let fade_index_range = lowest_position..(usize::min(lowest_position + n_virtual_on + 1, buffer.len()));
    for (step, index_led) in fade_index_range.enumerate() {
        let dimmed = set_logic_brightness(&color, 1.0 - dimm_per_step * step as f32);
        buffer[led_position_to_index(index_led)] = dimmed;
    }

    if let Some(background) = top_background {
        let background_index_range = (lowest_position + n_virtual_on + 1)..buffer.len();
        background_index_range.for_each(|i| buffer[led_position_to_index(i)] = background);
    }
}

fn set_logic_brightness(color: &RGB8, new_brightness: f32) -> RGB8 {
    let brightness = new_brightness.clamp(0.0, 1.0);
    return RGB8::new(
        // (color.r as f32 * brightness).round().clamp(0.0, 255.0) as u8,
        (roundf(color.r as f32 * brightness)).clamp(0.0, 255.0) as u8,
        (roundf(color.g as f32 * brightness)).clamp(0.0, 255.0) as u8,
        (roundf(color.b as f32 * brightness)).clamp(0.0, 255.0) as u8,
    );
}
// fn set_percieved_brightness

/// Takes the position of the led counted from the bottom up, and returns the index in the chain.
/// This is unique to Hyacinth FancyFin hardware.
fn led_position_to_index(position: usize) -> usize {
    let half = position / 2;
    10 + half - (position % 2) * (2 * half + 1)
}

fn set_color_at_index(buffer: &mut [RGB8], i: usize, color: RGB8) {
    let index_in_buffer = led_position_to_index(i);
    if index_in_buffer < buffer.len() {
        buffer[led_position_to_index(i)] = color;
    } else {
        error!("fn set_color_at_index: buffer access would be out of range");
    }
}
