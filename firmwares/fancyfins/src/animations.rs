use core::usize;

use smart_leds::RGB8;

const RED: RGB8 = RGB8::new(0, 255, 0);
const GREEN: RGB8 = RGB8::new(255, 0, 0);
const BLUE: RGB8 = RGB8::new(0, 0, 255);
const WHITE: RGB8 = RGB8::new(255, 255, 255);
const BLACK: RGB8 = RGB8::new(0, 0, 0);

// / Takes a percentage and lights up the leds up to this part.
pub fn mk_fill_percentage(percentage: u8, color: RGB8) -> [RGB8; 20] {
    let mut arr: [RGB8; 20] = [RGB8::default(); 20];
    let mut top_position = percentage / 5;
    top_position = u8::min(top_position, 20);
    for i in 0..top_position {
        arr[i as usize] = color;
    }
    arr
}

/// Takes the position of the led counted from the bottom up, and returns the index in the chain.
/// This is unique to Hyacinth FancyFin hardware.
fn led_position_to_index(position: u32) -> u32 {
    /*
    0 -> 10
    1 -> 9
    2 -> 11
    3 -> 8
    */
    match position % 2 {
        // right side (looked at from the outside)
        0 => 10 + (position / 2),
        /*
        0 =>  acc. 10
        2 =>  acc. 11
        4 =>  acc. 12
        6 =>  acc. 13
        8 =>  acc. 14
        10 =>  acc. 15
         */
        // left side (looked at from the outside)
        1 => 10 - ((position / 2) + 1),
        /*
        1 =>  acc. 9
        3 =>  acc. 8
        5 =>  acc. 7
        7 =>  acc. 6
        9 =>  acc. 5
        11 =>  acc. 4
        13 =>  acc. 3
        15 =>  acc. 2
        17 => 1 acc. 1
        19 => 0 acc. 0
         */
        _ => unreachable!(),
    }
}
