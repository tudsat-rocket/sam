use core::error;

use defmt::{error, info, warn};
use helpers::{mk_every_nth, mk_fill_percentage, mk_range_index};
use smart_leds::RGB8;

pub mod colors;
pub mod helpers;

pub use colors::*;

#[derive(Clone, Copy, Debug)]
pub struct AnimArgs {
    pub period_len: u32,
    pub period_offset: u32,
    pub primary_color: Color,
    pub secondary_color: Option<Color>,
}
impl AnimArgs {
    pub fn new(
        period_len: u32,
        period_offset: u32,
        primary_color: Color,
        secondary_color: Option<Color>,
    ) -> Result<Self, ()> {
        if period_len == 0 {
            return Err(());
        }
        Ok(Self {
            period_len,
            period_offset,
            primary_color,
            secondary_color,
        })
    }
    pub fn new_simple(period_len: u32, primary_color: Color) -> Result<Self, ()> {
        if period_len == 0 {
            return Err(());
        }
        Ok(Self {
            period_len,
            period_offset: 0,
            primary_color,
            secondary_color: Some(BLACK),
        })
    }
}

pub fn small_trickle(time: u32, buffer: &mut [RGB8], args: &AnimArgs) {
    let (period_len, offset, primary, secondary) =
        (args.period_len, args.period_offset, args.primary_color, args.secondary_color);

    let n_equal_steps = 4;
    let time_in_period = (time + offset) % period_len;
    let which_step = period_len / (time_in_period * n_equal_steps);
    mk_every_nth(4, which_step as usize, primary.into(), buffer, secondary.map(|c| c.into()));
}

pub fn blocky_blink_switch(time: u32, buffer: &mut [RGB8], args: &AnimArgs) {
    let (period_len, offset, primary, secondary) =
        (args.period_len, args.period_offset, args.primary_color, args.secondary_color);

    let n_equal_steps = 2;
    let time_in_period = (time + offset) % period_len;
    let which_step = period_len / (time_in_period * n_equal_steps);
    mk_every_nth(4, which_step as usize, primary.into(), buffer, secondary.map(|c| c.into()));
}
// speed in percentage per second
pub fn ping_pong(
    time: u32,
    buffer: &mut [RGB8],
    args: &AnimArgs,
    ball_size_percentage: u8,
    mut update_cb: impl FnMut(&[RGB8]),
) {
    info!("fn ping_pong");
    let (period_len, offset, primary, secondary) =
        (args.period_len, args.period_offset, args.primary_color, args.secondary_color);

    let ball_size = ball_size_percentage.clamp(0, 100);
    let ball_size_pixels = (ball_size as usize * buffer.len()) / 100;

    let n_equal_steps = (buffer.len() - ball_size_pixels) * 2;
    info!("n_equal_steps: {}", n_equal_steps);
    if n_equal_steps == 0 {
        warn!("fn ping_pong: n_equal_steps = 0");
        mk_fill_percentage(100, primary.into(), buffer, None);
    }
    if n_equal_steps <= 2 {
        error!("fn ping_pong: n_equal_steps <= 2");
    }
    let time_in_period = (time as usize + offset as usize) % period_len as usize;
    if time_in_period == 0 {
        error!("fn ping_pong: time_in_period == 0");
        return;
    }
    let which_step = (time_in_period * n_equal_steps) / period_len as usize;
    // let which_step = period_len as usize / (time_in_period * n_equal_steps);
    info!("period_len: {}, time_in_period: {} => which_step: {}", period_len, time_in_period, which_step);

    let start_i = if which_step <= n_equal_steps / 2 {
        //check
        info!("fn_ping_pong: first half");
        which_step
    } else {
        info!("fn_ping_pong: second half");
        // buffer.len() - ball_size_pixels - (buffer.len() / 2 - which_step)
        buffer.len() - ball_size_pixels - (which_step - (which_step / 2) + 1)
    };

    mk_range_index(buffer, start_i..(start_i + ball_size_pixels), primary.into(), secondary.map(|c| c.into()));
    update_cb(buffer);
}
