use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::peripherals::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn run_servo_check(mut pwm: SimplePwm<'static, TIM3>, toggle_time: Duration) -> ! {
    let mut ticker = Ticker::every(toggle_time);
    let mut pwm_out = pwm.ch4();
    pwm_out.enable();

    loop {
        // 50Hz = 20ms period
        pwm_out.set_duty_cycle_percent(5);
        defmt::info!("duty_cycle low");

        ticker.next().await;

        pwm_out.set_duty_cycle_percent(8);
        defmt::info!("duty_cycle high");

        ticker.next().await;
    }
}

