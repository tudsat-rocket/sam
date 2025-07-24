use embassy_stm32::peripherals::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn run_servo_check(mut pwm: SimplePwm<'static, TIM3>, toggle_time: Duration) -> ! {
    let mut ticker = Ticker::every(toggle_time);
    let mut pwm_out = pwm.ch3();
    pwm_out.enable();

    loop {
        // 1 ms high (in 20ms period)
        pwm_out.set_duty_cycle_percent(5);

        ticker.next().await;

        // 2 ms high (in 20ms period)
        pwm_out.set_duty_cycle_percent(10);

        ticker.next().await;
    }
}
