use embassy_time::{Duration, Ticker};
use embassy_stm32::peripherals::*;
use embassy_stm32::timer::simple_pwm::{SimplePwmChannel, SimplePwm};

#[embassy_executor::task]
pub async fn run_servo_check(
    mut pwm: SimplePwm<'static, TIM3>,
    toggle_time: Duration,
) -> ! {
    
    let mut ticker = Ticker::every(toggle_time);
    let mut pwm_out = pwm.ch4(); 
    pwm_out.enable();
    
    loop{
        
        pwm_out.set_duty_cycle_percent(33);
        
        ticker.next().await;
        
        pwm_out.set_duty_cycle_fully_off();

        ticker.next().await;
        
    }

}

