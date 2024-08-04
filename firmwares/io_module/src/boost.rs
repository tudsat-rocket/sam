use defmt::{error, Debug2Format};
use embassy_stm32::{gpio::Output, i2c::I2c};
use embassy_stm32::peripherals::*;
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn run_boost_converter(
    mut i2c: I2c<'static, I2C2, DMA1_CH4, DMA1_CH5>,
    target_voltage: u16,
    mut enable_pin: Output<'static, PB0>
) -> ! {
    const I2C_ADDRESS: u8 = 0b0101111;
    const R_AB: u16 = 5000; // Max resistance (5kOhm)

    const R_FB1: u16 = 33_000;
    const R_FB2: u16 = 1_500; // plus pot output (0-R_AB)
                              //
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        // https://www.ti.com/lit/ds/symlink/lm3488.pdf
        // p. 21
        let target_fb2 = 1.0 / ((((target_voltage as f32) / 1.26e3) - 1.0) / (R_FB1 as f32));
        let target_pot_resistance = target_fb2 - (R_FB2 as f32);
        defmt::println!("target resistance: {:?}", target_pot_resistance);

        let target_pot_value = (128.0 * target_pot_resistance / (R_AB as f32)) as u8 & 0x7f;

        defmt::println!("{:?}", target_pot_value);
        if let Err(e) = i2c.write(I2C_ADDRESS, &[target_pot_value]).await {
            error!("Failed to set pot value: {:?}", Debug2Format(&e))
        } else {
            enable_pin.set_low();
        }

        let mut buffer = [0u8];
        if let Err(e) = i2c.read(I2C_ADDRESS, &mut buffer).await {
            error!("Failed to read pot value: {:?}", Debug2Format(&e))
        } else {
            defmt::println!("Step Value: {:?}", buffer);
        }

        ticker.next().await;
    }
}
