#[embassy_executor::task]
pub async fn run_i2c_sensors(
    mut input1_i2c: Option<I2c<'static, I2C2, DMA1_CH4, DMA1_CH5>>,
    mut input3_i2c: Option<I2c<'static, I2C1, DMA1_CH6, DMA1_CH7>>,
    mut input3_gpio: Option<(Input<'static, PB6>, Input<'static, PB7>)>,
    publisher: crate::CanOutPublisher,
    role: IoBoardRole,
    interval: Duration,
) -> ! {
    // ADC101C027 addresses in order: ADR floating, GND, VCC
    const AMPLIFIER_ADDRESSES: [u8; 3] = [0b1010000, 0b1010001, 0b1010010];

    let mut ticker = Ticker::every(interval);
    loop {
        // If we use both busses for sensors, we assume that each I2C bus has
        // two sensors, allowing us to fill all four slots of a single sensor
        // message. If we use just one, we can do up to 3 sensors per bus.
        let i2c_sensors = match (input1_i2c.as_mut(), input3_i2c.as_mut(), input3_gpio.as_mut()) {
            (Some(i2c2), Some(i2c1), _) => [
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[1]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[1]).await.ok(),
            ],
            (Some(i2c2), None, pins) => [
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[1]).await.ok(),
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[2]).await.ok(),
                pins.map(|(p0, p1)| (((p0.is_high() as u16) << 1 | (p1.is_high() as u16), false))),
            ],
            (None, Some(i2c1), _) => [
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[1]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[2]).await.ok(),
                None,
            ],
            (None, None, _) => unreachable!(),
        };

        let msg = IoBoardSensorMessage { i2c_sensors };
        let (id, msg) = msg.to_frame(CanBusMessageId::IoBoardInput(role, 0));
        publisher.publish_immediate((id, msg));

        ticker.next().await;
    }
}
