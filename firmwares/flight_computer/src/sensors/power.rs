use embedded_hal::adc::Channel;
use hal::adc::config::SampleTime;
use hal::adc::Adc;
use hal::pac::ADC1;
use stm32f4xx_hal as hal;

const ST: SampleTime = SampleTime::Cycles_480;
const VDIV: f32 = 2.8;
const RES: f32 = 0.01;

pub struct PowerMonitor<HIGH, LOW, ARM> {
    adc: Adc<ADC1>,
    pin_bat_high: HIGH,
    pin_bat_low: LOW,
    pin_arm: ARM,
    battery_voltage: Option<u16>,
    battery_current: Option<u16>,
    arm_voltage: Option<u16>,
    cpu_voltage: Option<u16>,
    temperature: Option<f32>,
}

impl<
    HIGH: Channel<ADC1, ID = u8>,
    LOW: Channel<ADC1, ID = u8>,
    ARM: Channel<ADC1, ID = u8>
> PowerMonitor<HIGH, LOW, ARM> {
    pub fn new(
        adc: Adc<ADC1>,
        pin_bat_high: HIGH,
        pin_bat_low: LOW,
        pin_arm: ARM,
    ) -> Self {
        Self {
            adc,
            pin_bat_high,
            pin_bat_low,
            pin_arm,
            battery_voltage: None,
            battery_current: None,
            arm_voltage: None,
            cpu_voltage: None,
            temperature: None,
        }
    }

    pub fn tick(&mut self) {
        let sample = self.adc.convert(&hal::adc::Temperature, ST);
        let mv = self.adc.sample_to_millivolts(sample);
        let temp_cal_30 = hal::signature::VtempCal30::get().read();
        let temp = 30.0 * (mv as f32) / (temp_cal_30 as f32);

        let voltage_core = self.adc.reference_voltage() as u16;

        let sample = self.adc.convert(&self.pin_bat_high, ST);
        let voltage_high = (self.adc.sample_to_millivolts(sample) as f32) * VDIV;

        let sample = self.adc.convert(&self.pin_bat_low, ST);
        let voltage_low = (self.adc.sample_to_millivolts(sample) as f32) * VDIV;

        let sample = self.adc.convert(&self.pin_arm, ST);
        let voltage_arm = (self.adc.sample_to_millivolts(sample) as f32) * VDIV;

        let current = (voltage_high - voltage_low) / RES;

        self.battery_voltage = Some(voltage_high as u16);
        self.battery_current = Some(current as u16);
        self.arm_voltage = Some(voltage_arm as u16);
        self.cpu_voltage = Some(voltage_core);
        self.temperature = Some(temp);
    }

    pub fn battery_voltage(&self) -> Option<u16> {
        self.battery_voltage
    }

    pub fn battery_current(&self) -> Option<u16> {
        self.battery_current
    }

    pub fn arm_voltage(&self) -> Option<u16> {
        self.arm_voltage
    }

    pub fn armed(&self) -> bool {
        self.arm_voltage.map(|v| v > 50).unwrap_or(false)
    }

    pub fn cpu_voltage(&self) -> Option<u16> {
        self.cpu_voltage
    }

    pub fn temperature(&self) -> Option<f32> {
        self.temperature
    }
}
