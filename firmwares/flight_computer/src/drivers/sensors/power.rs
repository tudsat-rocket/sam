use embassy_stm32::adc::{Adc, Temperature, VrefInt, AdcPin, Instance, SampleTime};
use embassy_stm32::gpio::low_level::Pin;
use embassy_time::{Timer, Duration, Instant};

use crc::{Crc, CRC_16_IBM_SDLC};

const VDIV: f32 = 2.8;
const RES: f32 = 0.01;

const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

pub struct PowerMonitor<ADC: Instance, H, L, A> {
    adc: Adc<'static, ADC>,
    vref_sample: u16,
    internal_temperature: Temperature,
    pin_bat_high: H,
    pin_bat_low: L,
    pin_arm: A,

    battery_voltage: Option<u16>,
    battery_current: Option<i16>,
    arm_voltage: Option<u16>,
    temperature: Option<f32>,

    can_charge_voltage: Option<u16>,
    can_battery_voltage: Option<u16>,
    can_battery_current: Option<i16>,
    can_breakwire_open: Option<bool>,
    time_last_can_msg: Option<Instant>,
}

impl<
    ADC: Instance,
    H: Pin + AdcPin<ADC>,
    L: Pin + AdcPin<ADC>,
    A: Pin + AdcPin<ADC>,
> PowerMonitor<ADC, H, L, A>
where
    Temperature: AdcPin<ADC>,
    VrefInt: AdcPin<ADC>,
{
    pub async fn init(mut adc: Adc<'static, ADC>, pin_bat_high: H, pin_bat_low: L, pin_arm: A) -> Self {
        adc.set_sample_time(SampleTime::Cycles480);

        let mut internal_vref = adc.enable_vrefint();
        let internal_temperature = adc.enable_temperature();
        let start_time = Temperature::start_time_us().max(VrefInt::start_time_us());
        Timer::after(Duration::from_micros(start_time as u64)).await;

        let vref_sample = adc.read(&mut internal_vref);

        Self {
            adc,
            internal_temperature,
            vref_sample,
            pin_bat_high,
            pin_bat_low,
            pin_arm,
            battery_voltage: None,
            battery_current: None,
            arm_voltage: None,
            temperature: None,
            can_charge_voltage: None,
            can_battery_voltage: None,
            can_battery_current: None,
            can_breakwire_open: None,
            time_last_can_msg: None,
        }
    }

    fn to_millivolts(&self, sample: u16) -> u16 {
        // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
        // 6.3.24 Reference voltage
        const VREFINT_MV: u32 = 1210; // mV

        (u32::from(sample) * VREFINT_MV / u32::from(self.vref_sample)) as u16
    }

    fn to_celcius(&self, sample: u16) -> f32 {
        // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
        // 6.3.22 Temperature sensor characteristics
        const V25: i32 = 760; // mV
        const AVG_SLOPE: f32 = 2.5; // mV/C

        let sample_mv = self.to_millivolts(sample) as i32;

        (sample_mv - V25) as f32 / AVG_SLOPE + 25.0
    }

    pub fn tick(&mut self) {
        let sample = self.adc.read(&mut self.internal_temperature);
        self.temperature = Some(self.to_celcius(sample));

        let sample = self.adc.read(&mut self.pin_bat_high);
        let voltage_high = (self.to_millivolts(sample) as f32) * VDIV;

        let sample = self.adc.read(&mut self.pin_bat_low);
        let voltage_low = (self.to_millivolts(sample) as f32) * VDIV;
        self.battery_voltage = Some(voltage_high as u16);
        self.battery_current = Some(((voltage_high - voltage_low) / RES) as i16);

        let sample = self.adc.read(&mut self.pin_arm);
        self.arm_voltage = Some(((self.to_millivolts(sample) as f32) * VDIV) as u16);
    }

    pub fn battery_voltage(&self) -> Option<u16> {
        self.current_can_msg()
            .then(|| self.can_battery_voltage)
            .unwrap_or(self.battery_voltage)
    }

    pub fn battery_current(&self) -> Option<i16> {
        self.current_can_msg()
            .then(|| self.can_battery_current)
            .unwrap_or(self.battery_current)
    }

    pub fn arm_voltage(&self) -> Option<u16> {
        self.arm_voltage
    }

    pub fn armed(&self) -> bool {
        self.arm_voltage.map(|v| v > 50).unwrap_or(false)
    }

    pub fn temperature(&self) -> Option<f32> {
        self.temperature
    }

    pub fn charge_voltage(&self) -> Option<u16> {
        self.current_can_msg()
            .then(|| self.can_charge_voltage)
            .flatten()
    }

    pub fn breakwire_open(&self) -> Option<bool> {
        self.current_can_msg()
            .then(|| self.can_breakwire_open)
            .flatten()
    }

    pub fn handle_battery_can_msg(&mut self, mut msg: [u8; 8]) {
        let crc = u16::from_le_bytes([msg[6], msg[7]]);
        msg[6] = 0;
        msg[7] = 0;

        if crc != CRC.checksum(&msg) {
            //defmt::println!("CRC mismatch for power msg: {:04x} vs {:04x}", crc, CRC.checksum(&msg));
            //return;
        }

        self.can_charge_voltage = Some(u16::from_le_bytes([msg[0], msg[1] >> 1]));
        self.can_battery_current = Some(u16::from_le_bytes([msg[2], msg[3] >> 1]) as i16 - 2000);
        self.can_battery_voltage = Some(u16::from_le_bytes([msg[4], msg[5] >> 2]));
        self.can_breakwire_open = match msg[5] & 0x03 {
            0b00 => Some(false),
            0b01 => Some(true),
            _ => None
        };

        self.time_last_can_msg = Some(Instant::now());
    }

    fn current_can_msg(&self) -> bool {
        self.time_last_can_msg.map(|i| i.elapsed().as_millis() < 30).unwrap_or(false)
    }
}
