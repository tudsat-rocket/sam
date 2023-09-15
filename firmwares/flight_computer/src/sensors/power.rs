use crc::{Crc, CRC_16_IBM_SDLC};
use embedded_hal::adc::Channel;
use hal::adc::config::SampleTime;
use hal::adc::Adc;
use hal::pac::ADC1;
use stm32f4xx_hal as hal;

use crate::prelude::*;

const ST: SampleTime = SampleTime::Cycles_480;
const VDIV: f32 = 2.8;
const RES: f32 = 0.01;

const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

pub struct PowerMonitor<HIGH, LOW, ARM> {
    adc: Adc<ADC1>,
    pin_bat_high: HIGH,
    pin_bat_low: LOW,
    pin_arm: ARM,
    battery_voltage: Option<u16>,
    battery_current: Option<i16>,
    arm_voltage: Option<u16>,
    cpu_voltage: Option<u16>,
    temperature: Option<f32>,
    can_charge_voltage: Option<u16>,
    can_battery_voltage: Option<u16>,
    can_battery_current: Option<i16>,
    can_breakwire_open: Option<bool>,
    time: u32,
    time_last_battery_msg: u32,
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
            can_charge_voltage: None,
            can_battery_voltage: None,
            can_battery_current: None,
            can_breakwire_open: None,
            time: 0,
            time_last_battery_msg: 0,
        }
    }

    pub fn tick(&mut self, time: u32) {
        self.time = time;

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
        self.battery_current = Some(current as i16);
        self.arm_voltage = Some(voltage_arm as u16);
        self.cpu_voltage = Some(voltage_core);
        self.temperature = Some(temp);
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

    pub fn cpu_voltage(&self) -> Option<u16> {
        self.cpu_voltage
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

        self.time_last_battery_msg = self.time;
    }

    fn current_can_msg(&self) -> bool {
        self.time.wrapping_sub(self.time_last_battery_msg) < 30
    }
}
