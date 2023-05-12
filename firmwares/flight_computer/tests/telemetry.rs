#![no_std]
#![no_main]
#![feature(default_alloc_error_handler)]

use sting_fc_firmware as _; // memory layout + panic handler

// See https://crates.io/crates/defmt-test/0.3.0 for more documentation (e.g. about the 'state'
// feature)
#[defmt_test::tests]
mod tests {
    extern crate alloc;
    use alloc::vec::Vec;

    use nalgebra::{UnitQuaternion, Vector3};
    // In order for the defmt stuff to be properly linked, we need to make sure
    // we import the device HAL. The rust compiler doesn't get that, so we have
    // to suppress the warning about an unused import.
    #[allow(unused_imports)]
    use stm32f4xx_hal as hal;

    use sting_fc_firmware::telemetry::{*, Command};

    const HEAP_SIZE: usize = 4096;
    static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] = [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

    #[global_allocator]
    static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

    #[init]
    fn init() -> () {
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }

    #[test]
    fn f8_conversion_works() {
        let values: Vec<f32> = alloc::vec![-0.00001, 0.008, -0.125, 0.5, 1.0, -10.0, 100.0, 1000.0];
        let compressed: Vec<f8> = values.iter().map(|x| (*x).into()).collect();
        let recovered: Vec<f32> = compressed.iter().map(|x| (*x).into()).collect();
        assert_eq!(
            recovered,
            alloc::vec![-0.01953125, 0.015625, -0.125, 0.5, 1.0, -10.0, 96.0, 960.0]
        );
    }

    #[test]
    fn downlink_msg_sizes_correct() {
        let msg = DownlinkMessage::TelemetryMain(TelemetryMain::default());
        assert_eq!(msg.serialize().unwrap().len(), 30);

        let msg = DownlinkMessage::TelemetryMainCompressed(TelemetryMainCompressed::default());
        assert_eq!(msg.serialize().unwrap().len(), 15);

        let msg = DownlinkMessage::TelemetryRawSensors(TelemetryRawSensors::default());
        assert_eq!(msg.serialize().unwrap().len(), 64);

        let msg = DownlinkMessage::TelemetryRawSensorsCompressed(TelemetryRawSensorsCompressed::default());
        assert_eq!(msg.serialize().unwrap().len(), 18);

        let msg = DownlinkMessage::TelemetryDiagnostics(TelemetryDiagnostics::default());
        assert_eq!(msg.serialize().unwrap().len(), 13);

        let msg = DownlinkMessage::TelemetryGPS(TelemetryGPS::default());
        assert_eq!(msg.serialize().unwrap().len(), 14);

        let msg = DownlinkMessage::TelemetryGCS(TelemetryGCS::default());
        assert_eq!(msg.serialize().unwrap().len(), 7);

        let msg = DownlinkMessage::TelemetryMain(TelemetryMain {
            time: 0x12345678,
            mode: FlightMode::Armed,
            orientation: Some(UnitQuaternion::from_euler_angles(12.3, 23.4, 45.6)),
            vertical_speed: 5.231,
            vertical_accel: 15.92,
            vertical_accel_filtered: 15.8,
            altitude_baro: 1051.5,
            altitude_max: 1212.5,
            altitude: 1049.3,
        });
        assert_eq!(msg.serialize().unwrap().len(), 51);

        let msg = DownlinkMessage::TelemetryMainCompressed(TelemetryMainCompressed {
            time: 0x12345678,
            mode: FlightMode::Armed,
            orientation: (0x12, 0x23, 0x34, 0x45),
            vertical_speed: 5.231.into(),
            vertical_accel: 15.92.into(),
            vertical_accel_filtered: 15.8.into(),
            altitude_baro: 10515,
            altitude_max: 12125,
            altitude: 10493,
        });
        assert_eq!(msg.serialize().unwrap().len(), 22);

        let msg = DownlinkMessage::TelemetryRawSensors(TelemetryRawSensors {
            time: 0x12345678,
            gyro: Vector3::new(12.3456, 23.4567, 34.5678),
            accelerometer1: Vector3::new(1.512, 0.512, 10.241),
            accelerometer2: Vector3::new(1.759, 0.062, 52.5112),
            magnetometer: Vector3::new(43.123, 51.512, 24.213),
            temperature_baro: 36.12,
            pressure_baro: 1002.3,
        });
        assert_eq!(msg.serialize().unwrap().len(), 68);

        let msg = DownlinkMessage::TelemetryRawSensorsCompressed(TelemetryRawSensorsCompressed {
            time: 0x12345678,
            gyro: Vector3::new(12.3456, 23.4567, 34.5678).into(),
            accelerometer1: Vector3::new(1.512, 0.512, 10.241).into(),
            accelerometer2: Vector3::new(1.759, 0.062, 52.5112).into(),
            magnetometer: Vector3::new(43.123, 51.512, 24.213).into(),
            temperature_baro: 36,
            pressure_baro: 10023,
        });
        assert_eq!(msg.serialize().unwrap().len(), 23);

        let msg = DownlinkMessage::TelemetryDiagnostics(TelemetryDiagnostics {
            time: 0x12345678,
            cpu_utilization: 213,
            heap_utilization: 123,
            temperature_core: 25,
            cpu_voltage: 3100,
            battery_voltage: 7412,
            arm_voltage: 7411,
            current: 2021,
            lora_rssi: 127,
            altitude_ground: 215,
        });
        assert_eq!(msg.serialize().unwrap().len(), 22);

        let msg = DownlinkMessage::TelemetryGPS(TelemetryGPS {
            time: 0x12345678,
            fix_and_sats: 0xff,
            hdop: 9999,
            latitude: [0x12, 0x34, 0x56],
            longitude: [0xf1, 0xf2, 0xf3],
            altitude_asl: 1234,
            flash_pointer: 0xff12,
        });
        assert_eq!(msg.serialize().unwrap().len(), 22);

        let msg = DownlinkMessage::TelemetryGCS(TelemetryGCS {
            time: 0x12345678,
            lora_rssi: 83,
            lora_rssi_signal: 82,
            lora_snr: 10
        });
        assert_eq!(msg.serialize().unwrap().len(), 11);
    }

    #[test]
    fn uplink_msg_sizes_correct() {
        let msg = UplinkMessage::Heartbeat;
        assert_eq!(msg.serialize().unwrap().len(), 3);
        assert_eq!(msg.serialize().unwrap(), [0x01, 0x01, 0x00]);

        let msg = UplinkMessage::Command(Command::Reboot);
        assert_eq!(msg.serialize().unwrap().len(), 4);

        let msg = UplinkMessage::CommandAuth(Command::Reboot, 0x1234123412341234);
        assert_eq!(msg.serialize().unwrap().len(), 13);

        let msg = UplinkMessage::Command(Command::RebootToBootloader);
        assert_eq!(msg.serialize().unwrap().len(), 4);

        let msg = UplinkMessage::Command(Command::SetFlightMode(FlightMode::Idle));
        assert_eq!(msg.serialize().unwrap().len(), 5);

        let msg = UplinkMessage::CommandAuth(Command::SetFlightMode(FlightMode::Armed), 0x1234567812345678);
        assert_eq!(msg.serialize().unwrap().len(), 14);

        let msg = UplinkMessage::ReadFlash(0x0100, 0x1000);
        assert_eq!(msg.serialize().unwrap().len(), 7);

        let msg = UplinkMessage::Command(Command::EraseFlash);
        assert_eq!(msg.serialize().unwrap().len(), 4);

        let msg = UplinkMessage::CommandAuth(Command::EraseFlash, 0x1234123412341234);
        assert_eq!(msg.serialize().unwrap().len(), 13);
    }

    #[test]
    fn serialization_appends_delimiter() {
        let msg = DownlinkMessage::TelemetryMain(TelemetryMain {
            time: 0x7fffffff,
            mode: FlightMode::Armed,
            orientation: None,
            vertical_speed: 42.0,
            vertical_accel: 10.42,
            vertical_accel_filtered: 10.24,
            altitude_baro: 3124.52,
            altitude_max: 4201.12,
            altitude: 3214.56,
        });

        assert!(*msg.serialize().unwrap().last().unwrap() == 0x00);
    }
}
