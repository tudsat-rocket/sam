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

    // In order for the defmt stuff to be properly linked, we need to make sure
    // we import the device HAL. The rust compiler doesn't get that, so we have
    // to suppress the warning about an unused import.
    #[allow(unused_imports)]
    use stm32f4xx_hal as hal;

    use sting_fc_firmware::telemetry::*;

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
        assert_eq!(msg.serialize().unwrap().len(), 60);

        let msg = DownlinkMessage::TelemetryRawSensorsCompressed(TelemetryRawSensorsCompressed::default());
        assert_eq!(msg.serialize().unwrap().len(), 18);

        let msg = DownlinkMessage::TelemetryDiagnostics(TelemetryDiagnostics::default());
        assert_eq!(msg.serialize().unwrap().len(), 13);

        let msg = DownlinkMessage::TelemetryGPS(TelemetryGPS::default());
        assert_eq!(msg.serialize().unwrap().len(), 14);

        let msg = DownlinkMessage::TelemetryGCS(TelemetryGCS::default());
        assert_eq!(msg.serialize().unwrap().len(), 7);
    }

    #[test]
    fn uplink_msg_sizes_correct() {
        let msg = UplinkMessage::Heartbeat;
        assert_eq!(msg.serialize().unwrap().len(), 3);

        let msg = UplinkMessage::Reboot;
        assert_eq!(msg.serialize().unwrap().len(), 3);

        let msg = UplinkMessage::RebootAuth(0x1234);
        assert_eq!(msg.serialize().unwrap().len(), 5);

        let msg = UplinkMessage::RebootToBootloader;
        assert_eq!(msg.serialize().unwrap().len(), 3);

        let msg = UplinkMessage::SetFlightMode(FlightMode::Idle);
        assert_eq!(msg.serialize().unwrap().len(), 4);

        let msg = UplinkMessage::SetFlightModeAuth(FlightMode::Idle, 0x1234);
        assert_eq!(msg.serialize().unwrap().len(), 6);

        let msg = UplinkMessage::ReadFlash(0x0100, 0x1000);
        assert_eq!(msg.serialize().unwrap().len(), 7);

        let msg = UplinkMessage::EraseFlash;
        assert_eq!(msg.serialize().unwrap().len(), 3);

        let msg = UplinkMessage::EraseFlashAuth(0x1234);
        assert_eq!(msg.serialize().unwrap().len(), 5);
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
