#![cfg_attr(target_os = "none", no_std)] // this is imported by the firmware, so no standard library

mod metrics;
mod representation;
mod schema;
mod source;
use core::convert::Infallible;

pub use metrics::*;
pub use representation::*;
pub use schema::*;
pub use source::*;

pub const ALTITUDE_REPRESENTATION: Representation = Representation::fixed(16, -100.0..=10_000.0);

pub const TELEMETRY_GPS: MessageDefinition = MessageDefinition(&[
    (Metric::Latitude, Representation::float(64)),
    (Metric::Longitude, Representation::float(64)),
]);

pub const TELEMETRY_RAW_SENSORS: MessageDefinition = MessageDefinition(&[
    (Metric::RawAngularVelocity(GyroscopeId::LSM6DSR, Dim::X), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::LSM6DSR, Dim::Y), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::LSM6DSR, Dim::Z), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::LSM6DSR, Dim::X), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::LSM6DSR, Dim::Y), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::LSM6DSR, Dim::Z), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::ICM42670P, Dim::X), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::ICM42670P, Dim::Y), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::ICM42670P, Dim::Z), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::ICM42670P, Dim::X), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::ICM42670P, Dim::Y), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::ICM42670P, Dim::Z), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::ICM42688P, Dim::X), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::ICM42688P, Dim::Y), Representation::float(16)),
    (Metric::RawAngularVelocity(GyroscopeId::ICM42688P, Dim::Z), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::ICM42688P, Dim::X), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::ICM42688P, Dim::Y), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::ICM42688P, Dim::Z), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::H3LIS331, Dim::X), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::H3LIS331, Dim::Y), Representation::float(16)),
    (Metric::RawAcceleration(AccelerometerId::H3LIS331, Dim::Z), Representation::float(16)),
    (Metric::RawMagneticFluxDensity(MagnetometerId::LIS3MDL, Dim::X), Representation::float(16)),
    (Metric::RawMagneticFluxDensity(MagnetometerId::LIS3MDL, Dim::Y), Representation::float(16)),
    (Metric::RawMagneticFluxDensity(MagnetometerId::LIS3MDL, Dim::Z), Representation::float(16)),
    // recheck which of those are present on the new FC (stigma)
    (Metric::Pressure(PressureSensorId::FlightComputer(BarometerId::MS5611)), Representation::float(32)),
    (Metric::Pressure(PressureSensorId::FlightComputer(BarometerId::LPS22)), Representation::float(32)),
    (Metric::Pressure(PressureSensorId::FlightComputer(BarometerId::BMP580)), Representation::float(32)),
]);

pub const TELEMETRY_DIAGNOSTICS: MessageDefinition = MessageDefinition(&[
    (Metric::CpuUtilization, Representation::fixed(8, 0.0..=100.0)),
    (Metric::SupplyVoltage, Representation::fixed(16, 0.0..=25.0)),
    (Metric::BatteryVoltage(BatteryId::Avionics), Representation::fixed(16, 0.0..=25.0)),
    (Metric::UplinkRssi, Representation::fixed(8, -100.0..=0.0)),
    (Metric::GroundAltitudeASL, ALTITUDE_REPRESENTATION),
    (Metric::TransmitPower, Representation::Enum { bits: 8 }),
    // reduce temperature metrics sent when clear which baros still present
    (
        Metric::Temperature(TemperatureSensorId::Barometer(BarometerId::MS5611)),
        Representation::fixed(8, -20.0..=120.0),
    ),
    (
        Metric::Temperature(TemperatureSensorId::Barometer(BarometerId::LPS22)),
        Representation::fixed(8, -20.0..=120.0),
    ),
    (
        Metric::Temperature(TemperatureSensorId::Barometer(BarometerId::BMP580)),
        Representation::fixed(8, -20.0..=120.0),
    ),
]);

pub const TELEMETRY_MAIN: MessageDefinition = MessageDefinition(&[
    (Metric::FlightMode, Representation::Enum { bits: 8 }),
    (Metric::ProcedureStep, Representation::Enum { bits: 8 }),
    (Metric::AnomalousState, Representation::Enum { bits: 8 }),
    (Metric::Orientation(0), Representation::float(32)),
    (Metric::Orientation(1), Representation::float(32)),
    (Metric::Orientation(2), Representation::float(32)),
    (Metric::Orientation(3), Representation::float(32)),
    (Metric::Azimuth, Representation::fixed(8, 0.0..=360.0)),
    (Metric::Elevation, Representation::fixed(8, -90.0..=90.0)),
    (Metric::AccelerationWorldSpace(Dim::Z), Representation::float(16)),
    (Metric::VelocityWorldSpace(Dim::Z), Representation::float(16)),
    (Metric::PositionWorldSpace(Dim::Z), ALTITUDE_REPRESENTATION),
    // TODO: reduce number of different baros
    (Metric::RawBarometricAltitude(BarometerId::MS5611), ALTITUDE_REPRESENTATION),
    (Metric::RawBarometricAltitude(BarometerId::LPS22), ALTITUDE_REPRESENTATION),
    (Metric::RawBarometricAltitude(BarometerId::BMP580), ALTITUDE_REPRESENTATION),
    (Metric::ApogeeAltitudeASL, ALTITUDE_REPRESENTATION),
    (Metric::BatteryCurrent(BatteryId::Avionics), Representation::fixed(16, -50.0..=50.0)),
]);

pub const TELEMETRY_PRESSURES: MessageDefinition = MessageDefinition(&[
    (Metric::Pressure(PressureSensorId::AcsTank), Representation::fixed(16, 0.0..=400.0)),
    (Metric::Pressure(PressureSensorId::AcsPostRegulator), Representation::fixed(16, 0.0..=40.0)),
    (Metric::Pressure(PressureSensorId::AcsValveAccel), Representation::fixed(8, 0.0..=40.0)),
    (Metric::Pressure(PressureSensorId::AcsValveDecel), Representation::fixed(8, 0.0..=40.0)),
    (Metric::Pressure(PressureSensorId::RecoveryChamberDrogue), Representation::fixed(16, 0.0..=40.0)),
]);

// split out ereg, ox tank, cc to higher rate telemetry
pub const TELEMETRY_HYBRID: MessageDefinition = MessageDefinition(&[
    (Metric::Pressure(PressureSensorId::CombustionChamber), Representation::fixed(16, 0.0..=40.0)),
    (Metric::Pressure(PressureSensorId::OxidizerTank), Representation::fixed(8, 0.0..=100.0)),
    (Metric::Pressure(PressureSensorId::NitrogenTank), Representation::fixed(16, 0.0..=300.0)),
    (Metric::ValveState(ValveId::PressureRegulator), Representation::fixed(8, 0.0..=1.0)),
    (Metric::ValveState(ValveId::MainValve), Representation::fixed(8, 0.0..=1.0)),
    (Metric::ValveState(ValveId::VentValve), Representation::fixed(8, 0.0..=1.0)),
    (Metric::ValveState(ValveId::FillAndDumpValve), Representation::fixed(8, 0.0..=1.0)),
    (Metric::Temperature(TemperatureSensorId::OxidizerTank), Representation::fixed(8, -80.0..=40.0)),
]);

pub const TELEMETRY_KALMAN: MessageDefinition = MessageDefinition(&[]); // TODO low prio

pub const TELEMETRY_BUS: MessageDefinition = MessageDefinition(&[]); // TODO low prio

pub const TELEMETRY_FAST_COMPRESSED: MessageDefinition = MessageDefinition(&[
    (Metric::FlightMode, Representation::Enum { bits: 8 }),
    (Metric::Azimuth, Representation::fixed(8, 0.0..=360.0)),
    (Metric::Elevation, Representation::fixed(8, -90.0..=90.0)),
    (Metric::VelocityWorldSpace(Dim::Z), Representation::fixed(8, -100.0..=400.0)),
    (Metric::AccelerationWorldSpace(Dim::Z), Representation::fixed(8, -128.0..=128.0)),
    (Metric::PositionWorldSpace(Dim::Z), ALTITUDE_REPRESENTATION),
    (Metric::RawBarometricAltitude(BarometerId::MS5611), ALTITUDE_REPRESENTATION), // TODO: encode as offset lowish prio?
    (Metric::ApogeeAltitudeASL, ALTITUDE_REPRESENTATION),
    (Metric::BatteryCurrent(BatteryId::Acs), ALTITUDE_REPRESENTATION),
    (Metric::Pressure(PressureSensorId::CombustionChamber), Representation::fixed(16, 0.0..=40.0)),
    (Metric::Pressure(PressureSensorId::OxidizerTank), Representation::fixed(8, 0.0..=100.0)),
]);

pub const USB_SCHEMA: TelemetrySchema = TelemetrySchema::new(&[
    (TELEMETRY_GPS, 1000, 0),
    (TELEMETRY_RAW_SENSORS, 5, 0),
    (TELEMETRY_MAIN, 50, 12),
    (TELEMETRY_DIAGNOSTICS, 50, 32),
]);

pub const LORA_SCHEMA: TelemetrySchema = TelemetrySchema::new(&[
    (TELEMETRY_GPS, 1000, 0),
    (TELEMETRY_DIAGNOSTICS, 1000, 200),
    (TELEMETRY_PRESSURES, 1000, 400),
    (TELEMETRY_KALMAN, 1000, 600),
    (TELEMETRY_BUS, 1000, 800),
    (TELEMETRY_FAST_COMPRESSED, 100, 50),
    (TELEMETRY_HYBRID, 1000, 850),
]);

pub const FLASH_SCHEMA: TelemetrySchema = TelemetrySchema::new(&[
    (TELEMETRY_GPS, 100, 0),
    (TELEMETRY_DIAGNOSTICS, 50, 0),
    (TELEMETRY_MAIN, 50, 20),
    (TELEMETRY_RAW_SENSORS, 10, 5),
]);

impl Default for &'static TelemetrySchema {
    fn default() -> Self {
        &USB_SCHEMA
    }
}
