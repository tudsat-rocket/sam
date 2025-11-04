#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "serde")]
use strum::{EnumDiscriminants, VariantNames};

// TODO: maybe bitfields so we can do some sort of select-all operation?
#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum GyroscopeId {
    LSM6DSR,
    ICM42670P,
    ICM42688P,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum AccelerometerId {
    LSM6DSR,
    ICM42670P,
    ICM42688P,
    H3LIS331,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MagnetometerId {
    LIS3MDL,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum BarometerId {
    MS5611,
    LPS22,
    BMP580,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum BatteryId {
    Avionics,
    Acs,
    Recovery,
    Payload,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum PressureSensorId {
    FlightComputer(BarometerId),
    AcsTank,
    AcsPostRegulator,
    AcsValveAccel,
    AcsValveDecel,
    RecoveryChamberDrogue,
    RecoveryChamberMain,
    MainRelease,
    //Hybrid
    CombustionChamber,
    OxidizerTank,
    NitrogenTank,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum TemperatureSensorId {
    Barometer(BarometerId),
    Battery(BatteryId),
    Acs,
    Recovery,
    Payload,
    //Hybrid
    OxidizerTank,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ValveId {
    PressureRegulator,
    MainValve,
    VentValve,
    FillAndDumpValve,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Dim {
    X = 0,
    Y = 1,
    Z = 2,
}

/// This enum serves as an identifier for anything the ground station keeps
/// track of and displays for a single flight computer / connection.
#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, VariantNames, EnumDiscriminants))]
#[non_exhaustive]
pub enum Metric {
    FlightMode,
    ProcedureStep,
    TransmitPower,
    AcsMode,
    ThrusterValveState,
    ValveState(ValveId),

    // TODO: bit-fields: camera state, fin presence
    Orientation(usize),
    Elevation,
    Azimuth,
    AccelerationWorldSpace(Dim),
    VelocityWorldSpace(Dim),
    PositionWorldSpace(Dim),
    Latitude,
    Longitude,
    GroundAltitudeASL,
    ApogeeAltitudeASL,
    GroundSpeed,
    KalmanStateCovariance(usize, usize),
    KalmanMeasurementCovariance(usize, usize),
    RawAngularVelocity(GyroscopeId, Dim),
    RawAcceleration(AccelerometerId, Dim),
    RawMagneticFluxDensity(MagnetometerId, Dim),
    RawBarometricAltitude(BarometerId),
    GpsFix,
    GpsLatitude,
    GpsLongitude,
    GpsAltitude,
    GpsHdop,
    GpsSatellites,
    Pressure(PressureSensorId),
    Temperature(TemperatureSensorId),
    BatteryVoltage(BatteryId),
    BatteryCurrent(BatteryId),
    BatteryChargerState(BatteryId),
    SupplyVoltage,
    SupplyCurrent,
    RecoveryCurrent,
    CpuUtilization,
    FlashPointer,
    UplinkRssi,
    UplinkSnr,
    DownlinkRssi,
    DownlinkSnr,

    // Values describing the ground truth when we're in a simulation
    // TODO: actually implement sending these, maybe add a simulation "telemetry" schema?
    TrueElevation,
    TrueAzimuth,
    TrueAccelerationWorldSpace(Dim),
    TrueVelocityWorldSpace(Dim),
    TruePositionWorldSpace(Dim),
    TrueVehicleMass,
    TrueMotorMass,
    TrueThrusterPropellantMass,
    TrueThrust(Dim),

    //These Metrics are only used by SAM, e.g., for visualization and constraint purposes
    LocalMetric(LocalMetric),
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LocalMetric {
    HyacinthAnomalousState,
    //Binary Valve States
    N2BottleValve,
    N2OBottleValve,
    N2ReleaseValve,
    N2OReleaseValve,
    N2QuickDisconnect,
    N2OQuickDisconnect,
    N2PurgeValve,
    N2PressureRegulator,
    N2OVentValve,
    N2OBurstDisc,
    N2OFillAndDumpValve,
    N2OMainValve,
    //
    MaxPressureN2Tank,
}

#[cfg(not(target_os = "none"))]
impl std::fmt::Display for Metric {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&match self {
            Self::FlightMode => "Flight Mode".to_string(),
            Self::Orientation(i) => format!("Orientation Quaternion [{i}]"),
            Self::Elevation => "Elevation [°]".to_string(),
            Self::Azimuth => "Azimuth [°]".to_string(),

            Self::AccelerationWorldSpace(Dim::Z) => "Vertical Acceleration [m/s²]".to_string(),
            Self::AccelerationWorldSpace(dim) => format!("World-space Accleration ({dim:?}) [m/s²]"),
            Self::VelocityWorldSpace(Dim::Z) => "Vertical Speed [m/s]".to_string(),
            Self::VelocityWorldSpace(dim) => format!("World-space Velocity ({dim:?}) [m/s]"),
            Self::PositionWorldSpace(Dim::Z) => "Altitude ASL [m]".to_string(),
            Self::PositionWorldSpace(dim) => format!("World-space Position ({dim:?}) [m]"),
            Self::Latitude => "Latitude".to_string(),
            Self::Longitude => "Longitude".to_string(),
            Self::GroundAltitudeASL => "Ground Altitude ASL".to_string(),
            Self::ApogeeAltitudeASL => "Apogee ASL".to_string(),

            Self::RawAngularVelocity(id, dim) => format!("Angular Velocity ({id:?}, {dim:?}) [°/s]"),
            Self::RawAcceleration(id, dim) => format!("Acceleration ({id:?}, {dim:?}) [m/s²]"),
            Self::RawMagneticFluxDensity(id, dim) => format!("Magnetic Flux Density ({id:?}, {dim:?}) [µT]"),
            Self::RawBarometricAltitude(id) => format!("Barometric Altitude ({id:?}) [m]"),

            Self::Pressure(id) => format!("Pressure ({id:?}) [bar]"),

            Self::TrueElevation => "True Elevation [°]".to_string(),
            Self::TrueAzimuth => "True Azimuth [°]".to_string(),
            Self::KalmanStateCovariance(0, 0) => "X/Y Pos. Variance [m]".to_string(),
            Self::KalmanStateCovariance(2, 2) => "Altitude Variance [m]".to_string(),
            Self::KalmanStateCovariance(5, 5) => "Vertical Speed Variance [m/s]".to_string(),
            Self::KalmanMeasurementCovariance(0, 0) => "Barometer Variance [m]".to_string(),
            Self::KalmanMeasurementCovariance(1, 1) => "Accelerometer Variance [m/s²]".to_string(),
            Self::KalmanMeasurementCovariance(4, 4) => "GPS Variance [m]".to_string(),
            _ => format!("{:?}", self),
        })
    }
}
