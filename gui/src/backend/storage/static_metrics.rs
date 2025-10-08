use core::marker::PhantomData;

use crate::backend::storage::{
    constant::Constant, event_series::EventSeries, sample_series::SampleSeries, store::DataType,
    storeable_value::StorableValue,
};

pub trait MetricTrait: Default {
    type Value: StorableValue + PartialOrd;
    type DataType: DataType;
    fn metric() -> telemetry::Metric;
}

macro_rules! make_static_metric_with_storage_type {
    ($name:ident, $value:ty, $datatype:ty) => {
        #[derive(Default)]
        pub struct $name {}
        impl MetricTrait for $name {
            type Value = $value;
            type DataType = $datatype;

            fn metric() -> telemetry::Metric {
                telemetry::Metric::$name
            }
        }
    };
}

macro_rules! make_static_local_metric_with_storage_type {
    ($name:ident, $value:ty, $datatype:ty) => {
        #[derive(Default)]
        pub struct $name {}
        impl MetricTrait for $name {
            type Value = $value;
            type DataType = $datatype;

            fn metric() -> telemetry::Metric {
                telemetry::Metric::LocalMetric(telemetry::LocalMetric::$name)
            }
        }
    };
}

make_static_metric_with_storage_type!(FlightMode, shared_types::telemetry::FlightMode, SampleSeries);
make_static_metric_with_storage_type!(TransmitPower, shared_types::telemetry::TransmitPower, SampleSeries);
make_static_metric_with_storage_type!(AcsMode, shared_types::telemetry::AcsMode, SampleSeries);
make_static_metric_with_storage_type!(ThrusterValveState, shared_types::telemetry::ThrusterValveState, SampleSeries);
//ValveState(ValveId) - see below
//Orientation(usize) - see below
make_static_metric_with_storage_type!(Elevation, f64, SampleSeries);
make_static_metric_with_storage_type!(Azimuth, f64, SampleSeries);
//AccelerationWorldSpace(Dim) - see below
//VelocityWorldSpace(Dim) - see below
//PositionWorldSpace(Dim) - see below
make_static_metric_with_storage_type!(Latitude, f64, SampleSeries);
make_static_metric_with_storage_type!(Longitude, f64, SampleSeries);
make_static_metric_with_storage_type!(GroundAltitudeASL, f64, SampleSeries);
make_static_metric_with_storage_type!(ApogeeAltitudeASL, f64, SampleSeries);
make_static_metric_with_storage_type!(GroundSpeed, f64, SampleSeries);
//KalmanStateCovariance(usize, usize) - see below
//KalmanMeasurementCovariance(usize, usize) - see below
//RawAngularVelocity(GyroscopeId, Dim) - see below
//RawAcceleration(AccelerometerId, Dim) - see below
//RawMagneticFluxDensity(MagnetometerId) - see below
//RawBarometricAltitude(BarometerId) - see below
make_static_metric_with_storage_type!(GpsFix, shared_types::telemetry::GPSFixType, SampleSeries);
make_static_metric_with_storage_type!(GpsLatitude, f64, SampleSeries);
make_static_metric_with_storage_type!(GpsLongitude, f64, SampleSeries);
make_static_metric_with_storage_type!(GpsAltitude, f64, SampleSeries);
make_static_metric_with_storage_type!(GpsHdop, f64, SampleSeries);
make_static_metric_with_storage_type!(GpsSatellites, f64, SampleSeries);
//Pressure(PressureSensorId) - see below
//Temperature(TemperatureSensorId) - see below
//BatteryVoltage(BatteryId) - see below
//BatteryCurrent(BatteryId) - see below
//BatteryChargerState(BatteryId) - see below
make_static_metric_with_storage_type!(SupplyVoltage, f64, SampleSeries);
make_static_metric_with_storage_type!(SupplyCurrent, f64, SampleSeries);
make_static_metric_with_storage_type!(RecoveryCurrent, f64, SampleSeries);
make_static_metric_with_storage_type!(CpuUtilization, f64, SampleSeries);
make_static_metric_with_storage_type!(FlashPointer, f64, SampleSeries);
make_static_metric_with_storage_type!(UplinkRssi, f64, SampleSeries);
make_static_metric_with_storage_type!(UplinkSnr, f64, SampleSeries);
make_static_metric_with_storage_type!(DownlinkRssi, f64, SampleSeries);
make_static_metric_with_storage_type!(DownlinkSnr, f64, SampleSeries);
//TrueOrientation(usize) - see below
make_static_metric_with_storage_type!(TrueElevation, f64, SampleSeries);
make_static_metric_with_storage_type!(TrueAzimuth, f64, SampleSeries);
//TrueAccelerationWorldSpace(Dim) - see below
//TrueVelocityWorldSpace(Dim) - see below
//TruePositionWorldSpace(Dim) - see below
make_static_metric_with_storage_type!(TrueVehicleMass, f64, SampleSeries);
make_static_metric_with_storage_type!(TrueMotorMass, f64, SampleSeries);
make_static_metric_with_storage_type!(TrueThrusterPropellantMass, f64, SampleSeries);
//TrueDrag(Dim) - see below
//TrueThrust(Dim) - see below
make_static_metric_with_storage_type!(ApogeeError, f64, SampleSeries);
make_static_metric_with_storage_type!(ProcedureStep, shared_types::ProcedureStep, SampleSeries);

// ------------- Local Nested Metrics -------------

make_static_local_metric_with_storage_type!(
    N2BottleValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OBottleValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2ReleaseValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OReleaseValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2QuickDisconnect,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OQuickDisconnect,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2PurgeValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2PressureRegulator,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OVentValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OBurstDisc,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OFillAndDumpValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    N2OMainValve,
    crate::backend::storage::storeable_value::ValveState,
    EventSeries
);
make_static_local_metric_with_storage_type!(MaxPressureN2Tank, f64, Constant);
make_static_local_metric_with_storage_type!(
    HyacinthNominalState,
    crate::widgets::time_line::HyacinthNominalState,
    EventSeries
);
make_static_local_metric_with_storage_type!(
    HyacinthAnomalousState,
    Option<crate::widgets::time_line::HyacinthAnomalousState>,
    EventSeries
);
// ------------- Other Nested Metrics (currently do not work via macro) -------------

#[derive(Default)]
pub struct ValveState<Valve: ValveId> {
    valve_id: PhantomData<Valve>,
}
impl<Valve: ValveId + Default> MetricTrait for ValveState<Valve> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::ValveState(Valve::id())
    }
}

#[derive(Default)]
pub struct Orientation<const N: usize> {}
impl<const N: usize> MetricTrait for Orientation<N>
where
    Const<N>: InRange0To4Exclusive,
{
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::Orientation(N)
    }
}

#[derive(Default)]
pub struct AccelerationWorldSpace<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for AccelerationWorldSpace<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::AccelerationWorldSpace(D::dim())
    }
}

#[derive(Default)]
pub struct VelocityWorldSpace<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for VelocityWorldSpace<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::VelocityWorldSpace(D::dim())
    }
}

#[derive(Default)]
pub struct PositionWorldSpace<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for PositionWorldSpace<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::PositionWorldSpace(D::dim())
    }
}

#[derive(Default)]
pub struct KalmanStateCovariance<const N: usize, const M: usize> {} //TODO Hans: Limit Range of N and M
impl<const N: usize, const M: usize> MetricTrait for KalmanStateCovariance<N, M>
where
    Const<N>: InRange0To6Exclusive,
    Const<M>: InRange0To6Exclusive,
{
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::KalmanStateCovariance(N, M)
    }
}

#[derive(Default)]
pub struct KalmanMeasurementCovariance<const N: usize, const M: usize> {} //TODO Hans: Limit Range of N and M
impl<const N: usize, const M: usize> MetricTrait for KalmanMeasurementCovariance<N, M>
where
    Const<N>: InRange0To6Exclusive,
    Const<M>: InRange0To6Exclusive,
{
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::KalmanMeasurementCovariance(N, M)
    }
}

#[derive(Default)]
pub struct RawAngularVelocity<Gyroscope: GyroscopeId, D: Dim3> {
    gyroscope_id: PhantomData<Gyroscope>,
    dim: PhantomData<D>,
}
impl<Gyroscope: GyroscopeId + Default, D: Dim3 + Default> MetricTrait for RawAngularVelocity<Gyroscope, D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::RawAngularVelocity(Gyroscope::id(), D::dim())
    }
}

#[derive(Default)]
pub struct RawAcceleration<Accelerometer: AccelerometerId, D: Dim3> {
    accelerometer_id: PhantomData<Accelerometer>,
    dim: PhantomData<D>,
}
impl<Accelerometer: AccelerometerId + Default, D: Dim3 + Default> MetricTrait for RawAcceleration<Accelerometer, D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::RawAcceleration(Accelerometer::id(), D::dim())
    }
}

#[derive(Default)]
pub struct RawMagneticFluxDensity<Magnetometer: MagnetometerId, D: Dim3> {
    magnetometer_id: PhantomData<Magnetometer>,
    dim: PhantomData<D>,
}
impl<Magnetometer: MagnetometerId + Default, D: Dim3 + Default> MetricTrait
    for RawMagneticFluxDensity<Magnetometer, D>
{
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::RawMagneticFluxDensity(Magnetometer::id(), D::dim())
    }
}

#[derive(Default)]
pub struct RawBarometricAltitude<Barometer: BarometerId> {
    barometer_id: PhantomData<Barometer>,
}
impl<Barometer: BarometerId + Default> MetricTrait for RawBarometricAltitude<Barometer> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::RawBarometricAltitude(Barometer::id())
    }
}

#[derive(Default)]
pub struct Pressure<Sensor: PressureSensorId> {
    dim: PhantomData<Sensor>,
}
impl<Sensor: PressureSensorId + Default> MetricTrait for Pressure<Sensor> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::Pressure(Sensor::id())
    }
}

#[derive(Default)]
pub struct Temperature<Sensor: TemperatureSensorId> {
    dim: PhantomData<Sensor>,
}
impl<Sensor: TemperatureSensorId + Default> MetricTrait for Temperature<Sensor> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::Temperature(Sensor::id())
    }
}

#[derive(Default)]
pub struct BatteryVoltage<Battery: BatteryId> {
    dim: PhantomData<Battery>,
}
impl<Battery: BatteryId + Default> MetricTrait for BatteryVoltage<Battery> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::BatteryVoltage(Battery::id())
    }
}

#[derive(Default)]
pub struct BatteryCurrent<Battery: BatteryId> {
    dim: PhantomData<Battery>,
}
impl<Battery: BatteryId + Default> MetricTrait for BatteryCurrent<Battery> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::BatteryCurrent(Battery::id())
    }
}

#[derive(Default)]
pub struct BatteryChargerState<Battery: BatteryId> {
    dim: PhantomData<Battery>,
}
impl<Battery: BatteryId + Default> MetricTrait for BatteryChargerState<Battery> {
    type Value = shared_types::telemetry::BatteryChargerState;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::BatteryChargerState(Battery::id())
    }
}

#[derive(Default)]
pub struct TrueOrientation<const N: usize> {}
impl<const N: usize> MetricTrait for TrueOrientation<N>
where
    Const<N>: InRange0To4Exclusive,
{
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::TrueOrientation(N)
    }
}

#[derive(Default)]
pub struct TrueAccelerationWorldSpace<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for TrueAccelerationWorldSpace<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::TrueAccelerationWorldSpace(D::dim())
    }
}

#[derive(Default)]
pub struct TrueVelocityWorldSpace<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for TrueVelocityWorldSpace<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::TrueVelocityWorldSpace(D::dim())
    }
}

#[derive(Default)]
pub struct TruePositionWorldSpace<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for TruePositionWorldSpace<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::TruePositionWorldSpace(D::dim())
    }
}

#[derive(Default)]
pub struct TrueDrag<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for TrueDrag<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::TrueDrag(D::dim())
    }
}

#[derive(Default)]
pub struct TrueThrust<D: Dim3> {
    dim: PhantomData<D>,
}
impl<D: Dim3 + Default> MetricTrait for TrueThrust<D> {
    type Value = f64;
    type DataType = SampleSeries;

    fn metric() -> telemetry::Metric {
        telemetry::Metric::TrueThrust(D::dim())
    }
}

// ------------- Id definitions and so on for nested metrics -------------

struct Const<const N: usize> {}

trait InRange0To4Exclusive {}
impl InRange0To4Exclusive for Const<0> {}
impl InRange0To4Exclusive for Const<1> {}
impl InRange0To4Exclusive for Const<2> {}
impl InRange0To4Exclusive for Const<3> {}

trait InRange0To6Exclusive {}
impl InRange0To6Exclusive for Const<0> {}
impl InRange0To6Exclusive for Const<1> {}
impl InRange0To6Exclusive for Const<2> {}
impl InRange0To6Exclusive for Const<3> {}
impl InRange0To6Exclusive for Const<4> {}
impl InRange0To6Exclusive for Const<5> {}

#[derive(Default)]
pub struct X {}
#[derive(Default)]
pub struct Y {}
#[derive(Default)]
pub struct Z {}

pub trait Dim3 {
    fn dim() -> telemetry::Dim;
}
impl Dim3 for X {
    fn dim() -> telemetry::Dim {
        telemetry::Dim::X
    }
}
impl Dim3 for Y {
    fn dim() -> telemetry::Dim {
        telemetry::Dim::Y
    }
}
impl Dim3 for Z {
    fn dim() -> telemetry::Dim {
        telemetry::Dim::Z
    }
}

#[derive(Default)]
pub struct LSM6DSR {}
#[derive(Default)]
pub struct ICM42670P {}
#[derive(Default)]
pub struct ICM42688P {}

pub trait GyroscopeId {
    fn id() -> telemetry::GyroscopeId;
}
impl GyroscopeId for LSM6DSR {
    fn id() -> telemetry::GyroscopeId {
        return telemetry::GyroscopeId::LSM6DSR;
    }
}
impl GyroscopeId for ICM42670P {
    fn id() -> telemetry::GyroscopeId {
        return telemetry::GyroscopeId::ICM42670P;
    }
}
impl GyroscopeId for ICM42688P {
    fn id() -> telemetry::GyroscopeId {
        return telemetry::GyroscopeId::ICM42688P;
    }
}

#[derive(Default)]
pub struct H3LIS331 {}

pub trait AccelerometerId {
    fn id() -> telemetry::AccelerometerId;
}
impl AccelerometerId for LSM6DSR {
    fn id() -> telemetry::AccelerometerId {
        return telemetry::AccelerometerId::LSM6DSR;
    }
}
impl AccelerometerId for ICM42670P {
    fn id() -> telemetry::AccelerometerId {
        return telemetry::AccelerometerId::ICM42670P;
    }
}
impl AccelerometerId for ICM42688P {
    fn id() -> telemetry::AccelerometerId {
        return telemetry::AccelerometerId::ICM42688P;
    }
}
impl AccelerometerId for H3LIS331 {
    fn id() -> telemetry::AccelerometerId {
        return telemetry::AccelerometerId::H3LIS331;
    }
}

#[derive(Default)]
pub struct LIS3MDL {}

pub trait MagnetometerId {
    fn id() -> telemetry::MagnetometerId;
}
impl MagnetometerId for LIS3MDL {
    fn id() -> telemetry::MagnetometerId {
        return telemetry::MagnetometerId::LIS3MDL;
    }
}

#[derive(Default)]
pub struct MS5611 {}
#[derive(Default)]
pub struct LPS22 {}
#[derive(Default)]
pub struct BMP580 {}

pub trait BarometerId {
    fn id() -> telemetry::BarometerId;
}
impl BarometerId for MS5611 {
    fn id() -> telemetry::BarometerId {
        return telemetry::BarometerId::MS5611;
    }
}
impl BarometerId for LPS22 {
    fn id() -> telemetry::BarometerId {
        return telemetry::BarometerId::LPS22;
    }
}
impl BarometerId for BMP580 {
    fn id() -> telemetry::BarometerId {
        return telemetry::BarometerId::BMP580;
    }
}

#[derive(Default)]
pub struct BatteryAvionics {}
#[derive(Default)]
pub struct BatteryAcs {}
#[derive(Default)]
pub struct BatteryRecovery {}
#[derive(Default)]
pub struct BatteryPayload {}

pub trait BatteryId {
    fn id() -> telemetry::BatteryId;
}
impl BatteryId for BatteryAvionics {
    fn id() -> telemetry::BatteryId {
        return telemetry::BatteryId::Avionics;
    }
}
impl BatteryId for BatteryAcs {
    fn id() -> telemetry::BatteryId {
        return telemetry::BatteryId::Acs;
    }
}
impl BatteryId for BatteryRecovery {
    fn id() -> telemetry::BatteryId {
        return telemetry::BatteryId::Recovery;
    }
}
impl BatteryId for BatteryPayload {
    fn id() -> telemetry::BatteryId {
        return telemetry::BatteryId::Payload;
    }
}

#[derive(Default)]
pub struct PressureSensorFlightComputer<B: BarometerId> {
    barometer_id: PhantomData<B>,
}
#[derive(Default)]
pub struct PressureSensorAcsTank {}
#[derive(Default)]
pub struct PressureSensorAcsPostRegulator {}
#[derive(Default)]
pub struct PressureSensorAcsValveAccel {}
#[derive(Default)]
pub struct PressureSensorAcsValveDecel {}
#[derive(Default)]
pub struct PressureSensorRecoveryChamberDrogue {}
#[derive(Default)]
pub struct PressureSensorRecoveryChamberMain {}
#[derive(Default)]
pub struct PressureSensorMainRelease {}
#[derive(Default)]
pub struct PressureSensorCombustionChamber {}
#[derive(Default)]
pub struct PressureSensorOxidizerTank {}
#[derive(Default)]
pub struct PressureSensorNitrogenTank {}

pub trait PressureSensorId {
    fn id() -> telemetry::PressureSensorId;
}
impl<B: BarometerId> PressureSensorId for PressureSensorFlightComputer<B> {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::FlightComputer(B::id());
    }
}
impl PressureSensorId for PressureSensorAcsTank {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::AcsTank;
    }
}
impl PressureSensorId for PressureSensorAcsPostRegulator {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::AcsPostRegulator;
    }
}
impl PressureSensorId for PressureSensorAcsValveAccel {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::AcsValveAccel;
    }
}
impl PressureSensorId for PressureSensorAcsValveDecel {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::AcsValveDecel;
    }
}
impl PressureSensorId for PressureSensorRecoveryChamberDrogue {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::RecoveryChamberDrogue;
    }
}
impl PressureSensorId for PressureSensorRecoveryChamberMain {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::RecoveryChamberMain;
    }
}
impl PressureSensorId for PressureSensorMainRelease {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::MainRelease;
    }
}
impl PressureSensorId for PressureSensorCombustionChamber {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::CombustionChamber;
    }
}
impl PressureSensorId for PressureSensorOxidizerTank {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::OxidizerTank;
    }
}
impl PressureSensorId for PressureSensorNitrogenTank {
    fn id() -> telemetry::PressureSensorId {
        return telemetry::PressureSensorId::NitrogenTank;
    }
}

#[derive(Default)]
pub struct TemperatureSensorBarometer<B: BarometerId> {
    barometer_id: PhantomData<B>,
}
#[derive(Default)]
pub struct TemperatureSensorBattery<B: BatteryId> {
    barometer_id: PhantomData<B>,
}
#[derive(Default)]
pub struct TemperatureSensorAcs {}
#[derive(Default)]
pub struct TemperatureSensorRecovery {}
#[derive(Default)]
pub struct TemperatureSensorPayload {}
#[derive(Default)]
pub struct TemperatureSensorOxidizerTank {}

pub trait TemperatureSensorId {
    fn id() -> telemetry::TemperatureSensorId;
}
impl<B: BarometerId> TemperatureSensorId for TemperatureSensorBarometer<B> {
    fn id() -> telemetry::TemperatureSensorId {
        return telemetry::TemperatureSensorId::Barometer(B::id());
    }
}
impl<B: BatteryId> TemperatureSensorId for TemperatureSensorBattery<B> {
    fn id() -> telemetry::TemperatureSensorId {
        return telemetry::TemperatureSensorId::Battery(B::id());
    }
}
impl TemperatureSensorId for TemperatureSensorAcs {
    fn id() -> telemetry::TemperatureSensorId {
        return telemetry::TemperatureSensorId::Acs;
    }
}
impl TemperatureSensorId for TemperatureSensorRecovery {
    fn id() -> telemetry::TemperatureSensorId {
        return telemetry::TemperatureSensorId::Recovery;
    }
}
impl TemperatureSensorId for TemperatureSensorPayload {
    fn id() -> telemetry::TemperatureSensorId {
        return telemetry::TemperatureSensorId::Payload;
    }
}
impl TemperatureSensorId for TemperatureSensorOxidizerTank {
    fn id() -> telemetry::TemperatureSensorId {
        return telemetry::TemperatureSensorId::OxidizerTank;
    }
}

#[derive(Default)]
pub struct PressureRegulator {}
#[derive(Default)]
pub struct MainValve {}
#[derive(Default)]
pub struct VentValve {}
#[derive(Default)]
pub struct FillAndDumpValve {}

pub trait ValveId {
    fn id() -> telemetry::ValveId;
}
impl ValveId for PressureRegulator {
    fn id() -> telemetry::ValveId {
        return telemetry::ValveId::PressureRegulator;
    }
}
impl ValveId for MainValve {
    fn id() -> telemetry::ValveId {
        return telemetry::ValveId::MainValve;
    }
}
impl ValveId for VentValve {
    fn id() -> telemetry::ValveId {
        return telemetry::ValveId::VentValve;
    }
}
impl ValveId for FillAndDumpValve {
    fn id() -> telemetry::ValveId {
        return telemetry::ValveId::FillAndDumpValve;
    }
}

/// Usage:
/// call_static_metric!(func, <metric, generic_arg1, ..., >, arg1, arg2, ...);
/// where `func` is generic over `T: MetricTrait`.
/// The arguments and return type must be the same for every metric
/// The trailing comma inside the generic arguments is required!
#[macro_export]
macro_rules! call_static_metric {
    // $func is the generic function name
    // $metric is the runtime enum value
    // $( $args:expr ),* are the arguments to forward
    ($($func:ident)::+, < $metric:expr $(, $generics:ty)* , >, $($args:expr),*) => {{
        match $metric {
            telemetry::Metric::FlightMode => $($func)::+::<$crate::storage::static_metrics::FlightMode $(, $generics)*>($($args),*),
            telemetry::Metric::TransmitPower => $($func)::+::<$crate::storage::static_metrics::TransmitPower $(, $generics)*>($($args),*),
            telemetry::Metric::AcsMode => $($func)::+::<$crate::storage::static_metrics::AcsMode $(, $generics)*>($($args),*),
            telemetry::Metric::ThrusterValveState => $($func)::+::<$crate::storage::static_metrics::ThrusterValveState $(, $generics)*>($($args),*),
            telemetry::Metric::ValveState(valve_id) =>
                match valve_id {
                    telemetry::ValveId::PressureRegulator => $($func)::+::<$crate::storage::static_metrics::ValveState<$crate::storage::static_metrics::PressureRegulator> $(, $generics)*>($($args),*),
                    telemetry::ValveId::MainValve => $($func)::+::<$crate::storage::static_metrics::ValveState<$crate::storage::static_metrics::MainValve> $(, $generics)*>($($args),*),
                    telemetry::ValveId::VentValve => $($func)::+::<$crate::storage::static_metrics::ValveState<$crate::storage::static_metrics::VentValve> $(, $generics)*>($($args),*),
                    telemetry::ValveId::FillAndDumpValve => $($func)::+::<$crate::storage::static_metrics::ValveState<$crate::storage::static_metrics::FillAndDumpValve> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::Orientation(n) =>
                match n {
                    0 => $($func)::+::<$crate::storage::static_metrics::Orientation<0> $(, $generics)*>($($args),*),
                    1 => $($func)::+::<$crate::storage::static_metrics::Orientation<1> $(, $generics)*>($($args),*),
                    2 => $($func)::+::<$crate::storage::static_metrics::Orientation<2> $(, $generics)*>($($args),*),
                    3 => $($func)::+::<$crate::storage::static_metrics::Orientation<3> $(, $generics)*>($($args),*),
                    _ => unreachable!(),
                },
            telemetry::Metric::Elevation => $($func)::+::<$crate::storage::static_metrics::Elevation $(, $generics)*>($($args),*),
            telemetry::Metric::Azimuth => $($func)::+::<$crate::storage::static_metrics::Azimuth $(, $generics)*>($($args),*),
            telemetry::Metric::AccelerationWorldSpace(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::AccelerationWorldSpace<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::AccelerationWorldSpace<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::AccelerationWorldSpace<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::VelocityWorldSpace(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::VelocityWorldSpace<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::VelocityWorldSpace<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::VelocityWorldSpace<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::PositionWorldSpace(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::PositionWorldSpace<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::PositionWorldSpace<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::PositionWorldSpace<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::Latitude => $($func)::+::<$crate::storage::static_metrics::Latitude $(, $generics)*>($($args),*),
            telemetry::Metric::Longitude => $($func)::+::<$crate::storage::static_metrics::Longitude $(, $generics)*>($($args),*),
            telemetry::Metric::GroundAltitudeASL => $($func)::+::<$crate::storage::static_metrics::GroundAltitudeASL $(, $generics)*>($($args),*),
            telemetry::Metric::ApogeeAltitudeASL => $($func)::+::<$crate::storage::static_metrics::ApogeeAltitudeASL $(, $generics)*>($($args),*),
            telemetry::Metric::GroundSpeed => $($func)::+::<$crate::storage::static_metrics::GroundSpeed $(, $generics)*>($($args),*),
            telemetry::Metric::KalmanStateCovariance(n, m) =>
                match n {
                    0 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<0, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<0, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<0, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<0, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<0, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<0, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    1 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<1, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<1, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<1, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<1, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<1, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<1, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    2 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<2, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<2, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<2, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<2, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<2, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<2, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    3 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<3, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<3, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<3, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<3, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<3, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<3, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    4 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<4, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<4, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<4, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<4, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<4, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<4, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    5 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<5, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<5, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<5, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<5, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<5, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanStateCovariance<5, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    _ => unreachable!(),
                },
            telemetry::Metric::KalmanMeasurementCovariance(n, m) =>
                match n {
                    0 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<0, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<0, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<0, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<0, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<0, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<0, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    1 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<1, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<1, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<1, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<1, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<1, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<1, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    2 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<2, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<2, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<2, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<2, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<2, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<2, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    3 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<3, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<3, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<3, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<3, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<3, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<3, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    4 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<4, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<4, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<4, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<4, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<4, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<4, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    5 =>
                        match m {
                            0 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<5, 0> $(, $generics)*>($($args),*),
                            1 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<5, 1> $(, $generics)*>($($args),*),
                            2 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<5, 2> $(, $generics)*>($($args),*),
                            3 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<5, 3> $(, $generics)*>($($args),*),
                            4 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<5, 4> $(, $generics)*>($($args),*),
                            5 => $($func)::+::<$crate::storage::static_metrics::KalmanMeasurementCovariance<5, 5> $(, $generics)*>($($args),*),
                            _ => unreachable!(),
                        },
                    _ => unreachable!(),
                },
            telemetry::Metric::RawAngularVelocity(gyroscope_id, dim) =>
                match gyroscope_id {
                    telemetry::GyroscopeId::LSM6DSR =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::LSM6DSR, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::LSM6DSR, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::LSM6DSR, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        },
                    telemetry::GyroscopeId::ICM42670P =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::ICM42670P, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::ICM42670P, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::ICM42670P, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        }
                    telemetry::GyroscopeId::ICM42688P =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::ICM42688P, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::ICM42688P, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAngularVelocity<$crate::storage::static_metrics::ICM42688P, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        }
                },
            telemetry::Metric::RawAcceleration(accelerometer_id, dim) =>
                match accelerometer_id {
                    telemetry::AccelerometerId::LSM6DSR =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::LSM6DSR, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::LSM6DSR, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::LSM6DSR, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        }
                    telemetry::AccelerometerId::ICM42670P =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::ICM42670P, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::ICM42670P, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::ICM42670P, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        }
                    telemetry::AccelerometerId::ICM42688P =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::ICM42688P, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::ICM42688P, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::ICM42688P, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        }
                    telemetry::AccelerometerId::H3LIS331 =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::H3LIS331, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::H3LIS331, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawAcceleration<$crate::storage::static_metrics::H3LIS331, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        }
                },
            telemetry::Metric::RawMagneticFluxDensity(magnetometer_id, dim) =>
                match magnetometer_id {
                    telemetry::MagnetometerId::LIS3MDL =>
                        match dim {
                            telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::RawMagneticFluxDensity<$crate::storage::static_metrics::LIS3MDL, $crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                            telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::RawMagneticFluxDensity<$crate::storage::static_metrics::LIS3MDL, $crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                            telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::RawMagneticFluxDensity<$crate::storage::static_metrics::LIS3MDL, $crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                        },
                },
            telemetry::Metric::RawBarometricAltitude(barometer_id) =>
                match barometer_id {
                    telemetry::BarometerId::MS5611 => $($func)::+::<$crate::storage::static_metrics::RawBarometricAltitude<$crate::storage::static_metrics::MS5611> $(, $generics)*>($($args),*),
                    telemetry::BarometerId::LPS22 => $($func)::+::<$crate::storage::static_metrics::RawBarometricAltitude<$crate::storage::static_metrics::LPS22> $(, $generics)*>($($args),*),
                    telemetry::BarometerId::BMP580 => $($func)::+::<$crate::storage::static_metrics::RawBarometricAltitude<$crate::storage::static_metrics::BMP580> $(, $generics)*>($($args),*),
                },
            telemetry::Metric::GpsFix => $($func)::+::<$crate::storage::static_metrics::GpsFix $(, $generics)*>($($args),*),
            telemetry::Metric::GpsLatitude => $($func)::+::<$crate::storage::static_metrics::GpsLatitude $(, $generics)*>($($args),*),
            telemetry::Metric::GpsLongitude => $($func)::+::<$crate::storage::static_metrics::GpsLongitude $(, $generics)*>($($args),*),
            telemetry::Metric::GpsAltitude => $($func)::+::<$crate::storage::static_metrics::GpsAltitude $(, $generics)*>($($args),*),
            telemetry::Metric::GpsHdop => $($func)::+::<$crate::storage::static_metrics::GpsHdop $(, $generics)*>($($args),*),
            telemetry::Metric::GpsSatellites => $($func)::+::<$crate::storage::static_metrics::GpsSatellites $(, $generics)*>($($args),*),
            telemetry::Metric::Pressure(pressure_sensor_id) =>
                match pressure_sensor_id {
                    telemetry::PressureSensorId::FlightComputer(barometer_id) =>
                        match barometer_id {
                            telemetry::BarometerId::MS5611 => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorFlightComputer<$crate::storage::static_metrics::MS5611>> $(, $generics)*>($($args),*),
                            telemetry::BarometerId::LPS22 => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorFlightComputer<$crate::storage::static_metrics::LPS22>> $(, $generics)*>($($args),*),
                            telemetry::BarometerId::BMP580 => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorFlightComputer<$crate::storage::static_metrics::BMP580>> $(, $generics)*>($($args),*),
                        },
                    telemetry::PressureSensorId::AcsTank => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorAcsTank> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::AcsPostRegulator => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorAcsPostRegulator> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::AcsValveAccel => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorAcsValveAccel> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::AcsValveDecel => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorAcsValveDecel> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::RecoveryChamberDrogue => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorRecoveryChamberDrogue> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::RecoveryChamberMain => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorRecoveryChamberMain> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::MainRelease => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorMainRelease> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::CombustionChamber => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorCombustionChamber> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::OxidizerTank => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorOxidizerTank> $(, $generics)*>($($args),*),
                    telemetry::PressureSensorId::NitrogenTank => $($func)::+::<$crate::storage::static_metrics::Pressure<$crate::storage::static_metrics::PressureSensorNitrogenTank> $(, $generics)*>($($args),*),
                },
            telemetry::Metric::Temperature(temperature_sensor_id) =>
                match temperature_sensor_id {
                    telemetry::TemperatureSensorId::Barometer(barometer_id) =>
                        match barometer_id {
                            telemetry::BarometerId::MS5611 => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBarometer<$crate::storage::static_metrics::MS5611>> $(, $generics)*>($($args),*),
                            telemetry::BarometerId::LPS22 => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBarometer<$crate::storage::static_metrics::LPS22>> $(, $generics)*>($($args),*),
                            telemetry::BarometerId::BMP580 => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBarometer<$crate::storage::static_metrics::BMP580>> $(, $generics)*>($($args),*),
                        },
                    telemetry::TemperatureSensorId::Battery(battery_id) =>
                        match battery_id {
                            telemetry::BatteryId::Avionics => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBattery<$crate::storage::static_metrics::BatteryAvionics>> $(, $generics)*>($($args),*),
                            telemetry::BatteryId::Acs => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBattery<$crate::storage::static_metrics::BatteryAcs>> $(, $generics)*>($($args),*),
                            telemetry::BatteryId::Recovery => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBattery<$crate::storage::static_metrics::BatteryRecovery>> $(, $generics)*>($($args),*),
                            telemetry::BatteryId::Payload => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorBattery<$crate::storage::static_metrics::BatteryPayload>> $(, $generics)*>($($args),*),
                            _ => unreachable!()
                        },
                    telemetry::TemperatureSensorId::Acs => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorAcs> $(, $generics)*>($($args),*),
                    telemetry::TemperatureSensorId::Recovery => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorRecovery> $(, $generics)*>($($args),*),
                    telemetry::TemperatureSensorId::Payload => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorPayload> $(, $generics)*>($($args),*),
                    telemetry::TemperatureSensorId::OxidizerTank => $($func)::+::<$crate::storage::static_metrics::Temperature<$crate::storage::static_metrics::TemperatureSensorOxidizerTank> $(, $generics)*>($($args),*),
                },
            telemetry::Metric::BatteryVoltage(battery_id) =>
                match battery_id {
                    telemetry::BatteryId::Avionics => $($func)::+::<$crate::storage::static_metrics::BatteryVoltage<$crate::storage::static_metrics::BatteryAvionics> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Acs => $($func)::+::<$crate::storage::static_metrics::BatteryVoltage<$crate::storage::static_metrics::BatteryAcs> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Recovery => $($func)::+::<$crate::storage::static_metrics::BatteryVoltage<$crate::storage::static_metrics::BatteryRecovery> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Payload => $($func)::+::<$crate::storage::static_metrics::BatteryVoltage<$crate::storage::static_metrics::BatteryPayload> $(, $generics)*>($($args),*),
                    _ => unreachable!()
                },
            telemetry::Metric::BatteryCurrent(battery_id) =>
                match battery_id {
                    telemetry::BatteryId::Avionics => $($func)::+::<$crate::storage::static_metrics::BatteryCurrent<$crate::storage::static_metrics::BatteryAvionics> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Acs => $($func)::+::<$crate::storage::static_metrics::BatteryCurrent<$crate::storage::static_metrics::BatteryAcs> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Recovery => $($func)::+::<$crate::storage::static_metrics::BatteryCurrent<$crate::storage::static_metrics::BatteryRecovery> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Payload => $($func)::+::<$crate::storage::static_metrics::BatteryCurrent<$crate::storage::static_metrics::BatteryPayload> $(, $generics)*>($($args),*),
                    _ => unreachable!()
                },
            telemetry::Metric::BatteryChargerState(battery_id) =>
                match battery_id {
                    telemetry::BatteryId::Avionics => $($func)::+::<$crate::storage::static_metrics::BatteryChargerState<$crate::storage::static_metrics::BatteryAvionics> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Acs => $($func)::+::<$crate::storage::static_metrics::BatteryChargerState<$crate::storage::static_metrics::BatteryAcs> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Recovery => $($func)::+::<$crate::storage::static_metrics::BatteryChargerState<$crate::storage::static_metrics::BatteryRecovery> $(, $generics)*>($($args),*),
                    telemetry::BatteryId::Payload => $($func)::+::<$crate::storage::static_metrics::BatteryChargerState<$crate::storage::static_metrics::BatteryPayload> $(, $generics)*>($($args),*),
                    _ => unreachable!()
                },
            telemetry::Metric::SupplyVoltage => $($func)::+::<$crate::storage::static_metrics::SupplyVoltage $(, $generics)*>($($args),*),
            telemetry::Metric::SupplyCurrent => $($func)::+::<$crate::storage::static_metrics::SupplyCurrent $(, $generics)*>($($args),*),
            telemetry::Metric::RecoveryCurrent => $($func)::+::<$crate::storage::static_metrics::RecoveryCurrent $(, $generics)*>($($args),*),
            telemetry::Metric::CpuUtilization => $($func)::+::<$crate::storage::static_metrics::CpuUtilization $(, $generics)*>($($args),*),
            telemetry::Metric::FlashPointer => $($func)::+::<$crate::storage::static_metrics::FlashPointer $(, $generics)*>($($args),*),
            telemetry::Metric::UplinkRssi => $($func)::+::<$crate::storage::static_metrics::UplinkRssi $(, $generics)*>($($args),*),
            telemetry::Metric::UplinkSnr => $($func)::+::<$crate::storage::static_metrics::UplinkSnr $(, $generics)*>($($args),*),
            telemetry::Metric::DownlinkRssi => $($func)::+::<$crate::storage::static_metrics::DownlinkRssi $(, $generics)*>($($args),*),
            telemetry::Metric::DownlinkSnr => $($func)::+::<$crate::storage::static_metrics::DownlinkSnr $(, $generics)*>($($args),*),
            telemetry::Metric::TrueOrientation(n) =>
                match n {
                    0 => $($func)::+::<$crate::storage::static_metrics::TrueOrientation<0> $(, $generics)*>($($args),*),
                    1 => $($func)::+::<$crate::storage::static_metrics::TrueOrientation<1> $(, $generics)*>($($args),*),
                    2 => $($func)::+::<$crate::storage::static_metrics::TrueOrientation<2> $(, $generics)*>($($args),*),
                    3 => $($func)::+::<$crate::storage::static_metrics::TrueOrientation<3> $(, $generics)*>($($args),*),
                    _ => unreachable!(),
                },
            telemetry::Metric::TrueElevation => $($func)::+::<$crate::storage::static_metrics::TrueElevation $(, $generics)*>($($args),*),
            telemetry::Metric::TrueAzimuth => $($func)::+::<$crate::storage::static_metrics::TrueAzimuth $(, $generics)*>($($args),*),
            telemetry::Metric::TrueAccelerationWorldSpace(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::TrueAccelerationWorldSpace<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::TrueAccelerationWorldSpace<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::TrueAccelerationWorldSpace<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::TrueVelocityWorldSpace(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::TrueVelocityWorldSpace<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::TrueVelocityWorldSpace<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::TrueVelocityWorldSpace<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::TruePositionWorldSpace(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::TruePositionWorldSpace<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::TruePositionWorldSpace<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::TruePositionWorldSpace<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::TrueVehicleMass => $($func)::+::<$crate::storage::static_metrics::TrueVehicleMass $(, $generics)*>($($args),*),
            telemetry::Metric::TrueMotorMass => $($func)::+::<$crate::storage::static_metrics::TrueMotorMass $(, $generics)*>($($args),*),
            telemetry::Metric::TrueThrusterPropellantMass => $($func)::+::<$crate::storage::static_metrics::TrueThrusterPropellantMass $(, $generics)*>($($args),*),
            telemetry::Metric::TrueDrag(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::TrueDrag<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::TrueDrag<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::TrueDrag<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::TrueThrust(dim) =>
                match dim {
                    telemetry::Dim::X => $($func)::+::<$crate::storage::static_metrics::TrueThrust<$crate::storage::static_metrics::X> $(, $generics)*>($($args),*),
                    telemetry::Dim::Y => $($func)::+::<$crate::storage::static_metrics::TrueThrust<$crate::storage::static_metrics::Y> $(, $generics)*>($($args),*),
                    telemetry::Dim::Z => $($func)::+::<$crate::storage::static_metrics::TrueThrust<$crate::storage::static_metrics::Z> $(, $generics)*>($($args),*),
                }
            telemetry::Metric::ApogeeError => $($func)::+::<$crate::storage::static_metrics::ApogeeError $(, $generics)*>($($args),*),
            telemetry::Metric::ProcedureStep => $($func)::+::<$crate::storage::static_metrics::ProcedureStep $(, $generics)*>($($args),*),
            telemetry::Metric::LocalMetric(local_metric) =>
            match local_metric {
                telemetry::LocalMetric::HyacinthNominalState  => $($func)::+::<$crate::storage::static_metrics::HyacinthNominalState $(, $generics)*>($($args),*),
                telemetry::LocalMetric::HyacinthAnomalousState  => $($func)::+::<$crate::storage::static_metrics::HyacinthAnomalousState $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2BottleValve => $($func)::+::<$crate::storage::static_metrics::N2BottleValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OBottleValve => $($func)::+::<$crate::storage::static_metrics::N2OBottleValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2ReleaseValve => $($func)::+::<$crate::storage::static_metrics::N2ReleaseValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OReleaseValve => $($func)::+::<$crate::storage::static_metrics::N2OReleaseValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2QuickDisconnect => $($func)::+::<$crate::storage::static_metrics::N2QuickDisconnect $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OQuickDisconnect => $($func)::+::<$crate::storage::static_metrics::N2OQuickDisconnect $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2PurgeValve => $($func)::+::<$crate::storage::static_metrics::N2PurgeValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2PressureRegulator => $($func)::+::<$crate::storage::static_metrics::N2PressureRegulator $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OVentValve => $($func)::+::<$crate::storage::static_metrics::N2OVentValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OBurstDisc => $($func)::+::<$crate::storage::static_metrics::N2OBurstDisc $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OFillAndDumpValve => $($func)::+::<$crate::storage::static_metrics::N2OFillAndDumpValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::N2OMainValve => $($func)::+::<$crate::storage::static_metrics::N2OMainValve $(, $generics)*>($($args),*),
                telemetry::LocalMetric::MaxPressureN2Tank => $($func)::+::<$crate::storage::static_metrics::MaxPressureN2Tank $(, $generics)*>($($args),*),
            },
            _ => unreachable!()
        }
    }};
}
