use core::f32;
use nalgebra::{Point2, Rotation2, Scale2, Translation2};
use shared_types::ThrusterValveState;
use telemetry::Metric;

use crate::{backend::Backend, system_diagram_components::{core::{display_value::DisplayValue, flow_painter::{Line1D, Painter, Symbol}, fluids::FluidType}, math::transform::Transform, storage::storage_state::StorageState, valves::valve_state::ValveState}, widgets::system_diagram::SystemDiagram};

pub fn create_diagram(backend: &Backend) -> SystemDiagram {

    let tank_pressure = backend.current_value(Metric::Pressure(telemetry::PressureSensorId::AcsTank));
    let fill_level = (tank_pressure.unwrap_or_default() / 300f64) as f32;
    let thruster_valve_state:Option<ThrusterValveState> =  backend.current_enum(Metric::ThrusterValveState);
    let (accel_valve_state, decel_valve_state) = match thruster_valve_state {
        Some(state) => match state {
            ThrusterValveState::Closed => (ValveState::Disconnected, ValveState::Disconnected),
            ThrusterValveState::OpenAccel => (ValveState::Connected(None), ValveState::Disconnected),
            ThrusterValveState::OpenDecel => (ValveState::Disconnected, ValveState::Connected(None)),
            ThrusterValveState::OpenBoth => (ValveState::Connected(None), ValveState::Connected(None)),
        }
        None => (ValveState::Disconnected, ValveState::Disconnected),
    };
    SystemDiagram::new(
        vec![
            Symbol::new(Painter::Tank(StorageState::new(FluidType::CompressedAir, fill_level)), Transform::new(Rotation2::identity(), Scale2::new(0.2, 0.6), Translation2::new(0.25, 0.5)), vec![DisplayValue::from_metric(Metric::Pressure(telemetry::PressureSensorId::AcsTank), backend)]),
            Symbol::new(Painter::Missing, Transform::new(Rotation2::identity(), Scale2::new(0.1, 0.1), Translation2::new(0.55, 0.5)), vec![DisplayValue::from_metric(Metric::Pressure(telemetry::PressureSensorId::AcsPostRegulator), backend)]),
            Symbol::new(Painter::GenericValve(decel_valve_state), Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.1, 0.05), Translation2::new(0.8, 0.4)), vec![]),
            Symbol::new(Painter::GenericValve(accel_valve_state), Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.1, 0.05), Translation2::new(0.8, 0.6)), vec![]),
            Symbol::new(Painter::Missing, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.1, 0.05), Translation2::new(0.8, 0.2)), vec![]),
            Symbol::new(Painter::Missing, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.1, 0.05), Translation2::new(0.8, 0.8)), vec![]),
        ],
        vec![
            Line1D::new(vec![Point2::new(0.35, 0.5), Point2::new(0.5, 0.5)]),
            Line1D::new(vec![Point2::new(0.6, 0.5), Point2::new(0.8, 0.5)]),
            Line1D::new(vec![Point2::new(0.8, 0.5), Point2::new(0.8, 0.45)]),
            Line1D::new(vec![Point2::new(0.8, 0.5), Point2::new(0.8, 0.55)]),
            Line1D::new(vec![Point2::new(0.8, 0.35), Point2::new(0.8, 0.25)]),
            Line1D::new(vec![Point2::new(0.8, 0.65), Point2::new(0.8, 0.75)]),
        ]
    )
}