use core::f32;
use std::sync::LazyLock;
use nalgebra::{Point2, Rotation2, Scale2, Translation2};
use shared_types::ThrusterValveState;
use telemetry::{Metric, PressureSensorId};

use crate::{backend::Backend, frontend::{constraints::{EqualsConstraint, SomeConstraint}, Frontend}, system_diagram_components::{core::{display_value::{DisplayValue, Justification, JustifiedValue, Value}, flow_painter::{Line1D, Painter, Symbol}, fluids::FluidType}, math::transform::Transform, storage::storage_state::StorageState, valves::valve_state::ValveState}, widgets::system_diagram::{Component, SystemDiagram}};

//TODO MOVE
static ACS_SYSTEM_DEFINITION: LazyLock<Vec<Component>> = LazyLock::new(|| 
    vec![
        Component::new(
            "Compressed Air Tank".to_string(),
            None,
            vec![
                DisplayValue::new("Max Pressure".to_string(), Some("bar".to_string()), JustifiedValue::new(Some(Value::F32(300f32)), Justification::Datasheet)),
                DisplayValue::new("Max Capacity".to_string(), Some("l".to_string()), JustifiedValue::new(Some(Value::F32(1.1)), Justification::Datasheet))
            ], 
            vec![
                Metric::Pressure(telemetry::PressureSensorId::AcsTank),
                Metric::Temperature(telemetry::TemperatureSensorId::Acs),
            ]
        ),
    ]
);

pub fn create_diagram<'a>(backend: &'a Backend, frontend: &'a mut Frontend) -> SystemDiagram<'a> {

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

    frontend.metric_monitor_mut().add_constraint(Box::new(SomeConstraint::new(Metric::Pressure(PressureSensorId::AcsTank), crate::frontend::constraints::ConstraintResult::WARNING)));
    frontend.metric_monitor_mut().add_constraint(Box::new(EqualsConstraint::new(Metric::ThrusterValveState, ThrusterValveState::OpenBoth, crate::frontend::constraints::ConstraintResult::DANGER)));

    SystemDiagram::new(
        vec![
            Symbol::new(Painter::Tank(StorageState::new(FluidType::CompressedAir, fill_level)), Transform::new(Rotation2::identity(), Scale2::new(0.2, 0.6), Translation2::new(0.25, 0.5)), Some(ACS_SYSTEM_DEFINITION[0].clone())),
            Symbol::new(Painter::Missing, Transform::new(Rotation2::identity(), Scale2::new(0.1, 0.1), Translation2::new(0.55, 0.5)), None/*vec![Metric::Pressure(telemetry::PressureSensorId::AcsPostRegulator)]*/),
            Symbol::new(Painter::GenericValve(decel_valve_state), Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.1, 0.05), Translation2::new(0.8, 0.4)), None),
            Symbol::new(Painter::GenericValve(accel_valve_state), Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.1, 0.05), Translation2::new(0.8, 0.6)), None),
            Symbol::new(Painter::Thruster, Transform::new(Rotation2::new(f32::consts::PI), Scale2::new(0.1, 0.1), Translation2::new(0.8, 0.2)), None),
            Symbol::new(Painter::Thruster, Transform::new(Rotation2::identity(), Scale2::new(0.1, 0.1), Translation2::new(0.8, 0.8)), None),
        ],
        vec![
            Line1D::new(vec![Point2::new(0.35, 0.50), Point2::new(0.5, 0.50)]),
            Line1D::new(vec![Point2::new(0.60, 0.50), Point2::new(0.8, 0.50)]),
            Line1D::new(vec![Point2::new(0.80, 0.50), Point2::new(0.8, 0.45)]),
            Line1D::new(vec![Point2::new(0.80, 0.50), Point2::new(0.8, 0.55)]),
            Line1D::new(vec![Point2::new(0.80, 0.35), Point2::new(0.8, 0.25)]),
            Line1D::new(vec![Point2::new(0.80, 0.65), Point2::new(0.8, 0.75)]),
        ],
        backend,
        frontend
    )
}