use nalgebra::{Point2, Rotation2, Scale2, Translation2};

use crate::{backend::Backend, system_diagram_components::{core::{flow_painter::{Line1D, Painter, Symbol}, fluids::FluidType}, math::transform::Transform, storage::storage_state::StorageState, valves::valve_state::ValveState}, widgets::system_diagram::SystemDiagram};

pub fn create_diagram(_backend: &Backend) -> SystemDiagram {
    SystemDiagram::new(
        vec![
            //GSE
            Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.38)), vec![]),
            Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.77)), vec![]),
            Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2O))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.175, 0.61)), vec![]),
            Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.175, 0.23)), vec![]),
            Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.37, 0.325)), vec![]),
            Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.47, 0.23)), vec![]),
            Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.47, 0.61)), vec![]),
            Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.585, 0.23)), vec![]),
            Symbol::new(Painter::MotorizedValve(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.295, 0.61)), vec![]),
            Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.37, 0.715)), vec![]),
            Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.585, 0.61)), vec![]),
            Symbol::new(Painter::Missing, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.37, 0.16)), vec![]),
            //ROCKET
            Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.12)), vec![]),
            Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.44)), vec![]),
            Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.71, 0.28)), vec![]),
            Symbol::new(Painter::BurstDisc(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.03, 0.03), Translation2::new(0.92, 0.37)), vec![]),
            Symbol::new(Painter::Missing, Transform::new(Rotation2::identity(), Scale2::new(0.06, 0.03), Translation2::new(0.90, 0.28)), vec![]),
            Symbol::new(Painter::TemperatureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.79, 0.65)), vec![]),
            Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.63, 0.65)), vec![]),
    ],
    vec![
            //GSE
            Line1D::new(vec![Point2::new(0.10, 0.26), Point2::new(0.10, 0.23), Point2::new(0.14, 0.23)]),
            Line1D::new(vec![Point2::new(0.21, 0.23), Point2::new(0.41, 0.23)]),
            Line1D::new(vec![Point2::new(0.53, 0.23), Point2::new(0.55, 0.23)]),
            Line1D::new(vec![Point2::new(0.37, 0.29), Point2::new(0.37, 0.18)]),
            Line1D::new(vec![Point2::new(0.10, 0.65), Point2::new(0.10, 0.61), Point2::new(0.14, 0.61)]),
            Line1D::new(vec![Point2::new(0.21, 0.61), Point2::new(0.26, 0.61)]),
            Line1D::new(vec![Point2::new(0.33, 0.61), Point2::new(0.41, 0.61)]),
            Line1D::new(vec![Point2::new(0.37, 0.61), Point2::new(0.37, 0.68)]),
            Line1D::new(vec![Point2::new(0.53, 0.61), Point2::new(0.55, 0.61)]),
        ]
    )
}