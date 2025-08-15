use core::f32;
use std::{collections::HashMap, hash::Hash};

use egui::accesskit::Point;
use nalgebra::{Point2, Rotation2, Scale2, Translation2};
use telemetry::Metric;

use crate::{backend::Backend, frontend::Frontend, system_diagram_components::{core::{flow_painter::{Line1D, Painter, Symbol}, fluids::FluidType}, math::transform::Transform, storage::storage_state::StorageState, valves::valve_state::ValveState}, widgets::system_diagram::SystemDiagram};

pub struct Port {
    _name: &'static str,
    _position: Point2<f32>,
}

pub struct SysComponent {
    _name: &'static str,
    symbol: Symbol,
}

pub struct SysConnection {
    //from: &'a SysComponent,
    //to: &'a SysComponent,
    points: Vec<Point2<f32>>,
}


pub struct Reservoir<'a> {
    components: Vec<&'a SysComponent>,
}

pub struct SystemDefinition {
    //reservoirs: HashMap<&'static str, Reservoir<'a>>,
    components: Vec<SysComponent>,
    connections: Vec<SysConnection>,
    //eservoirs: Vec<Reservoir>
}

fn system_definition() -> SystemDefinition { SystemDefinition {
    components: vec![
        SysComponent {
            _name: "N₂ Bottle",
            symbol: Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.24), Translation2::new(0.10, 0.32)), None),
        },
        SysComponent {
            _name: "N₂O Bottle",
            symbol: Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.24), Translation2::new(0.10, 0.84)), None),
        },
        SysComponent {
            _name: "N₂ Bottle Valve",
            symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2))), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.18, 0.16)), None),
        },
        SysComponent {
            _name: "N₂O Bottle Valve",
            symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2O))), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.18, 0.68)), None),
        },
        SysComponent {
            _name: "N₂ Manometer",
            symbol: Symbol::new(Painter::Manometer, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.28, 0.10)), None),
        },
        SysComponent {
            _name: "N₂ Release Valve",
            symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.28, 0.24)), None),
        },
        SysComponent {
            _name: "N₂O Release Valve",
            symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.28, 0.76)), None),
        },
        SysComponent {
            _name: "N₂ Flex Tube",
            symbol: Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.06), Translation2::new(0.38, 0.16)), None),
        },
        SysComponent {
            _name: "N₂O Flex Tube",
            symbol: Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.06), Translation2::new(0.38, 0.68)), None),
        },
        SysComponent {
            _name: "N₂ Quick Disconnect",
            symbol: Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.02), Translation2::new(0.50, 0.16)), None),
        },
        SysComponent {
            _name: "N₂O Quick Disconnect",
            symbol: Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.02), Translation2::new(0.50, 0.68)), None),
        },
        SysComponent {
            _name: "N₂ Tank Check Valve",
            symbol: Symbol::new(Painter::CheckValve, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.62, 0.16)), None),
        },
        SysComponent {
            _name: "N₂ Tank",
            symbol: Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.10), Translation2::new(0.77, 0.09)), None),
        },
        SysComponent {
            _name: "N₂ Purge Valve",
            symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.92, 0.13)), None),
        },
        SysComponent {
            _name: "N₂ Tank Pressure Sensor",
            symbol: Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.19)), None),
        },
        SysComponent {
            _name: "N₂ Pressure Regulator",
            symbol: Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.22)), None),
        },
        SysComponent {
            _name: "N₂ Tank Check Valve",
            symbol: Symbol::new(Painter::CheckValve, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.34)), None),
        },
        SysComponent {
            _name: "N₂O Pressure Sensor Top",
            symbol: Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.64, 0.40)), None),
        },
        SysComponent {
            _name: "N₂O Vent Valve",
            symbol: Symbol::new(Painter::SolenoidValve(ValveState::Connected(None)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.92, 0.36)), None),
        },
        SysComponent {
            _name: "N₂O Burst Disc (1)",
            symbol: Symbol::new(Painter::BurstDisc(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.06), Translation2::new(0.94, 0.43)), None),
        },
        SysComponent {
            _name: "N₂O Burst Disc (2)",
            symbol: Symbol::new(Painter::BurstDisc(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.06), Translation2::new(0.94, 0.51)), None),
        },
        SysComponent {
            _name: "N₂O Temperature Sensor",
            symbol: Symbol::new(Painter::TemperatureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.65)), None),
        },
        SysComponent {
            _name: "N₂O Pressure Sensor Bot",
            symbol: Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.71)), None),
        },
        SysComponent {
            _name: "N₂O Tank",
            symbol: Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.24), Translation2::new(0.77, 0.54)), None),
        },
        SysComponent {
            _name: "N₂O Fill & Dump Valve",
            symbol: Symbol::new(Painter::SolenoidValve(ValveState::Connected(None)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.62, 0.68)), None),
        },
        SysComponent {
            _name: "N₂O Main Valve",
            symbol: Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.74)), None),
        },
        SysComponent {
            _name: "Igniter",
            symbol: Symbol::new(Painter::Thruster, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.04, 0.06), Translation2::new(0.85, 0.84)), None),   
        },
        SysComponent {
            _name: "Combustion Chamber Pressure Sensor",
            symbol: Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.64, 0.80)), None),   
        },
        SysComponent {
            _name: "Combustion Chamber",
            symbol: Symbol::new(Painter::Thruster, Transform::new(Rotation2::identity(), Scale2::new(0.10, 0.14), Translation2::new(0.77, 0.89)), None),   
        },
    ],
    connections: vec![
        SysConnection { //N2 Bottle - N2 Bottle Valve
            points: vec![Point2::new(0.10, 0.20), Point2::new(0.10, 0.16), Point2::new(0.14, 0.16)]
        },
        SysConnection { //N2 Bottle Valve - N2 Flex Tube
            points: vec![Point2::new(0.22, 0.16), Point2::new(0.34, 0.16)]
        },
        SysConnection { //N2 Manometer - N2 Release Valve
            points: vec![Point2::new(0.28, 0.12), Point2::new(0.28, 0.20)]
        },
        SysConnection { //N2 Flex Tube - N2 Quick Disconnect
            points: vec![Point2::new(0.42, 0.16), Point2::new(0.46, 0.16)]
        },
        SysConnection { //N2 Quick Disconnect - N2 Tank Check Valve
            points: vec![Point2::new(0.54, 0.16), Point2::new(0.58, 0.16)]
        },
        SysConnection { //N2 Tank Check Valve - (Void)
            points: vec![Point2::new(0.66, 0.16), Point2::new(0.86, 0.16)]
        },
        SysConnection { //N2 Purge Valve - N2 Tank Pressure Sensor
            points: vec![Point2::new(0.88, 0.13), Point2::new(0.86, 0.13), Point2::new(0.86, 0.19), Point2::new(0.92, 0.19)]
        },
        SysConnection { //N2 Tank - N2 Pressure Regulator
            points: vec![Point2::new(0.77, 0.14), Point2::new(0.77, 0.18)]
        },
        SysConnection { //N2 Pressure Regulator - N2O Tank Check Valve
            points: vec![Point2::new(0.77, 0.26), Point2::new(0.77, 0.30)]
        },
        SysConnection { //N2O Tank Check Valve - N2O Tank
            points: vec![Point2::new(0.77, 0.38), Point2::new(0.77, 0.42)]
        },
        SysConnection { //N2O Tank - N2O Tank Pressure Sensor Top
            points: vec![Point2::new(0.735, 0.42), Point2::new(0.735, 0.40), Point2::new(0.66, 0.40)]
        },
        SysConnection { //N2O Tank - N2O Vent Valve
            points: vec![Point2::new(0.795, 0.42), Point2::new(0.795, 0.39), Point2::new(0.86, 0.39), Point2::new(0.86, 0.36), Point2::new(0.88, 0.36)]
        },
        SysConnection { //N2O Tank - N2O Burst Disc (1)
            points: vec![Point2::new(0.805, 0.42), Point2::new(0.805, 0.40), Point2::new(0.87, 0.40), Point2::new(0.87, 0.43), Point2::new(0.92, 0.43)]
        },
        SysConnection { //N2O Tank - N2O Burst Disc (2)
            points: vec![Point2::new(0.815, 0.42), Point2::new(0.815, 0.41), Point2::new(0.86, 0.41), Point2::new(0.86, 0.51), Point2::new(0.92, 0.51)]
        },
        SysConnection { //N2O Bottle - N2O Bottle Valve
            points: vec![Point2::new(0.10, 0.72), Point2::new(0.10, 0.68), Point2::new(0.14, 0.68)]
        },
        SysConnection { //N2O Bottle Valve - N2O Flex Tube
            points: vec![Point2::new(0.22, 0.68), Point2::new(0.34, 0.68)]
        },
        SysConnection { // (Void) - N2O Release Valve
            points: vec![Point2::new(0.28, 0.68), Point2::new(0.28, 0.72)]
        },
        SysConnection { //N2O Flex Tube - N2O Quick Disconnect
            points: vec![Point2::new(0.42, 0.68), Point2::new(0.46, 0.68)]
        },
        SysConnection { //N2O Quick Disconnect - N2O Fill & Dump Valve
            points: vec![Point2::new(0.54, 0.68), Point2::new(0.58, 0.68)]
        },
        SysConnection { //N2O Fill & Dump Valve - N2O Tank
            points: vec![Point2::new(0.66, 0.68), Point2::new(0.735, 0.68), Point2::new(0.735, 0.66)]
        },
        SysConnection { //N2O Tank - N2O Temperature Sensor
            points: vec![Point2::new(0.81, 0.66), Point2::new(0.81, 0.675), Point2::new(0.88, 0.675), Point2::new(0.88, 0.66), Point2::new(0.92, 0.66)]
        },
        SysConnection { //N2O Tank - N2O Pressure Sensor Bottom
            points: vec![Point2::new(0.80, 0.66), Point2::new(0.80, 0.685), Point2::new(0.88, 0.685), Point2::new(0.88, 0.70), Point2::new(0.92, 0.70)]
        },
        SysConnection { //N2O Tank - N2O Main Valve
            points: vec![Point2::new(0.77, 0.66), Point2::new(0.77, 0.70)]
        },
        SysConnection { //N2O Main Valve - Combustion Chamber
            points: vec![Point2::new(0.77, 0.78), Point2::new(0.77, 0.82)]
        },
        SysConnection { //Combustion Chamber - Combustion Chamber Pressure Sensor
            points: vec![Point2::new(0.745, 0.82), Point2::new(0.745, 0.80), Point2::new(0.66, 0.80)]
        },

    ]
}}

pub fn create_diagram<'a>(backend: &'a Backend, frontend: &'a mut Frontend) -> SystemDiagram<'a> {
    let system_definition = system_definition();
    SystemDiagram::new(
        system_definition.components.iter().map(|c| c.symbol.clone()).collect(),
        system_definition.connections.iter().map(|c| Line1D::new(c.points.clone())).collect(),
        // vec![
        //     //GSE
        //     Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.38)), None),
        //     Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.77)), None),
        //     Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2O))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.175, 0.61)), None),
        //     Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.175, 0.23)), None),
        //     Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.37, 0.325)), None),
        //     Symbol::new(Painter::Atmosphere, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.37, 0.4)), None),
        //     Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.47, 0.23)), None),
        //     Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.47, 0.61)), None),
        //     Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.585, 0.23)), None),
        //    // Symbol::new(Painter::MotorizedValve(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.295, 0.61)), None),
        //     Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.37, 0.715)), None),
        //     Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.585, 0.61)), None),
        //     Symbol::new(Painter::Manometer, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.37, 0.16)), None),
        //     //ROCKET
        //     Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.12)), None),
        //     Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.44)), None),
        //     Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.71, 0.28)), None),
        //     Symbol::new(Painter::BurstDisc(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.03, 0.03), Translation2::new(0.92, 0.37)), None),
        //     Symbol::new(Painter::Missing, Transform::new(Rotation2::identity(), Scale2::new(0.06, 0.03), Translation2::new(0.90, 0.28)), None),
        //     Symbol::new(Painter::TemperatureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.79, 0.65)), None),
        //     Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.63, 0.65)), None),
        //     Symbol::new(Painter::Thruster, Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.9)), None),   
        //     Symbol::new(Painter::Thruster, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.03, 0.08), Translation2::new(0.6, 0.83)), None),   
        //     Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.07, 0.04), Translation2::new(0.71, 0.76)), None),
        //     Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.76, 0.78)), None),
        // ],
        // vec![
        //     //GSE
        //     Line1D::new(vec![Point2::new(0.10, 0.26), Point2::new(0.10, 0.23), Point2::new(0.14, 0.23)]),
        //     Line1D::new(vec![Point2::new(0.21, 0.23), Point2::new(0.41, 0.23)]),
        //     Line1D::new(vec![Point2::new(0.53, 0.23), Point2::new(0.55, 0.23)]),
        //     Line1D::new(vec![Point2::new(0.37, 0.29), Point2::new(0.37, 0.18)]),
        //     Line1D::new(vec![Point2::new(0.10, 0.65), Point2::new(0.10, 0.61), Point2::new(0.14, 0.61)]),
        //     Line1D::new(vec![Point2::new(0.21, 0.61), Point2::new(0.41, 0.61)]),
        //     Line1D::new(vec![Point2::new(0.37, 0.61), Point2::new(0.37, 0.68)]),
        //     Line1D::new(vec![Point2::new(0.53, 0.61), Point2::new(0.55, 0.61)]),
        // ],
        backend,
        frontend
    )
}