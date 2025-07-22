use core::f32;
use std::{collections::HashMap, sync::LazyLock};

use nalgebra::{Point2, Rotation2, Scale2, Translation2};
use telemetry::Metric;

use crate::{backend::Backend, frontend::{constraints::Constraint, metric_monitor::MetricMonitor, popup_manager::PopupManager}, system_diagram_components::{core::{flow_painter::{Painter, Symbol}, fluids::FluidType}, math::transform::Transform, storage::storage_state::StorageState, valves::valve_state::ValveState}, widgets::{plot::SharedPlotState, system_diagram::SystemDiagram}};

type ComponentID = u16;
type PortID = u16;

const LEFT: Port = Port { _name: "LEFT", _position: Point2::new(-0.5, 0.0) };
const RIGHT: Port = Port { _name: "RIGHT", _position: Point2::new(0.5, 0.0) };
const TOP: Port = Port { _name: "TOP", _position: Point2::new(0.0, -0.5) };
const BOT: Port = Port { _name: "BOT", _position: Point2::new(0.0, 0.5) };

pub struct Port {
    _name: &'static str,
    _position: Point2<f32>,
}

pub struct SysComponent {
    _name: &'static str,
    _metric: Vec<Metric>,
    _constraints: Vec<Constraint>,
    symbol: Symbol,
    _ports: Vec<Port> //TODO TO HASH MAP
}

pub struct System {
    _name: &'static str,
    _components: HashMap<ComponentID, SysComponent>,
    _connections: HashMap<(ComponentID, PortID), (ComponentID, PortID)>
}

pub static HYACINTH_SYSTEM: LazyLock<Vec<SysComponent>> = LazyLock::new(|| vec![
    SysComponent {
        _name: "N₂ Bottle",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.38)), None),
        _ports: vec![ TOP ]
    },
    SysComponent {
        _name: "N₂O Bottle",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.77)), None),
        _ports: vec![ TOP ]
    },
    SysComponent {
        _name: "N₂O Bottle Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2O))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.18, 0.61)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Bottle Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.18, 0.23)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂O Release Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.07, 0.04), Translation2::new(0.27, 0.715)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Release Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.07, 0.04), Translation2::new(0.27, 0.325)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Manometer",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Manometer, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.27, 0.16)), None),
        _ports: vec![ BOT ]
    },
    SysComponent {
        _name: "N₂O Flex Tube",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.37, 0.61)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Flex Tube",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.37, 0.23)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂O Quick Disconnect",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.52, 0.61)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Quick Disconnect",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.52, 0.23)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Tank",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.72, 0.12)), None),
        _ports: vec![ BOT ]
    },
    SysComponent {
        _name: "N₂ Pressure Regulator",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.07, 0.04), Translation2::new(0.72, 0.28)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Relief Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Missing, Transform::new(Rotation2::identity(), Scale2::new(0.06, 0.03), Translation2::new(0.90, 0.28)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂ Burst Disc",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::BurstDisc(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.03, 0.03), Translation2::new(0.90, 0.38)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂O Vent Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.52, 0.33)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂O Tank",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.72, 0.44)), None),
        _ports: vec![ TOP, BOT ]
    },
    SysComponent {
        _name: "N₂O Fill & Dump Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.62, 0.61)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "N₂O Main Valve",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.07, 0.04), Translation2::new(0.72, 0.76)), None),
        _ports: vec![ LEFT, RIGHT ]
    },
    SysComponent {
        _name: "Igniter",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Thruster, Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.04, 0.06), Translation2::new(0.62, 0.83)), None),   
        _ports: vec![ TOP, BOT ]
    },
    SysComponent {
        _name: "Combustion Chamber",
        _metric: vec![],
        _constraints: vec![],
        symbol: Symbol::new(Painter::Thruster, Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.18), Translation2::new(0.72, 0.9)), None),   
        _ports: vec![ TOP, BOT ]
    },
]);

pub fn create_diagram<'a>(backend: &'a Backend, shared_plot_state: &'a mut SharedPlotState, popup_manager: &'a mut PopupManager, metric_monitor: &'a mut MetricMonitor) -> SystemDiagram<'a> {
    SystemDiagram::new(
        HYACINTH_SYSTEM.iter().map(|c| c.symbol.clone()).collect(),
        vec![],
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
        shared_plot_state,
        popup_manager,
        metric_monitor
    )
}