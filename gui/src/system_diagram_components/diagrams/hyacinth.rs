use core::f32;
use std::sync::{LazyLock, Once};

use enum_map::{Enum, EnumMap, enum_map};
use itertools::Itertools;
use nalgebra::{Point2, Rotation2, Scale2, Translation2};
use strum::{EnumIter, IntoEnumIterator};
use telemetry::Metric;

use crate::{
    backend::{
        Backend,
        storage::{
            static_metrics::{MaxPressureN2Tank, N2ReleaseValveState},
            storeable_value::ValveState,
        },
    },
    frontend::Frontend,
    system_diagram_components::{
        core::{
            flow_painter::{Line1D, Painter, Symbol},
            fluids::FluidType,
        },
        math::transform::Transform,
        storage::storage_state::StorageState,
    },
    widgets::system_diagram::SystemDiagram,
};

#[derive(Clone)]
pub enum ComponentInteraction {
    PrintInteraction(&'static str),
    ToggleN2ReleaseValveState,
}

impl ComponentInteraction {
    pub fn interact(&self, backend: &mut Backend) {
        match self {
            ComponentInteraction::PrintInteraction(str) => println!("{str}"),
            ComponentInteraction::ToggleN2ReleaseValveState => {
                backend.current_value::<N2ReleaseValveState>().map(|state| match state {
                    ValveState::Open => backend.set_value::<N2ReleaseValveState>(ValveState::Closed),
                    ValveState::Closed => backend.set_value::<N2ReleaseValveState>(ValveState::Open),
                });
            }
        }
    }

    pub fn is_possible(&self, backend: &Backend) -> bool {
        match self {
            ComponentInteraction::PrintInteraction(_) => true,
            ComponentInteraction::ToggleN2ReleaseValveState => {
                backend.current_value::<N2ReleaseValveState>().map(|_| true).unwrap_or(false)
            }
        }
    }

    pub fn description(&self, backend: &Backend) -> String {
        match self {
            ComponentInteraction::PrintInteraction(str) => format!("Print \"{str}\""),
            ComponentInteraction::ToggleN2ReleaseValveState => backend
                .current_value::<N2ReleaseValveState>()
                .map(|state| match state {
                    ValveState::Open => return format!("Close N2ReleaseValve"),
                    ValveState::Closed => return format!("Open N2ReleaseValve"),
                })
                .unwrap_or(format!("Invalid metric or value for N2ReleaseValve")),
        }
    }
}

pub trait SystemComponent {
    fn symbol(&self) -> &'static Symbol;
    fn name(&self) -> &'static str;
    fn metrics(&self) -> Vec<Metric>;
    fn interactions(&self) -> Vec<ComponentInteraction>;
}

impl SystemComponent for HyacinthComponent {
    fn symbol(&self) -> &'static Symbol {
        return &COMPONENT_TO_SYMBOL[*self];
    }

    fn name(&self) -> &'static str {
        return COMPONENT_TO_NAME[*self];
    }

    fn metrics(&self) -> Vec<Metric> {
        return match COMPONENT_TO_RESERVOIR[*self] {
            ReservoirAssoc::PartOf(res) => RESERVOIR_TO_METRICS[res].clone(),
            ReservoirAssoc::ConnectionBetween(res1, res2) => {
                let mut metrics = RESERVOIR_TO_METRICS[res1].clone();
                metrics.append(&mut RESERVOIR_TO_METRICS[res2].clone());
                return metrics;
            }
        };
    }

    fn interactions(&self) -> Vec<ComponentInteraction> {
        return COMPONENT_TO_INTERACTION[*self].clone();
    }
}

#[derive(Enum, EnumIter, Clone, Copy)]
pub enum HyacinthComponent {
    N2Bottle,
    N2OBottle,
    N2BottleValve,
    N2OBottleValve,
    N2Manometer,
    N2ReleaseValve,
    N2OReleaseValve,
    N2FlexTube,
    N2OFlexTube,
    N2QuickDisconnect,
    N2OQuickDisconnect,
    N2FillingCheckValve,
    N2Tank,
    N2TankPressureSensor,
    N2PurgeValve,
    N2PressureRegulator,
    N2ToN2OCheckValve,
    N2OTank,
    N2OVentValve,
    N2OPressureSensorTop,
    N2OBurstDiscOne,
    N2OBurstDiscTwo,
    N2OPressureSensorBottom,
    N2OTemperatureSensor,
    N2OFillAndDumpValve,
    N2OMainValve,
    Igniter,
    CombustionChamberPressureSensor,
    CombustionChamber,
}

#[derive(Enum, Clone, Copy)]
pub enum HyacinthReservoir {
    N2Bottle,
    N2Filling,
    N2OBottle,
    N2OFilling,
    N2Tank,
    N2OTank,
    CombustionChamber,
    Atmosphere,
}

enum ReservoirAssoc {
    PartOf(HyacinthReservoir),
    ConnectionBetween(HyacinthReservoir, HyacinthReservoir),
}

//TODO Hans: Check Valve and QuickDisconnects
static COMPONENT_TO_RESERVOIR: LazyLock<EnumMap<HyacinthComponent, ReservoirAssoc>> = LazyLock::new(|| {
    enum_map![
        HyacinthComponent::N2Bottle => ReservoirAssoc::PartOf(HyacinthReservoir::N2Bottle),
        HyacinthComponent::N2OBottle => ReservoirAssoc::PartOf(HyacinthReservoir::N2OBottle),
        HyacinthComponent::N2BottleValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Bottle, HyacinthReservoir::N2Filling),
        HyacinthComponent::N2OBottleValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OBottle, HyacinthReservoir::N2OFilling),
        HyacinthComponent::N2Manometer => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
        HyacinthComponent::N2ReleaseValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Filling, HyacinthReservoir::Atmosphere),
        HyacinthComponent::N2OReleaseValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OFilling, HyacinthReservoir::Atmosphere),
        HyacinthComponent::N2FlexTube => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
        HyacinthComponent::N2OFlexTube => ReservoirAssoc::PartOf(HyacinthReservoir::N2OFilling),
        HyacinthComponent::N2QuickDisconnect => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
        HyacinthComponent::N2OQuickDisconnect => ReservoirAssoc::PartOf(HyacinthReservoir::N2OFilling),
        HyacinthComponent::N2FillingCheckValve => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
        HyacinthComponent::N2Tank => ReservoirAssoc::PartOf(HyacinthReservoir::N2Tank),
        HyacinthComponent::N2TankPressureSensor => ReservoirAssoc::PartOf(HyacinthReservoir::N2Tank),
        HyacinthComponent::N2PurgeValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Tank, HyacinthReservoir::Atmosphere),
        HyacinthComponent::N2PressureRegulator => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Tank, HyacinthReservoir::N2OTank),
        HyacinthComponent::N2ToN2OCheckValve => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OTank => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OVentValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OTank, HyacinthReservoir::Atmosphere),
        HyacinthComponent::N2OPressureSensorTop => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OBurstDiscOne => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OBurstDiscTwo => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OPressureSensorBottom => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OTemperatureSensor => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
        HyacinthComponent::N2OFillAndDumpValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OFilling, HyacinthReservoir::N2OBottle),
        HyacinthComponent::N2OMainValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OTank, HyacinthReservoir::CombustionChamber),
        HyacinthComponent::Igniter => ReservoirAssoc::PartOf(HyacinthReservoir::CombustionChamber),
        HyacinthComponent::CombustionChamberPressureSensor => ReservoirAssoc::PartOf(HyacinthReservoir::CombustionChamber),
        HyacinthComponent::CombustionChamber => ReservoirAssoc::PartOf(HyacinthReservoir::CombustionChamber),
    ]
});

static COMPONENT_TO_INTERACTION: LazyLock<EnumMap<HyacinthComponent, Vec<ComponentInteraction>>> =
    LazyLock::new(|| {
        let mut interactions: EnumMap<HyacinthComponent, Vec<ComponentInteraction>> = Default::default();
        interactions[HyacinthComponent::N2Bottle]
            .push(ComponentInteraction::PrintInteraction("This is a test interaction"));
        interactions[HyacinthComponent::N2ReleaseValve].push(ComponentInteraction::ToggleN2ReleaseValveState);
        return interactions;
    });

// static RESERVOIR_TO_COMPONENTS: LazyLock<EnumMap<HyacinthReservoir, Vec<HyacinthComponent>>> = LazyLock::new(|| {
//     let mut map = enum_map![
//         HyacinthReservoir::N2Bottle => vec![],
//         HyacinthReservoir::N2Filling => vec![],
//         HyacinthReservoir::N2OBottle => vec![],
//         HyacinthReservoir::N2OFilling => vec![],
//         HyacinthReservoir::N2Tank => vec![],
//         HyacinthReservoir::N2OTank => vec![],
//         HyacinthReservoir::CombustionChamber => vec![],
//         HyacinthReservoir::Atmosphere => vec![],
//     ];
//     for (component, assoc) in COMPONENT_TO_RESERVOIR.iter() {
//         match assoc {
//             ReservoirAssoc::PartOf(reservoir) => {
//                 map[*reservoir].push(component);
//             },
//             ReservoirAssoc::ConnectionBetween(in_reservoir, out_reservoir) => {
//                 map[*in_reservoir].push(component);
//                 map[*out_reservoir].push(component);
//             },
//         }
//     }
//     map[HyacinthReservoir::Atmosphere] = vec![];
//     return map;
// });

static RESERVOIR_TO_METRICS: LazyLock<EnumMap<HyacinthReservoir, Vec<Metric>>> = LazyLock::new(|| {
    enum_map![
        HyacinthReservoir::N2Bottle => vec![
            Metric::LocalMetric(telemetry::LocalMetric::N2ReleaseValveState),
        ],
        HyacinthReservoir::N2Filling => vec![],
        HyacinthReservoir::N2OBottle => vec![],
        HyacinthReservoir::N2OFilling => vec![
            Metric::ValveState(telemetry::ValveId::FillAndDumpValve),
        ],
        HyacinthReservoir::N2Tank => vec![
            Metric::Pressure(telemetry::PressureSensorId::NitrogenTank),
            Metric::ValveState(telemetry::ValveId::PressureRegulator),
            Metric::LocalMetric(telemetry::LocalMetric::MaxPressureN2Tank),
        ],
        HyacinthReservoir::N2OTank => vec![
            Metric::Pressure(telemetry::PressureSensorId::OxidizerTank),
            Metric::Temperature(telemetry::TemperatureSensorId::OxidizerTank),
            Metric::ValveState(telemetry::ValveId::MainValve),
            Metric::ValveState(telemetry::ValveId::FillAndDumpValve),
            Metric::ValveState(telemetry::ValveId::PressureRegulator),
        ],
        HyacinthReservoir::CombustionChamber => vec![
            Metric::Pressure(telemetry::PressureSensorId::CombustionChamber),
            Metric::ValveState(telemetry::ValveId::MainValve),
        ],
        HyacinthReservoir::Atmosphere => vec![],
    ]
});

static COMPONENT_TO_SYMBOL: LazyLock<EnumMap<HyacinthComponent, Symbol>> = LazyLock::new(|| {
    enum_map![
        HyacinthComponent::N2Bottle => Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.24), Translation2::new(0.10, 0.32))),
        HyacinthComponent::N2OBottle => Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.24), Translation2::new(0.10, 0.84))),
        HyacinthComponent::N2BottleValve => Symbol::new(Painter::ManualValve(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.18, 0.16))),
        HyacinthComponent::N2OBottleValve => Symbol::new(Painter::ManualValve(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.18, 0.68))),
        HyacinthComponent::N2Manometer => Symbol::new(Painter::Manometer, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.28, 0.10))),
        HyacinthComponent::N2ReleaseValve => Symbol::new(Painter::ManualValve(ValveState::Closed), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.28, 0.24))),
        HyacinthComponent::N2OReleaseValve => Symbol::new(Painter::ManualValve(ValveState::Closed), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.28, 0.76))),
        HyacinthComponent::N2FlexTube => Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.06), Translation2::new(0.38, 0.16))),
        HyacinthComponent::N2OFlexTube => Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.06), Translation2::new(0.38, 0.68))),
        HyacinthComponent::N2QuickDisconnect => Symbol::new(Painter::QuickDisconnect(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.50, 0.16))),
        HyacinthComponent::N2OQuickDisconnect => Symbol::new(Painter::QuickDisconnectWithCheckValve(ValveState::Open), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.50, 0.68))),
        HyacinthComponent::N2FillingCheckValve => Symbol::new(Painter::CheckValve, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.62, 0.16))),
        HyacinthComponent::N2Tank => Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 0f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.10), Translation2::new(0.77, 0.09))),
        HyacinthComponent::N2PurgeValve => Symbol::new(Painter::ManualValve(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.92, 0.13))),
        HyacinthComponent::N2TankPressureSensor => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.19))),
        HyacinthComponent::N2PressureRegulator => Symbol::new(Painter::MotorizedValve(ValveState::Closed), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.22))),
        HyacinthComponent::N2ToN2OCheckValve => Symbol::new(Painter::CheckValve, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.34))),
        HyacinthComponent::N2OTank => Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 0f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.24), Translation2::new(0.77, 0.54))),
        HyacinthComponent::N2OVentValve => Symbol::new(Painter::SolenoidValve(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.92, 0.36))),
        HyacinthComponent::N2OPressureSensorTop => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.64, 0.40))),
        HyacinthComponent::N2OBurstDiscOne => Symbol::new(Painter::BurstDisc(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.06), Translation2::new(0.94, 0.43))),
        HyacinthComponent::N2OBurstDiscTwo => Symbol::new(Painter::BurstDisc(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.06), Translation2::new(0.94, 0.51))),
        HyacinthComponent::N2OPressureSensorBottom => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.71))),
        HyacinthComponent::N2OTemperatureSensor => Symbol::new(Painter::TemperatureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.65))),
        HyacinthComponent::N2OFillAndDumpValve => Symbol::new(Painter::SolenoidValve(ValveState::Closed), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.62, 0.68))),
        HyacinthComponent::N2OMainValve => Symbol::new(Painter::MotorizedValve(ValveState::Closed), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.74))),
        HyacinthComponent::Igniter => Symbol::new(Painter::Thruster, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.04, 0.06), Translation2::new(0.85, 0.84))),
        HyacinthComponent::CombustionChamberPressureSensor => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.64, 0.80))),
        HyacinthComponent::CombustionChamber => Symbol::new(Painter::Thruster, Transform::new(Rotation2::identity(), Scale2::new(0.10, 0.14), Translation2::new(0.77, 0.89)))
    ]
});

static COMPONENT_TO_NAME: LazyLock<EnumMap<HyacinthComponent, &'static str>> = LazyLock::new(|| {
    enum_map![
        HyacinthComponent::N2Bottle => "N₂ Bottle",
        HyacinthComponent::N2OBottle => "N₂O Bottle",
        HyacinthComponent::N2BottleValve => "N₂ Bottle Valve",
        HyacinthComponent::N2OBottleValve => "N₂O Bottle Valve",
        HyacinthComponent::N2Manometer => "N₂ Manometer",
        HyacinthComponent::N2ReleaseValve => "N₂ Release Valve",
        HyacinthComponent::N2OReleaseValve => "N₂O Release Valve",
        HyacinthComponent::N2FlexTube => "N₂ Flex Tube",
        HyacinthComponent::N2OFlexTube => "N₂O Flex Tube",
        HyacinthComponent::N2QuickDisconnect => "N₂ Quick Disconnect",
        HyacinthComponent::N2OQuickDisconnect => "N₂O Quick Disconnect",
        HyacinthComponent::N2FillingCheckValve => "N₂ Filling Check Valve",
        HyacinthComponent::N2Tank => "N₂ Tank",
        HyacinthComponent::N2TankPressureSensor => "N₂ Tank Pressure Sensor",
        HyacinthComponent::N2PurgeValve => "N₂ Purge Valve",
        HyacinthComponent::N2PressureRegulator =>"N₂ Pressure Regulator",
        HyacinthComponent::N2ToN2OCheckValve =>"N₂ To N₂O Check Valve",
        HyacinthComponent::N2OTank => "N₂O Tank",
        HyacinthComponent::N2OVentValve => "N₂O Vent Valve",
        HyacinthComponent::N2OPressureSensorTop => "N₂O Pressure Sensor Top" ,
        HyacinthComponent::N2OBurstDiscOne => "N₂O Burst Disc (1)",
        HyacinthComponent::N2OBurstDiscTwo => "N₂O Burst Disc (2)",
        HyacinthComponent::N2OPressureSensorBottom => "N₂O Pressure Sensor Bottom",
        HyacinthComponent::N2OTemperatureSensor => "N₂O Temperature Sensor",
        HyacinthComponent::N2OFillAndDumpValve => "N₂O Fill & Dump Valve",
        HyacinthComponent::N2OMainValve => "N₂O Main Valve",
        HyacinthComponent::Igniter => "Igniter",
        HyacinthComponent::CombustionChamberPressureSensor => "Combustion Chamber Pressure Sensor",
        HyacinthComponent::CombustionChamber => "Combustion Chamber",
    ]
});

pub struct SysConnection {
    points: Vec<Point2<f32>>,
}

pub struct SystemDefinition {
    connections: Vec<SysConnection>,
}

fn system_definition() -> SystemDefinition {
    SystemDefinition {
        connections: vec![
            SysConnection {
                //N2 Bottle - N2 Bottle Valve
                points: vec![
                    Point2::new(0.10, 0.20),
                    Point2::new(0.10, 0.16),
                    Point2::new(0.14, 0.16),
                ],
            },
            SysConnection {
                //N2 Bottle Valve - N2 Flex Tube
                points: vec![Point2::new(0.22, 0.16), Point2::new(0.34, 0.16)],
            },
            SysConnection {
                //N2 Manometer - N2 Release Valve
                points: vec![Point2::new(0.28, 0.12), Point2::new(0.28, 0.20)],
            },
            SysConnection {
                //N2 Flex Tube - N2 Quick Disconnect
                points: vec![Point2::new(0.42, 0.16), Point2::new(0.46, 0.16)],
            },
            SysConnection {
                //N2 Quick Disconnect - N2 Tank Check Valve
                points: vec![Point2::new(0.54, 0.16), Point2::new(0.58, 0.16)],
            },
            SysConnection {
                //N2 Tank Check Valve - (Void)
                points: vec![Point2::new(0.66, 0.16), Point2::new(0.86, 0.16)],
            },
            SysConnection {
                //N2 Purge Valve - N2 Tank Pressure Sensor
                points: vec![
                    Point2::new(0.88, 0.13),
                    Point2::new(0.86, 0.13),
                    Point2::new(0.86, 0.19),
                    Point2::new(0.92, 0.19),
                ],
            },
            SysConnection {
                //N2 Tank - N2 Pressure Regulator
                points: vec![Point2::new(0.77, 0.14), Point2::new(0.77, 0.18)],
            },
            SysConnection {
                //N2 Pressure Regulator - N2O Tank Check Valve
                points: vec![Point2::new(0.77, 0.26), Point2::new(0.77, 0.30)],
            },
            SysConnection {
                //N2O Tank Check Valve - N2O Tank
                points: vec![Point2::new(0.77, 0.38), Point2::new(0.77, 0.42)],
            },
            SysConnection {
                //N2O Tank - N2O Tank Pressure Sensor Top
                points: vec![
                    Point2::new(0.735, 0.42),
                    Point2::new(0.735, 0.40),
                    Point2::new(0.66, 0.40),
                ],
            },
            SysConnection {
                //N2O Tank - N2O Vent Valve
                points: vec![
                    Point2::new(0.795, 0.42),
                    Point2::new(0.795, 0.39),
                    Point2::new(0.86, 0.39),
                    Point2::new(0.86, 0.36),
                    Point2::new(0.88, 0.36),
                ],
            },
            SysConnection {
                //N2O Tank - N2O Burst Disc (1)
                points: vec![
                    Point2::new(0.805, 0.42),
                    Point2::new(0.805, 0.40),
                    Point2::new(0.87, 0.40),
                    Point2::new(0.87, 0.43),
                    Point2::new(0.92, 0.43),
                ],
            },
            SysConnection {
                //N2O Tank - N2O Burst Disc (2)
                points: vec![
                    Point2::new(0.815, 0.42),
                    Point2::new(0.815, 0.41),
                    Point2::new(0.86, 0.41),
                    Point2::new(0.86, 0.51),
                    Point2::new(0.92, 0.51),
                ],
            },
            SysConnection {
                //N2O Bottle - N2O Bottle Valve
                points: vec![
                    Point2::new(0.10, 0.72),
                    Point2::new(0.10, 0.68),
                    Point2::new(0.14, 0.68),
                ],
            },
            SysConnection {
                //N2O Bottle Valve - N2O Flex Tube
                points: vec![Point2::new(0.22, 0.68), Point2::new(0.34, 0.68)],
            },
            SysConnection {
                // (Void) - N2O Release Valve
                points: vec![Point2::new(0.28, 0.68), Point2::new(0.28, 0.72)],
            },
            SysConnection {
                //N2O Flex Tube - N2O Quick Disconnect
                points: vec![Point2::new(0.42, 0.68), Point2::new(0.46, 0.68)],
            },
            SysConnection {
                //N2O Quick Disconnect - N2O Fill & Dump Valve
                points: vec![Point2::new(0.54, 0.68), Point2::new(0.58, 0.68)],
            },
            SysConnection {
                //N2O Fill & Dump Valve - N2O Tank
                points: vec![
                    Point2::new(0.66, 0.68),
                    Point2::new(0.735, 0.68),
                    Point2::new(0.735, 0.66),
                ],
            },
            SysConnection {
                //N2O Tank - N2O Temperature Sensor
                points: vec![
                    Point2::new(0.81, 0.66),
                    Point2::new(0.81, 0.675),
                    Point2::new(0.88, 0.675),
                    Point2::new(0.88, 0.66),
                    Point2::new(0.92, 0.66),
                ],
            },
            SysConnection {
                //N2O Tank - N2O Pressure Sensor Bottom
                points: vec![
                    Point2::new(0.80, 0.66),
                    Point2::new(0.80, 0.685),
                    Point2::new(0.88, 0.685),
                    Point2::new(0.88, 0.70),
                    Point2::new(0.92, 0.70),
                ],
            },
            SysConnection {
                //N2O Tank - N2O Main Valve
                points: vec![Point2::new(0.77, 0.66), Point2::new(0.77, 0.70)],
            },
            SysConnection {
                //N2O Main Valve - Combustion Chamber
                points: vec![Point2::new(0.77, 0.78), Point2::new(0.77, 0.82)],
            },
            SysConnection {
                //Combustion Chamber - Combustion Chamber Pressure Sensor
                points: vec![
                    Point2::new(0.745, 0.82),
                    Point2::new(0.745, 0.80),
                    Point2::new(0.66, 0.80),
                ],
            },
        ],
    }
}

static INIT_METRICS: Once = Once::new();

pub fn create_diagram<'a>(
    backend: &'a mut Backend,
    frontend: &'a mut Frontend,
) -> SystemDiagram<'a, HyacinthComponent> {
    let system_definition = system_definition();
    INIT_METRICS.call_once(|| {
        let _ = backend.set_value::<MaxPressureN2Tank>(50f64);
        let _ = backend.set_value::<N2ReleaseValveState>(ValveState::Closed);
    });
    SystemDiagram::new(
        HyacinthComponent::iter().collect_vec(),
        system_definition.connections.iter().map(|c| Line1D::new(c.points.clone())).collect(),
        backend,
        frontend,
    )
}
