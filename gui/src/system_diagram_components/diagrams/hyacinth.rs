use core::f32;
use std::sync::LazyLock;

use crate::{
    backend::storage::static_metrics::{
        N2BottleValve, N2OBottleValve, N2OBurstDisc, N2OFillAndDumpValve, N2OMainValve, N2OQuickDisconnect,
        N2OReleaseValve, N2OVentValve, N2PressureRegulator, N2PurgeValve, N2QuickDisconnect, N2ReleaseValve, Pressure,
        PressureSensorNitrogenTank, ProcedureStep,
    },
    frontend::constraints::{Constraint, ConstraintBuilder},
    storage::static_metrics::MetricTrait,
};
use enum_map::{Enum, EnumMap, enum_map};
use itertools::Itertools;
use nalgebra::{Point2, Rotation2, Scale2, Translation2};
use strum::{EnumIter, IntoEnumIterator};
use telemetry::{LocalMetric, Metric};

use crate::{
    backend::{
        Backend,
        storage::{static_metrics::MaxPressureN2Tank, storeable_value::ValveState},
    },
    frontend::{Frontend, constraints::ConstraintResult},
    system_diagram_components::{
        core::{
            flow_painter::{BinaryValvePainter, Line1D, Painter, Symbol},
            fluids::FluidType,
        },
        math::transform::Transform,
        storage::storage_state::StorageState,
    },
    widgets::system_diagram::SystemDiagram,
};

macro_rules! make_valve_interactions {
    ($($name:ident),*) => {
        #[derive(Clone)]
        pub enum ToggleValve {
            $( $name ),*
        }

    impl ToggleValve {
        pub fn interact(&self, backend: &mut Backend) {
            match self {
                $( ToggleValve::$name => {
                backend.current_value::<crate::storage::static_metrics::$name>().map(|state| match state {
                    ValveState::Open => backend.set_value::<crate::storage::static_metrics::$name>(ValveState::Closed),
                    ValveState::Closed => backend.set_value::<crate::storage::static_metrics::$name>(ValveState::Open),
                });
            }),*
            }
        }

        pub fn is_possible(&self, backend: &Backend) -> bool {
            match self {
                $( ToggleValve::$name => {
                backend.current_value::<crate::storage::static_metrics::$name>().is_some()
            }),*
            }
        }

        pub fn description(&self, backend: &Backend) -> String {
            match self {
                $( ToggleValve::$name => backend.current_value::<crate::storage::static_metrics::$name>()
                .map(|state| match state {
                    ValveState::Open => format!("Close {}", <crate::storage::static_metrics::$name as MetricTrait>::metric()),
                    ValveState::Closed => format!("Open {}", <crate::storage::static_metrics::$name as MetricTrait>::metric()),
                })
                .unwrap_or(format!("Invalid metric or value for {}", <crate::storage::static_metrics::$name as MetricTrait>::metric()))
            ),*
            }
        }
    }

            #[derive(Clone)]
        pub enum OpenValve {
            $( $name ),*
        }

    impl OpenValve {
        pub fn interact(&self, backend: &mut Backend) {
            let _ = match self {
                $( OpenValve::$name =>
                    backend.set_value::<crate::storage::static_metrics::$name>(ValveState::Open),

            )*
            };
        }

        pub fn is_possible(&self, backend: &Backend) -> bool {
            match self {
                $( OpenValve::$name => {
                backend.current_value::<crate::storage::static_metrics::$name>().is_none()
            }),*
            }
        }

        pub fn description(&self, _backend: &Backend) -> String {
            match self {
                $( OpenValve::$name => format!("Open {}", <crate::storage::static_metrics::$name as MetricTrait>::metric())
            ),*
            }
        }
    }
                #[derive(Clone)]
        pub enum CloseValve {
            $( $name ),*
        }

    impl CloseValve {
        pub fn interact(&self, backend: &mut Backend) {
            let _ = match self {
                $( CloseValve::$name =>
                    backend.set_value::<crate::storage::static_metrics::$name>(ValveState::Closed),

            )*
            };
        }

        pub fn is_possible(&self, backend: &Backend) -> bool {
            match self {
                $( CloseValve::$name => {
                backend.current_value::<crate::storage::static_metrics::$name>().is_none()
            }),*
            }
        }

        pub fn description(&self, _backend: &Backend) -> String {
            match self {
                $( CloseValve::$name => format!("Close {}", <crate::storage::static_metrics::$name as MetricTrait>::metric())
            ),*
            }
        }
    }
    }
}

make_valve_interactions!(
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
    N2OMainValve
);

#[derive(Clone)]
pub enum ComponentInteraction {
    ToggleValve(ToggleValve),
    OpenValve(OpenValve),
    CloseValve(CloseValve),
}

impl ComponentInteraction {
    pub fn interact(&self, backend: &mut Backend) {
        match self {
            ComponentInteraction::ToggleValve(valve) => valve.interact(backend),
            ComponentInteraction::OpenValve(open_valve) => open_valve.interact(backend),
            ComponentInteraction::CloseValve(close_valve) => close_valve.interact(backend),
        }
    }

    pub fn is_possible(&self, backend: &Backend) -> bool {
        match self {
            ComponentInteraction::ToggleValve(valve) => valve.is_possible(backend),
            ComponentInteraction::OpenValve(open_valve) => open_valve.is_possible(backend),
            ComponentInteraction::CloseValve(close_valve) => close_valve.is_possible(backend),
        }
    }

    pub fn description(&self, backend: &Backend) -> String {
        match self {
            ComponentInteraction::ToggleValve(valve) => valve.description(backend),
            ComponentInteraction::OpenValve(open_valve) => open_valve.description(backend),
            ComponentInteraction::CloseValve(close_valve) => close_valve.description(backend),
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
        return COMPONENT_TO_METRIC[*self].clone();
        // return match COMPONENT_TO_RESERVOIR[*self] {
        //     ReservoirAssoc::PartOf(res) => RESERVOIR_TO_METRICS[res].clone(),
        //     ReservoirAssoc::ConnectionBetween(res1, res2) => {
        //         let mut metrics = RESERVOIR_TO_METRICS[res1].clone();
        //         metrics.append(&mut RESERVOIR_TO_METRICS[res2].clone());
        //         return metrics;
        //     }
        // };
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

// #[derive(Enum, Clone, Copy)]
// pub enum HyacinthReservoir {
//     N2Bottle,
//     N2Filling,
//     N2OBottle,
//     N2OFilling,
//     N2Tank,
//     N2OTank,
//     CombustionChamber,
//     Atmosphere,
// }

// enum ReservoirAssoc {
//     PartOf(HyacinthReservoir),
//     ConnectionBetween(HyacinthReservoir, HyacinthReservoir),
// }

//TODO Hans: Check Valve and QuickDisconnects
// static COMPONENT_TO_RESERVOIR: LazyLock<EnumMap<HyacinthComponent, ReservoirAssoc>> = LazyLock::new(|| {
//     enum_map![
//         HyacinthComponent::N2Bottle => ReservoirAssoc::PartOf(HyacinthReservoir::N2Bottle),
//         HyacinthComponent::N2OBottle => ReservoirAssoc::PartOf(HyacinthReservoir::N2OBottle),
//         HyacinthComponent::N2BottleValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Bottle, HyacinthReservoir::N2Filling),
//         HyacinthComponent::N2OBottleValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OBottle, HyacinthReservoir::N2OFilling),
//         HyacinthComponent::N2Manometer => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
//         HyacinthComponent::N2ReleaseValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Filling, HyacinthReservoir::Atmosphere),
//         HyacinthComponent::N2OReleaseValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OFilling, HyacinthReservoir::Atmosphere),
//         HyacinthComponent::N2FlexTube => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
//         HyacinthComponent::N2OFlexTube => ReservoirAssoc::PartOf(HyacinthReservoir::N2OFilling),
//         HyacinthComponent::N2QuickDisconnect => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
//         HyacinthComponent::N2OQuickDisconnect => ReservoirAssoc::PartOf(HyacinthReservoir::N2OFilling),
//         HyacinthComponent::N2FillingCheckValve => ReservoirAssoc::PartOf(HyacinthReservoir::N2Filling),
//         HyacinthComponent::N2Tank => ReservoirAssoc::PartOf(HyacinthReservoir::N2Tank),
//         HyacinthComponent::N2TankPressureSensor => ReservoirAssoc::PartOf(HyacinthReservoir::N2Tank),
//         HyacinthComponent::N2PurgeValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Tank, HyacinthReservoir::Atmosphere),
//         HyacinthComponent::N2PressureRegulator => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2Tank, HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2ToN2OCheckValve => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OTank => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OVentValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OTank, HyacinthReservoir::Atmosphere),
//         HyacinthComponent::N2OPressureSensorTop => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OBurstDiscOne => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OBurstDiscTwo => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OPressureSensorBottom => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OTemperatureSensor => ReservoirAssoc::PartOf(HyacinthReservoir::N2OTank),
//         HyacinthComponent::N2OFillAndDumpValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OFilling, HyacinthReservoir::N2OBottle),
//         HyacinthComponent::N2OMainValve => ReservoirAssoc::ConnectionBetween(HyacinthReservoir::N2OTank, HyacinthReservoir::CombustionChamber),
//         HyacinthComponent::Igniter => ReservoirAssoc::PartOf(HyacinthReservoir::CombustionChamber),
//         HyacinthComponent::CombustionChamberPressureSensor => ReservoirAssoc::PartOf(HyacinthReservoir::CombustionChamber),
//         HyacinthComponent::CombustionChamber => ReservoirAssoc::PartOf(HyacinthReservoir::CombustionChamber),
//     ]
// });

static COMPONENT_TO_INTERACTION: LazyLock<EnumMap<HyacinthComponent, Vec<ComponentInteraction>>> =
    LazyLock::new(|| {
        let mut interactions: EnumMap<HyacinthComponent, Vec<ComponentInteraction>> = Default::default();
        //Toggle Valve
        interactions[HyacinthComponent::N2BottleValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2BottleValve));
        interactions[HyacinthComponent::N2OBottleValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OBottleValve));
        interactions[HyacinthComponent::N2ReleaseValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2ReleaseValve));
        interactions[HyacinthComponent::N2OReleaseValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OReleaseValve));
        interactions[HyacinthComponent::N2QuickDisconnect]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2QuickDisconnect));
        interactions[HyacinthComponent::N2OQuickDisconnect]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OQuickDisconnect));
        interactions[HyacinthComponent::N2PurgeValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2PurgeValve));
        interactions[HyacinthComponent::N2PressureRegulator]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2PressureRegulator));
        interactions[HyacinthComponent::N2OVentValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OVentValve));
        interactions[HyacinthComponent::N2OBurstDiscOne]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OBurstDisc));
        interactions[HyacinthComponent::N2OBurstDiscTwo]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OBurstDisc));
        interactions[HyacinthComponent::N2OFillAndDumpValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OFillAndDumpValve));
        interactions[HyacinthComponent::N2OMainValve]
            .push(ComponentInteraction::ToggleValve(ToggleValve::N2OMainValve));
        //Close Valve
        interactions[HyacinthComponent::N2BottleValve]
            .push(ComponentInteraction::CloseValve(CloseValve::N2BottleValve));
        interactions[HyacinthComponent::N2OBottleValve]
            .push(ComponentInteraction::CloseValve(CloseValve::N2OBottleValve));
        interactions[HyacinthComponent::N2ReleaseValve]
            .push(ComponentInteraction::CloseValve(CloseValve::N2ReleaseValve));
        interactions[HyacinthComponent::N2OReleaseValve]
            .push(ComponentInteraction::CloseValve(CloseValve::N2OReleaseValve));
        interactions[HyacinthComponent::N2QuickDisconnect]
            .push(ComponentInteraction::CloseValve(CloseValve::N2QuickDisconnect));
        interactions[HyacinthComponent::N2OQuickDisconnect]
            .push(ComponentInteraction::CloseValve(CloseValve::N2OQuickDisconnect));
        interactions[HyacinthComponent::N2PurgeValve].push(ComponentInteraction::CloseValve(CloseValve::N2PurgeValve));
        interactions[HyacinthComponent::N2PressureRegulator]
            .push(ComponentInteraction::CloseValve(CloseValve::N2PressureRegulator));
        interactions[HyacinthComponent::N2OVentValve].push(ComponentInteraction::CloseValve(CloseValve::N2OVentValve));
        interactions[HyacinthComponent::N2OBurstDiscOne]
            .push(ComponentInteraction::CloseValve(CloseValve::N2OBurstDisc));
        interactions[HyacinthComponent::N2OBurstDiscTwo]
            .push(ComponentInteraction::CloseValve(CloseValve::N2OBurstDisc));
        interactions[HyacinthComponent::N2OFillAndDumpValve]
            .push(ComponentInteraction::CloseValve(CloseValve::N2OFillAndDumpValve));
        interactions[HyacinthComponent::N2OMainValve].push(ComponentInteraction::CloseValve(CloseValve::N2OMainValve));
        //Open Valve
        interactions[HyacinthComponent::N2BottleValve].push(ComponentInteraction::OpenValve(OpenValve::N2BottleValve));
        interactions[HyacinthComponent::N2OBottleValve]
            .push(ComponentInteraction::OpenValve(OpenValve::N2OBottleValve));
        interactions[HyacinthComponent::N2ReleaseValve]
            .push(ComponentInteraction::OpenValve(OpenValve::N2ReleaseValve));
        interactions[HyacinthComponent::N2OReleaseValve]
            .push(ComponentInteraction::OpenValve(OpenValve::N2OReleaseValve));
        interactions[HyacinthComponent::N2QuickDisconnect]
            .push(ComponentInteraction::OpenValve(OpenValve::N2QuickDisconnect));
        interactions[HyacinthComponent::N2OQuickDisconnect]
            .push(ComponentInteraction::OpenValve(OpenValve::N2OQuickDisconnect));
        interactions[HyacinthComponent::N2PurgeValve].push(ComponentInteraction::OpenValve(OpenValve::N2PurgeValve));
        interactions[HyacinthComponent::N2PressureRegulator]
            .push(ComponentInteraction::OpenValve(OpenValve::N2PressureRegulator));
        interactions[HyacinthComponent::N2OVentValve].push(ComponentInteraction::OpenValve(OpenValve::N2OVentValve));
        interactions[HyacinthComponent::N2OBurstDiscOne].push(ComponentInteraction::OpenValve(OpenValve::N2OBurstDisc));
        interactions[HyacinthComponent::N2OBurstDiscTwo].push(ComponentInteraction::OpenValve(OpenValve::N2OBurstDisc));
        interactions[HyacinthComponent::N2OFillAndDumpValve]
            .push(ComponentInteraction::OpenValve(OpenValve::N2OFillAndDumpValve));
        interactions[HyacinthComponent::N2OMainValve].push(ComponentInteraction::OpenValve(OpenValve::N2OMainValve));
        return interactions;
    });

static COMPONENT_TO_METRIC: LazyLock<EnumMap<HyacinthComponent, Vec<Metric>>> = LazyLock::new(|| {
    let mut metrics: EnumMap<HyacinthComponent, Vec<Metric>> = Default::default();
    //Valves
    metrics[HyacinthComponent::N2BottleValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2BottleValve)]);
    metrics[HyacinthComponent::N2OBottleValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2OBottleValve)]);
    metrics[HyacinthComponent::N2ReleaseValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2ReleaseValve)]);
    metrics[HyacinthComponent::N2OReleaseValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2OReleaseValve)]);
    metrics[HyacinthComponent::N2QuickDisconnect]
        .append(&mut vec![Metric::LocalMetric(LocalMetric::N2QuickDisconnect)]);
    metrics[HyacinthComponent::N2OQuickDisconnect]
        .append(&mut vec![Metric::LocalMetric(LocalMetric::N2OQuickDisconnect)]);
    metrics[HyacinthComponent::N2PurgeValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2PurgeValve)]);
    metrics[HyacinthComponent::N2PressureRegulator]
        .append(&mut vec![Metric::LocalMetric(LocalMetric::N2PressureRegulator)]);
    metrics[HyacinthComponent::N2OVentValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2OVentValve)]);
    metrics[HyacinthComponent::N2OBurstDiscOne].append(&mut vec![Metric::LocalMetric(LocalMetric::N2OBurstDisc)]);
    metrics[HyacinthComponent::N2OBurstDiscTwo].append(&mut vec![Metric::LocalMetric(LocalMetric::N2OBurstDisc)]);
    metrics[HyacinthComponent::N2OFillAndDumpValve]
        .append(&mut vec![Metric::LocalMetric(LocalMetric::N2OFillAndDumpValve)]);
    metrics[HyacinthComponent::N2OMainValve].append(&mut vec![Metric::LocalMetric(LocalMetric::N2OMainValve)]);
    //Tanks
    metrics[HyacinthComponent::N2Tank].append(&mut vec![
        Metric::Pressure(telemetry::PressureSensorId::NitrogenTank),
        Metric::LocalMetric(LocalMetric::MaxPressureN2Tank),
    ]);
    metrics[HyacinthComponent::N2OTank].append(&mut vec![
        Metric::Pressure(telemetry::PressureSensorId::OxidizerTank),
        Metric::Temperature(telemetry::TemperatureSensorId::OxidizerTank),
    ]);
    metrics[HyacinthComponent::CombustionChamber]
        .append(&mut vec![Metric::Pressure(telemetry::PressureSensorId::CombustionChamber)]);

    return metrics;
});
// static RESERVOIR_TO_METRICS: LazyLock<EnumMap<HyacinthReservoir, Vec<Metric>>> = LazyLock::new(|| {
//     enum_map![
//         HyacinthReservoir::N2Bottle => vec![
//             Metric::LocalMetric(telemetry::LocalMetric::N2ReleaseValve),
//         ],
//         HyacinthReservoir::N2Filling => vec![],
//         HyacinthReservoir::N2OBottle => vec![],
//         HyacinthReservoir::N2OFilling => vec![
//             Metric::ValveState(telemetry::ValveId::FillAndDumpValve),
//         ],
//         HyacinthReservoir::N2Tank => vec![
//             Metric::Pressure(telemetry::PressureSensorId::NitrogenTank),
//             Metric::ValveState(telemetry::ValveId::PressureRegulator),
//             Metric::LocalMetric(telemetry::LocalMetric::MaxPressureN2Tank),
//         ],
//         HyacinthReservoir::N2OTank => vec![
//             Metric::Pressure(telemetry::PressureSensorId::OxidizerTank),
//             Metric::Temperature(telemetry::TemperatureSensorId::OxidizerTank),
//             Metric::ValveState(telemetry::ValveId::MainValve),
//             Metric::ValveState(telemetry::ValveId::FillAndDumpValve),
//             Metric::ValveState(telemetry::ValveId::PressureRegulator),
//         ],
//         HyacinthReservoir::CombustionChamber => vec![
//             Metric::Pressure(telemetry::PressureSensorId::CombustionChamber),
//             Metric::ValveState(telemetry::ValveId::MainValve),
//         ],
//         HyacinthReservoir::Atmosphere => vec![],
//     ]
// });

static COMPONENT_TO_SYMBOL: LazyLock<EnumMap<HyacinthComponent, Symbol>> = LazyLock::new(|| {
    enum_map![
        HyacinthComponent::N2Bottle => Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.24), Translation2::new(0.10, 0.32))),
        HyacinthComponent::N2OBottle => Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.24), Translation2::new(0.10, 0.84))),
        HyacinthComponent::N2BottleValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::ManualValve, Metric::LocalMetric(LocalMetric::N2BottleValve)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.18, 0.16))),
        HyacinthComponent::N2OBottleValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::ManualValve, Metric::LocalMetric(LocalMetric::N2OBottleValve)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.18, 0.68))),
        HyacinthComponent::N2Manometer => Symbol::new(Painter::Manometer, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.28, 0.10))),
        HyacinthComponent::N2ReleaseValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::ManualValve, Metric::LocalMetric(LocalMetric::N2ReleaseValve)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.28, 0.24))),
        HyacinthComponent::N2OReleaseValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::ManualValve, Metric::LocalMetric(LocalMetric::N2OReleaseValve)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.28, 0.76))),
        HyacinthComponent::N2FlexTube => Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.06), Translation2::new(0.38, 0.16))),
        HyacinthComponent::N2OFlexTube => Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.06), Translation2::new(0.38, 0.68))),
        HyacinthComponent::N2QuickDisconnect => Symbol::new(Painter::BinaryValve(BinaryValvePainter::QuickDisconnect, Metric::LocalMetric(LocalMetric::N2QuickDisconnect)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.50, 0.16))),
        HyacinthComponent::N2OQuickDisconnect => Symbol::new(Painter::BinaryValve(BinaryValvePainter::QuickDisconnectWithCheckValve, Metric::LocalMetric(LocalMetric::N2OQuickDisconnect)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.50, 0.68))),
        HyacinthComponent::N2FillingCheckValve => Symbol::new(Painter::CheckValve, Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.62, 0.16))),
        HyacinthComponent::N2Tank => Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 0f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.10), Translation2::new(0.77, 0.09))),
        HyacinthComponent::N2PurgeValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::ManualValve, Metric::LocalMetric(LocalMetric::N2PurgeValve)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.92, 0.13))),
        HyacinthComponent::N2TankPressureSensor => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.19))),
        HyacinthComponent::N2PressureRegulator => Symbol::new(Painter::BinaryValve(BinaryValvePainter::MotorizedValve, Metric::LocalMetric(LocalMetric::N2PressureRegulator)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.22))),
        HyacinthComponent::N2ToN2OCheckValve => Symbol::new(Painter::CheckValve, Transform::new(Rotation2::new(f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.34))),
        HyacinthComponent::N2OTank => Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 0f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.24), Translation2::new(0.77, 0.54))),
        HyacinthComponent::N2OVentValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::SolenoidValve, Metric::LocalMetric(LocalMetric::N2OVentValve)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.92, 0.36))),
        HyacinthComponent::N2OPressureSensorTop => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.64, 0.40))),
        HyacinthComponent::N2OBurstDiscOne => Symbol::new(Painter::BinaryValve(BinaryValvePainter::BurstDisc, Metric::LocalMetric(LocalMetric::N2OBurstDisc)), Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.06), Translation2::new(0.94, 0.43))),
        HyacinthComponent::N2OBurstDiscTwo => Symbol::new(Painter::BinaryValve(BinaryValvePainter::BurstDisc, Metric::LocalMetric(LocalMetric::N2OBurstDisc)), Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.06), Translation2::new(0.94, 0.51))),
        HyacinthComponent::N2OPressureSensorBottom => Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.71))),
        HyacinthComponent::N2OTemperatureSensor => Symbol::new(Painter::TemperatureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.94, 0.65))),
        HyacinthComponent::N2OFillAndDumpValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::SolenoidValve, Metric::LocalMetric(LocalMetric::N2OFillAndDumpValve)), Transform::new(Rotation2::identity(), Scale2::new(0.08, 0.04), Translation2::new(0.62, 0.68))),
        HyacinthComponent::N2OMainValve => Symbol::new(Painter::BinaryValve(BinaryValvePainter::MotorizedValve, Metric::LocalMetric(LocalMetric::N2OMainValve)), Transform::new(Rotation2::new(-f32::consts::FRAC_PI_2), Scale2::new(0.08, 0.04), Translation2::new(0.77, 0.74))),
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

pub fn create_diagram<'a>(
    backend: &'a mut Backend,
    frontend: &'a mut Frontend,
) -> SystemDiagram<'a, HyacinthComponent> {
    let system_definition = system_definition();
    if !backend.has_initialized_local_metrics() {
        let _ = backend.set_value::<MaxPressureN2Tank>(280f64);
        //TODO REMOVE: Initialize with norminal valve states for display
        let _ = backend.set_value::<N2BottleValve>(ValveState::Closed);
        let _ = backend.set_value::<N2OBottleValve>(ValveState::Closed);
        let _ = backend.set_value::<N2ReleaseValve>(ValveState::Closed);
        let _ = backend.set_value::<N2OReleaseValve>(ValveState::Closed);
        let _ = backend.set_value::<N2QuickDisconnect>(ValveState::Closed);
        let _ = backend.set_value::<N2OQuickDisconnect>(ValveState::Closed);
        let _ = backend.set_value::<N2PurgeValve>(ValveState::Closed);
        let _ = backend.set_value::<N2PressureRegulator>(ValveState::Closed);
        let _ = backend.set_value::<N2OVentValve>(ValveState::Closed);
        let _ = backend.set_value::<N2OBurstDisc>(ValveState::Closed);
        let _ = backend.set_value::<N2OFillAndDumpValve>(ValveState::Closed);
        let _ = backend.set_value::<N2OMainValve>(ValveState::Closed);

        backend.initialize_local_metrics();
    }
    if !frontend.initialized() {
        frontend
            .metric_monitor_mut()
            .add_constraint(
                Pressure::<PressureSensorNitrogenTank>::leq_metric::<MaxPressureN2Tank>()
                    .on_violation(ConstraintResult::DANGER),
            )
            .add_constraint(
                ProcedureStep::eq_const(shared_types::telemetry::ProcedureStep::N2Filling)
                    .invert()
                    .implies(N2BottleValve::eq_const(ValveState::Closed))
                    .on_violation(ConstraintResult::WARNING),
            )
            .add_constraint(
                ProcedureStep::gt_const(shared_types::telemetry::ProcedureStep::IdleActive)
                    .implies(N2QuickDisconnect::eq_const(ValveState::Open))
                    .on_violation(ConstraintResult::WARNING),
            )
            .add_constraint(
                ProcedureStep::gt_const(shared_types::telemetry::ProcedureStep::N2OFilling)
                    .implies(N2OQuickDisconnect::eq_const(ValveState::Open))
                    .on_violation(ConstraintResult::WARNING),
            )
            .add_constraint(
                ProcedureStep::eq_const(shared_types::telemetry::ProcedureStep::N2OFilling)
                    .invert()
                    .implies(N2OBottleValve::eq_const(ValveState::Closed))
                    .on_violation(ConstraintResult::WARNING),
            )
            .add_constraint(N2OBurstDisc::eq_const(ValveState::Closed).on_violation(ConstraintResult::DANGER));
        frontend.initialize();
    }

    SystemDiagram::new(
        HyacinthComponent::iter().collect_vec(),
        system_definition.connections.iter().map(|c| Line1D::new(c.points.clone())).collect(),
        backend,
        frontend,
    )
}
