use crate::backend::{Backend, storage::storeable_value::ValveState};
use egui::Ui;
use nalgebra::{Affine2, Point2};
use telemetry::Metric;

use crate::{
    system_diagram_components::{
        math::{parallelogram::CENTERED_UNIT_RECT, transform::Transform},
        other::*,
        sensors_and_actuators::{manometer, motor, pressure_sensor, temperature_sensor},
        storage::{storage_state::StorageState, *},
        valves::*,
    },
    utils::theme::ThemeColors,
};

#[derive(Clone)]
pub struct Symbol {
    painter: Painter,
    local_transform: Affine2<f32>,
}

#[derive(Clone)]
pub struct Line1D {
    points: Vec<Point2<f32>>,
}

impl Line1D {
    pub fn new(points: Vec<Point2<f32>>) -> Self {
        Self { points }
    }

    pub fn paint(&self, global_transform: &Affine2<f32>, painter: &egui::Painter, ctx: &egui::Context) {
        let theme = &ThemeColors::new(ctx);
        line1d::paint(&self.points, global_transform, painter, theme);
    }
}

impl Symbol {
    pub fn new(painter: Painter, local_transform: Transform) -> Self {
        Self {
            painter,
            local_transform: local_transform.to_affine2(),
        }
    }

    pub fn paint(&self, global_transform: &Affine2<f32>, ui: &mut egui::Ui, backend: &Backend) -> egui::Rect {
        let transform = &(global_transform * self.local_transform);
        let ctx = ui.ctx();
        let theme = &ThemeColors::new(ctx);
        let painter = ui.painter();
        match &self.painter {
            //----------------------- Valves -----------------------
            Painter::BinaryValve(valve_painter, metric) => {
                valve_painter.paint(transform, ui, &backend.current_value_dynamic::<ValveState>(metric))
            }
            // Painter::GenericValve(state) => generic_valve::paint(transform, state, painter, theme),
            // Painter::ManualValve(state) => manual_valve::paint(transform, state, painter, theme),
            // Painter::MotorizedValve(state) => motorized_valve::paint(transform, state, painter, ctx),
            // Painter::BurstDisc(state) => burst_disc::paint(transform, state, painter, theme),
            // Painter::QuickDisconnect(state) => quick_disconnect::paint(transform, state, painter, theme),
            // Painter::QuickDisconnectWithCheckValve(state) => {
            //     quick_disconnect_with_check_valve::paint(transform, state, painter, theme)
            // }
            // Painter::PressureReliefValve(state) => pressure_relief_valve::paint(transform, state, painter, theme),
            // Painter::SolenoidValve(state) => solenoid_valve::paint(transform, state, painter, ctx),
            Painter::CheckValve => check_valve::paint(transform, painter, theme),

            //----------------------- Storage ----------------------
            Painter::Tank(state) => tank::paint(transform, state, painter, theme),
            Painter::Bottle(state) => bottle::paint(transform, state, painter, theme),
            Painter::Atmosphere => atmosphere::paint(transform, painter, ctx),

            //--------------- Sensors and Actuators ----------------
            Painter::Motor => motor::paint(transform, painter, ctx),
            Painter::PressureSensor => pressure_sensor::paint(transform, painter, ctx),
            Painter::TemperatureSensor => temperature_sensor::paint(transform, painter, ctx),
            Painter::Manometer => manometer::paint(transform, painter, ctx),

            //----------------------- Other ------------------------
            Painter::Missing => missing::paint(transform, painter),
            Painter::FlexTube => flex_tube::paint(transform, painter, theme),
            Painter::Thruster => thruster::paint(transform, painter, theme),
        }
        return CENTERED_UNIT_RECT.transform(transform).axis_aligned_bounding_box();
    }
}

#[derive(Clone)]
pub enum Painter {
    //----------------------- Valves -----------------------
    BinaryValve(BinaryValvePainter, Metric),
    // GenericValve(ValveState),
    // ManualValve(ValveState),
    // MotorizedValve(ValveState),
    // BurstDisc(ValveState),
    // QuickDisconnect(ValveState),
    // QuickDisconnectWithCheckValve(ValveState),
    // PressureReliefValve(ValveState),
    // SolenoidValve(ValveState),
    CheckValve,
    //----------------------- Storage ----------------------
    Tank(StorageState),
    Bottle(StorageState),
    Atmosphere,
    //--------------- Sensors and Actuators ----------------
    Motor,
    PressureSensor,
    TemperatureSensor,
    Manometer,
    //----------------------- Other ------------------------
    Missing,
    FlexTube,
    Thruster,
}

//TODO Hans: Make this static, see below
#[derive(Clone)]
pub enum BinaryValvePainter {
    GenericValve, //Hans: Until rust gits gud, we need to store ValveStateMetric somwhere
    ManualValve,
    MotorizedValve,
    BurstDisc,
    QuickDisconnect,
    QuickDisconnectWithCheckValve,
    PressureReliefValve,
    SolenoidValve,
}

impl BinaryValvePainter {
    fn paint(&self, transform: &Affine2<f32>, ui: &mut Ui, valve_state: &Option<ValveState>) {
        let theme = &ThemeColors::new(ui.ctx());
        match self {
            BinaryValvePainter::GenericValve => generic_valve::paint(transform, valve_state, &ui.painter(), theme),

            BinaryValvePainter::ManualValve => generic_valve::paint(transform, valve_state, &ui.painter(), theme),
            BinaryValvePainter::MotorizedValve => {
                motorized_valve::paint(transform, valve_state, &ui.painter(), ui.ctx())
            }
            BinaryValvePainter::BurstDisc => {
                burst_disc::paint(transform, &valve_state.clone().unwrap_or_default(), &ui.painter(), theme)
            }
            BinaryValvePainter::QuickDisconnect => {
                quick_disconnect_no_check_valve::paint(transform, valve_state, &ui.painter(), theme, ui.ctx())
            }
            BinaryValvePainter::QuickDisconnectWithCheckValve => {
                quick_disconnect_with_check_valve::paint(transform, valve_state, &ui.painter(), theme, ui.ctx())
            }
            BinaryValvePainter::PressureReliefValve => {
                pressure_relief_valve::paint(transform, valve_state, &ui.painter(), theme)
            }
            BinaryValvePainter::SolenoidValve => solenoid_valve::paint(transform, valve_state, &ui.painter(), ui.ctx()),
        };
    }
}

// trait PainterType {
//     fn paint(&self, transform: &Affine2<f32>, ui: &mut Ui, backend: &mut Backend);
// }
// impl<ValveStateMetric: MetricTrait<Value = ValveState>> PainterType for BinaryValvePainter<ValveStateMetric> {
//     fn paint(&self, transform: &Affine2<f32>, ui: &mut Ui, backend: &mut Backend) {
//         let theme = &ThemeColors::new(ui.ctx());
//         let valve_state = backend.current_value::<ValveStateMetric>();
//         match self {
//             BinaryValvePainter::GenericValve(_) => {
//                 generic_valve::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), theme)
//             }

//             BinaryValvePainter::ManualValve => {
//                 generic_valve::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), theme)
//             }
//             BinaryValvePainter::MotorizedValve => {
//                 motorized_valve::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), ui.ctx())
//             }
//             BinaryValvePainter::BurstDisc => {
//                 burst_disc::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), theme)
//             }
//             BinaryValvePainter::QuickDisconnect => {
//                 quick_disconnect::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), theme)
//             }
//             BinaryValvePainter::QuickDisconnectWithCheckValve => quick_disconnect_with_check_valve::paint(
//                 transform,
//                 &valve_state.unwrap_or_default(),
//                 &ui.painter(),
//                 theme,
//             ),
//             BinaryValvePainter::PressureReliefValve => {
//                 pressure_relief_valve::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), theme)
//             }
//             BinaryValvePainter::SolenoidValve => {
//                 solenoid_valve::paint(transform, &valve_state.unwrap_or_default(), &ui.painter(), ui.ctx())
//             }
//         };
//     }
// }

// pub enum BinaryValvePainter<ValveStateMetric: MetricTrait<Value = ValveState>> {
//     GenericValve(PhantomData<ValveStateMetric>), //Hans: Until rust gits gud, we need to store ValveStateMetric somwhere
//     ManualValve,
//     MotorizedValve,
//     BurstDisc,
//     QuickDisconnect,
//     QuickDisconnectWithCheckValve,
//     PressureReliefValve,
//     SolenoidValve,
// }
