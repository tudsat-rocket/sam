use egui::{Pos2, Vec2};

use crate::{
    flow_components::{
        bottle::BottlePainter,
        burst_disc::BurstDiscPainter,
        flex_tube::FlexTubePainter,
        flow_component::{
            ComponentInfo, ConnectionState, DisplayValue, FillState, FlowComponent, FluidType, Justification, JustifiedValue, Value, TEST_SENSOR
        },
        line::{LinePainter1D, LinePainter2D},
        sensor::SymbolPainter,
        tank::TankPainter,
        valve::{BallValvePainter, CheckValvePainter, TankValvePainter, ValvePainter},
    }, flow_components_new::{flow_painter::FlowPainter::GenericValve, math::transform::Transform, painter_instance::PainterInstance}, utils::{mesh::register_textures, theme::ThemeColors}
};
use nalgebra::{Rotation2, Scale2, Translation2};
pub struct HybridSystemDiagram {
    components: Vec<FlowComponent>,
    new_test_painter: PainterInstance
}


impl HybridSystemDiagram {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new() -> Self {
        Self {
            components: vec![
                //------N2O Bottle-----
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("N\u{2082}O Bottle"),
                        vec![DisplayValue::new(
                            "Fill".to_string(),
                            Some("%".to_string()),
                            JustifiedValue::new(Some(Value::F32(0.2)), Justification::Measured(TEST_SENSOR)),
                        )],
                    ),
                    Box::new(BottlePainter::new(
                        Pos2::new(0.15, 0.5),
                        0.2,
                        0.6,
                        /*60.0,*/ FillState::new(FluidType::N2O, 0.3),
                    )),
                    None,
                ),
                //------N2 Tank------
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("N\u{2082} Tank"),
                        vec![
                            DisplayValue::new(
                                "Fill".to_string(),
                                Some("%".to_string()),
                                JustifiedValue::new(Some(Value::F32(0.6)), Justification::Measured(TEST_SENSOR)),
                            ),
                            DisplayValue::new("Test".to_string(), None, JustifiedValue::new(None, Justification::None)),
                        ],
                    ),
                    Box::new(TankPainter::new(
                        Vec2::new(0.4, 0.5),
                        0.15,
                        0.4,
                        /*300.0,*/ FillState::new(FluidType::N2, 0.6),
                    )),
                    Some("You clicked the N2 tank".to_string()),
                ),
                //------Ethanol Tank-----
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("N\u{2082}O Tank"),
                        vec![DisplayValue::new(
                            "Fill".to_string(),
                            Some("%".to_string()),
                            JustifiedValue::new(Some(Value::F32(0.2)), Justification::Measured(TEST_SENSOR)),
                        )],
                    ),
                    Box::new(TankPainter::new(
                        Vec2::new(0.6, 0.5),
                        0.15,
                        0.4,
                        /*60.0,*/ FillState::new(FluidType::ETHANOL, 0.2),
                    )),
                    None,
                ),
                //------Test Line 1D -----
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("Line 1D"),
                        vec![DisplayValue::new(
                            "Test".to_string(),
                            None,
                            JustifiedValue::new(None, Justification::None),
                        )],
                    ),
                    Box::new(LinePainter1D::new(vec![Pos2::new(0.6, 0.8), Pos2::new(0.65, 0.7), Pos2::new(0.7, 0.8)])),
                    None,
                ),
                //------Test Line 2D -----
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("Line 2D"),
                        vec![DisplayValue::new(
                            "Test".to_string(),
                            None,
                            JustifiedValue::new(None, Justification::None),
                        )],
                    ),
                    Box::new(LinePainter2D::new(vec![
                        Pos2::new(0.15, 0.8),
                        Pos2::new(0.15, 0.9),
                        Pos2::new(0.5, 0.9),
                        Pos2::new(0.5, 0.8),
                    ])),
                    None,
                ),
                //------Tank Valve---
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("Tank valve"),
                        vec![DisplayValue::new(
                            "Test".to_string(),
                            None,
                            JustifiedValue::new(None, Justification::None),
                        )],
                    ),
                    Box::new(TankValvePainter::new(ValvePainter::new(Pos2::new(0.75, 0.5), 0.1))),
                    None,
                ),
                //------Ball Valve---
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("Ball valve"),
                        vec![DisplayValue::new(
                            "Test".to_string(),
                            None,
                            JustifiedValue::new(None, Justification::None),
                        )],
                    ),
                    Box::new(BallValvePainter::new(ValvePainter::new(Pos2::new(0.9, 0.5), 0.1))),
                    None,
                ),
                //-Temperature Sensor-
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("Temperature Sensor"),
                        vec![DisplayValue::new(
                            "Temperature".to_string(),
                            Some("C".to_string()),
                            JustifiedValue::new(None, Justification::None),
                        )],
                    ),
                    Box::new(SymbolPainter::new(Pos2::new(0.75, 0.75), 0.02, "T")),
                    None,
                ),
                //-Pressure Sensor-
                FlowComponent::new(
                    ComponentInfo::new(
                        String::from("Pressure Sensor"),
                        vec![DisplayValue::new(
                            "Pressure".to_string(),
                            Some("Bar".to_string()),
                            JustifiedValue::new(None, Justification::None),
                        )],
                    ),
                    Box::new(SymbolPainter::new(Pos2::new(0.85, 0.75), 0.03, "P")),
                    None,
                ),
                //-Burst Disc-
                FlowComponent::new(
                    ComponentInfo::new(String::from("Burst Disc"), vec![]),
                    Box::new(BurstDiscPainter::new(Pos2::new(0.93, 0.75), 0.06, 0.12)),
                    None,
                ),
                //-Flex Tube-
                FlowComponent::new(
                    ComponentInfo::new(String::from("Flex Tube"), vec![]),
                    Box::new(FlexTubePainter::new(Pos2::new(0.75, 0.25), 0.1, 0.08)),
                    None,
                ),
                //-Check valve-
                FlowComponent::new(
                    ComponentInfo::new(String::from("Check valve"), vec![]),
                    Box::new(CheckValvePainter::new(Pos2::new(0.90, 0.25), 0.1, 0.08)),
                    None,
                ),
            ],
            new_test_painter: PainterInstance::new(
                //GenericValve(ConnectionState::Connected(None)),
                crate::flow_components_new::flow_painter::FlowPainter::Missing,
                Transform::new(Rotation2::new(45f32.to_radians()), Scale2::new(0.1, 0.1), Translation2::new(0.2, 0.2)))
        }
    }
}

impl egui::Widget for HybridSystemDiagram {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let res = ui.vertical(|ui| {
            for comp in self.components {
                comp.draw_and_respond(ui);
            }
        })
        .response;
        let available_space = ui.available_rect_before_wrap();
        let global_transform = Transform::new(
            Rotation2::new(0f32),
            Scale2::new(available_space.width(), available_space.height()),
            Translation2::new(available_space.min.x, available_space.min.y))
            .to_affine2();
        self.new_test_painter.paint(&global_transform, ui.painter(), ThemeColors::new(ui.ctx()));
        return res;
    }
}
