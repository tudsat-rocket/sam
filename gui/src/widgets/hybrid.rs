use egui::{Pos2, Vec2};

use crate::{
    flow_components::{
        bottle::BottlePainter,
        burst_disc::BurstDiscPainter,
        constants::{ETHANOL, N2, N2O},
        flex_tube::FlexTubePainter,
        flow_component::{
            ComponentInfo, DisplayValue, FlowComponent, Fluid, Justification, JustifiedValue, Value, TEST_SENSOR,
        },
        line::{LinePainter1D, LinePainter2D},
        sensor::SymbolPainter,
        tank::TankPainter,
        valve::{BallValvePainter, CheckValvePainter, TankValvePainter, ValvePainter},
    },
    utils::mesh::register_textures,
};

pub struct HybridSystemDiagram {
    components: Vec<FlowComponent>,
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
                        60.0,
                        Fluid {
                            fluid_type: N2O,
                            pressure: JustifiedValue::new(Some(Value::F32(15.0)), Justification::Reasoned),
                        },
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
                        300.0,
                        Fluid {
                            fluid_type: N2,
                            pressure: JustifiedValue::new(Some(Value::F32(180.0)), Justification::Reasoned),
                        },
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
                        60.0,
                        Fluid {
                            fluid_type: ETHANOL,
                            pressure: JustifiedValue::new(Some(Value::F32(15.0)), Justification::Reasoned),
                        },
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
                    Box::new(FlexTubePainter::new(Pos2::new(0.93, 0.25), 0.1, 0.08)),
                    None,
                ),
                //-Check valve-
                FlowComponent::new(
                    ComponentInfo::new(String::from("Check valve"), vec![]),
                    Box::new(CheckValvePainter::new(Pos2::new(0.75, 0.25), 0.1, 0.08)),
                    None,
                ),
            ],
        }
    }
}

impl egui::Widget for HybridSystemDiagram {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            for comp in self.components {
                comp.draw_and_respond(ui);
            }
        })
        .response
    }
}
