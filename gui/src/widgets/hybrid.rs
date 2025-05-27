use crate::{flow_components::flow_component::FluidType, flow_components_new::{core::flow_painter::{Line1D, Painter, Symbol}, math::transform::Transform, storage::storage_state::StorageState, valves::valve_state::ValveState}, utils::mesh::register_textures};
use nalgebra::{Point2, Rotation2, Scale2, Translation2};
pub struct HybridSystemDiagram {
    //components: Vec<FlowComponent>,
    //new_test_painter: PainterInstance
    symbols: Vec<Symbol>,
    lines: Vec<Line1D>,
}

impl HybridSystemDiagram {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new() -> Self {
        Self {
            symbols: vec![
                //GSE
                Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.38))),
                Symbol::new(Painter::Bottle(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.17, 0.24), Translation2::new(0.10, 0.77))),
                Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2O))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.175, 0.61))),
                Symbol::new(Painter::ManualValve(ValveState::Connected(Some(FluidType::N2))), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.175, 0.23))),
                Symbol::new(Painter::ManualValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.37, 0.325))),
                Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.47, 0.23))),
                Symbol::new(Painter::FlexTube, Transform::new(Rotation2::identity(), Scale2::new(0.12, 0.05), Translation2::new(0.47, 0.61))),
                Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.585, 0.23))),
                Symbol::new(Painter::MotorizedValve(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.04), Translation2::new(0.295, 0.61))),
                Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.37, 0.715))),
                Symbol::new(Painter::QuickDisconnect(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.07, 0.02), Translation2::new(0.585, 0.61))),
                Symbol::new(Painter::Missing, Transform::new(Rotation2::new(0f32), Scale2::new(0.04, 0.04), Translation2::new(0.37, 0.16))),
                //ROCKET
                Symbol::new(Painter::Tank(StorageState::new(FluidType::N2, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.12))),
                Symbol::new(Painter::Tank(StorageState::new(FluidType::N2O, 1f32)), Transform::new(Rotation2::identity(), Scale2::new(0.14, 0.17), Translation2::new(0.71, 0.44))),
                Symbol::new(Painter::MotorizedValve(ValveState::Connected(None)), Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.07, 0.04), Translation2::new(0.71, 0.28))),
                Symbol::new(Painter::BurstDisc(ValveState::Disconnected), Transform::new(Rotation2::identity(), Scale2::new(0.03, 0.03), Translation2::new(0.92, 0.37))),
                Symbol::new(Painter::Missing, Transform::new(Rotation2::identity(), Scale2::new(0.06, 0.03), Translation2::new(0.90, 0.28))),
                Symbol::new(Painter::TemperatureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.79, 0.65))),
                Symbol::new(Painter::PressureSensor, Transform::new(Rotation2::identity(), Scale2::new(0.04, 0.04), Translation2::new(0.63, 0.65))),
            ],
            lines: vec![
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
        }
    }
}


impl egui::Widget for HybridSystemDiagram {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let available_space = ui.available_rect_before_wrap();
        let global_transform = Transform::new(
            Rotation2::new(0f32),
            Scale2::new(available_space.width(), available_space.height()),
            Translation2::new(available_space.min.x, available_space.min.y))
            .to_affine2();
        return ui.vertical(|ui| {
            let painter= ui.painter();
            let ctx = ui.ctx();
            let mut i = 0;
            for symbol in self.symbols {
                let bounding_box= symbol.paint(&global_transform, painter, ctx);
                
                //Temporary code for tooltips
                let response = ui.interact(bounding_box, egui::Id::new(format!("Test {}", i)), egui::Sense::click());
                if response.hovered() {
                    let pos = response.hover_pos();
                    egui::show_tooltip_at(ui.ctx(), ui.layer_id(), egui::Id::new(format!("Test {}", i)), pos.unwrap(), |ui| {
                        ui.vertical(|ui| {
                            ui.label("Test Hover String");
                        });
                    })
                }
                i += 1;
            }
            for line in self.lines{
                line.paint(&global_transform, painter, ctx);
            }
        })
        .response;
    }
}


// impl HybridSystemDiagram {
//     pub fn init(ctx: &egui::Context) {
//         register_textures(ctx);
//     }

//     pub fn new() -> Self {
//         Self {
//             components: vec![
//                 //------N2O Bottle-----
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("N\u{2082}O Bottle"),
//                         vec![DisplayValue::new(
//                             "Fill".to_string(),
//                             Some("%".to_string()),
//                             JustifiedValue::new(Some(Value::F32(0.2)), Justification::Measured(TEST_SENSOR)),
//                         )],
//                     ),
//                     Box::new(BottlePainter::new(
//                         Pos2::new(0.15, 0.5),
//                         0.2,
//                         0.6,
//                         /*60.0,*/ FillState::new(FluidType::N2O, 0.3),
//                     )),
//                     None,
//                 ),
//                 //------N2 Tank------
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("N\u{2082} Tank"),
//                         vec![
//                             DisplayValue::new(
//                                 "Fill".to_string(),
//                                 Some("%".to_string()),
//                                 JustifiedValue::new(Some(Value::F32(0.6)), Justification::Measured(TEST_SENSOR)),
//                             ),
//                             DisplayValue::new("Test".to_string(), None, JustifiedValue::new(None, Justification::None)),
//                         ],
//                     ),
//                     Box::new(TankPainter::new(
//                         Vec2::new(0.4, 0.5),
//                         0.15,
//                         0.4,
//                         /*300.0,*/ FillState::new(FluidType::N2, 0.6),
//                     )),
//                     Some("You clicked the N2 tank".to_string()),
//                 ),
//                 //------Ethanol Tank-----
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("N\u{2082}O Tank"),
//                         vec![DisplayValue::new(
//                             "Fill".to_string(),
//                             Some("%".to_string()),
//                             JustifiedValue::new(Some(Value::F32(0.2)), Justification::Measured(TEST_SENSOR)),
//                         )],
//                     ),
//                     Box::new(TankPainter::new(
//                         Vec2::new(0.6, 0.5),
//                         0.15,
//                         0.4,
//                         /*60.0,*/ FillState::new(FluidType::ETHANOL, 0.2),
//                     )),
//                     None,
//                 ),
//                 //------Test Line 1D -----
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("Line 1D"),
//                         vec![DisplayValue::new(
//                             "Test".to_string(),
//                             None,
//                             JustifiedValue::new(None, Justification::None),
//                         )],
//                     ),
//                     Box::new(LinePainter1D::new(vec![Pos2::new(0.6, 0.8), Pos2::new(0.65, 0.7), Pos2::new(0.7, 0.8)])),
//                     None,
//                 ),
//                 //------Test Line 2D -----
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("Line 2D"),
//                         vec![DisplayValue::new(
//                             "Test".to_string(),
//                             None,
//                             JustifiedValue::new(None, Justification::None),
//                         )],
//                     ),
//                     Box::new(LinePainter2D::new(vec![
//                         Pos2::new(0.15, 0.8),
//                         Pos2::new(0.15, 0.9),
//                         Pos2::new(0.5, 0.9),
//                         Pos2::new(0.5, 0.8),
//                     ])),
//                     None,
//                 ),
//                 //------Tank Valve---
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("Tank valve"),
//                         vec![DisplayValue::new(
//                             "Test".to_string(),
//                             None,
//                             JustifiedValue::new(None, Justification::None),
//                         )],
//                     ),
//                     Box::new(TankValvePainter::new(ValvePainter::new(Pos2::new(0.75, 0.5), 0.1))),
//                     None,
//                 ),
//                 //------Ball Valve---
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("Ball valve"),
//                         vec![DisplayValue::new(
//                             "Test".to_string(),
//                             None,
//                             JustifiedValue::new(None, Justification::None),
//                         )],
//                     ),
//                     Box::new(BallValvePainter::new(ValvePainter::new(Pos2::new(0.9, 0.5), 0.1))),
//                     None,
//                 ),
//                 //-Temperature Sensor-
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("Temperature Sensor"),
//                         vec![DisplayValue::new(
//                             "Temperature".to_string(),
//                             Some("C".to_string()),
//                             JustifiedValue::new(None, Justification::None),
//                         )],
//                     ),
//                     Box::new(SymbolPainter::new(Pos2::new(0.75, 0.75), 0.02, "T")),
//                     None,
//                 ),
//                 //-Pressure Sensor-
//                 FlowComponent::new(
//                     ComponentInfo::new(
//                         String::from("Pressure Sensor"),
//                         vec![DisplayValue::new(
//                             "Pressure".to_string(),
//                             Some("Bar".to_string()),
//                             JustifiedValue::new(None, Justification::None),
//                         )],
//                     ),
//                     Box::new(SymbolPainter::new(Pos2::new(0.85, 0.75), 0.03, "P")),
//                     None,
//                 ),
//                 //-Burst Disc-
//                 FlowComponent::new(
//                     ComponentInfo::new(String::from("Burst Disc"), vec![]),
//                     Box::new(BurstDiscPainter::new(Pos2::new(0.93, 0.75), 0.06, 0.12)),
//                     None,
//                 ),
//                 //-Flex Tube-
//                 FlowComponent::new(
//                     ComponentInfo::new(String::from("Flex Tube"), vec![]),
//                     Box::new(FlexTubePainter::new(Pos2::new(0.75, 0.25), 0.1, 0.08)),
//                     None,
//                 ),
//                 //-Check valve-
//                 FlowComponent::new(
//                     ComponentInfo::new(String::from("Check valve"), vec![]),
//                     Box::new(CheckValvePainter::new(Pos2::new(0.90, 0.25), 0.1, 0.08)),
//                     None,
//                 ),
//             ],
//             new_test_painter: PainterInstance::new(
//                 crate::flow_components_new::flow_painter::FlowPainter::TankValve(ConnectionState::Connected(Some(FluidType::N2O))),
//                 //crate::flow_components_new::flow_painter::FlowPainter::Missing,
//                 Transform::new(Rotation2::new(270f32.to_radians()), Scale2::new(0.1, 0.1), Translation2::new(0.2, 0.2)))
//         }
//     }
// }

// impl egui::Widget for HybridSystemDiagram {
//     fn ui(self, ui: &mut egui::Ui) -> egui::Response {
//         let res = ui.vertical(|ui| {
//             for comp in self.components {
//                 comp.draw_and_respond(ui);
//             }
//         })
//         .response;
//         let available_space = ui.available_rect_before_wrap();
//         let global_transform = Transform::new(
//             Rotation2::new(0f32),
//             Scale2::new(available_space.width(), available_space.height()),
//             Translation2::new(available_space.min.x, available_space.min.y))
//             .to_affine2();
//         self.new_test_painter.paint(&global_transform, ui.painter(), ThemeColors::new(ui.ctx()));
//         return res;
//     }
// }
