use egui::{Pos2, Vec2};

use crate::flow_components::{flow_component::{ComponentInfo, Value, DisplayValue, FlowComponent, Fluid, Justification, JustifiedValue, N2, N2O, TEST_SENSOR}, line::LinePainter, tank::TankPainter, valve::ValvePainter};

pub struct HybridSystemDiagram {
    components: Vec<FlowComponent>
}

impl HybridSystemDiagram {
    pub fn new() -> Self {

        Self { components: vec![

            //------N2 Tank------
            FlowComponent::new(
                ComponentInfo::new(String::from("N\u{2082} Tank"), vec![
                    DisplayValue::new("Fill".to_string(), Some("%".to_string()), JustifiedValue::new(Some(Value::F32(0.6)), Justification::Measured(TEST_SENSOR))),
                    DisplayValue::new("Test".to_string(), None, JustifiedValue::new(None, Justification::None))
                ]),
                Box::new(TankPainter::new(Vec2::new(0.15, 0.5), 0.2, 0.6, 1.8, 300.0, Fluid{fluid_type: N2, pressure: JustifiedValue::new(Some(Value::F32(180.0)), Justification::Reasoned)})),
                Some("You clicked the N2 tank".to_string())
            ),

            //------N2O Tank-----
            FlowComponent::new(
                ComponentInfo::new(String::from("N\u{2082}O Tank"), vec![
                    DisplayValue::new("Fill".to_string(), Some("%".to_string()), JustifiedValue::new(Some(Value::F32(0.2)), Justification::Measured(TEST_SENSOR)))
                ]),
                Box::new(TankPainter::new(Vec2::new(0.5, 0.5), 0.2, 0.6, 1.8, 60.0, Fluid{fluid_type: N2O, pressure: JustifiedValue::new(Some(Value::F32(15.0)), Justification::Reasoned)})),
                None
            ),

            //------Test Line-----
            FlowComponent::new(
                ComponentInfo::new(String::from("Test Line"), vec![
                    DisplayValue::new("Test".to_string(), None, JustifiedValue::new(None, Justification::None))
                ]),
                Box::new(LinePainter::new(vec![
                    Pos2::new(0.15, 0.8), Pos2::new(0.15, 0.9), Pos2::new(0.5, 0.9), Pos2::new(0.5, 0.8)
                ])),
                None
            ),

            //------Test Valve---
            FlowComponent::new(
                ComponentInfo::new(String::from("Test valve"), vec![
                    DisplayValue::new("Test".to_string(), None, JustifiedValue::new(None, Justification::None))
                ]), 
                Box::new(ValvePainter::new(Vec2::new(0.75, 0.5), 0.2)),
                None
            ),
        
        ]}
        // let connecting_points= vec![Pos2::new(0.15, 0.8), Pos2::new(0.15, 0.9), Pos2::new(0.5, 0.9), Pos2::new(0.5, 0.8)];
        // let n2o_vis_data = PropellantTankVisData::new(Vec2::new(0.5, 0.5), 0.2, 0.6, 1.8, Color32::LIGHT_BLUE);
        // Self {
        //     n2_tank: Tank::new( ,0.6, n2_vis_data),
        //     n2o_tank: Tank::new(String::from("N\u{2082}O"), 0.2, n2o_vis_data),
        //     connecting_line: Line::new(connecting_points),
        // }
    }
}

impl egui::Widget for HybridSystemDiagram {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            for comp in self.components {
                comp.draw_and_respond(ui);
            }
        }).response
    }
}
