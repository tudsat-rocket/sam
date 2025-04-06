use egui::{Color32, Vec2};

use crate::flow_components::{flow_component::{ComponentInfo, DataPoint, FlowComponent}, tank::Tank};

pub struct HybridSystemDiagram {
    components: Vec<FlowComponent>
}

impl HybridSystemDiagram {
    pub fn new() -> Self {

        let n2_tank = FlowComponent::new(
            ComponentInfo::new(String::from("N\u{2082} Tank"), vec![
                DataPoint::new("Fill".to_string(), Some("%".to_string()), Some(0.6)),
                DataPoint::new("Test".to_string(), None, None)
            ]),
            Box::new(Tank::new(Vec2::new(0.15, 0.5), 0.2, 0.6, 1.8, Color32::LIGHT_GREEN)));
        let n2o_tank = FlowComponent::new(
            ComponentInfo::new(String::from("N\u{2082}O Tank"), vec![
                DataPoint::new("Fill".to_string(), Some("%".to_string()), Some(0.2))
            ]),
            Box::new(Tank::new(Vec2::new(0.5, 0.5), 0.2, 0.6, 1.8, Color32::LIGHT_BLUE)));
        
        Self { components: vec![n2_tank, n2o_tank] }
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
