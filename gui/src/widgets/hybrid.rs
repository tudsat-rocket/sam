use egui::{Color32, Pos2, Vec2};

use crate::{flow_components::{line::Line, tank::*}, utils::theme::ThemeColors};

pub struct HybridSystemDiagram {
    n2_tank: Tank,
    n2o_tank: Tank,
    connecting_line: Line
}

impl HybridSystemDiagram {
    pub fn new() -> Self {
        //Maybe this is not the best place for these constants
        let n2_vis_data = PropellantTankVisData::new(Vec2::new(0.15, 0.5), 0.2, 0.6, 1.8, Color32::LIGHT_GREEN);
        let connecting_points= vec![Pos2::new(0.15, 0.8), Pos2::new(0.15, 0.9), Pos2::new(0.5, 0.9), Pos2::new(0.5, 0.8)];
        let n2o_vis_data = PropellantTankVisData::new(Vec2::new(0.5, 0.5), 0.2, 0.6, 1.8, Color32::LIGHT_BLUE);
        Self {
            n2_tank: Tank::new(0.6, n2_vis_data),
            n2o_tank: Tank::new(0.2, n2o_vis_data),
            connecting_line: Line::new(connecting_points),
        }
    }
}

impl egui::Widget for HybridSystemDiagram {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            let painter = ui.painter();
            let theme = ThemeColors::new(ui.ctx());
            let available_space = ui.available_rect_before_wrap();
    
            self.n2_tank.draw(painter, &theme, &available_space);
            self.n2o_tank.draw(painter, &theme, &available_space);
            self.connecting_line.draw(painter, &theme, &available_space);
        }).response
    }
}
