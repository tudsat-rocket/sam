use crate::{system_diagram_components::{core::flow_painter::{Line1D, Symbol}, math::transform::Transform}, utils::mesh::register_textures};
use egui::RichText;
use nalgebra::{Rotation2, Scale2, Translation2};
pub struct SystemDiagram {
    symbols: Vec<Symbol>,
    lines: Vec<Line1D>,
}


impl SystemDiagram {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new(symbols: Vec<Symbol>, lines: Vec<Line1D>) -> Self {
        Self { symbols, lines }
    }
}


impl egui::Widget for SystemDiagram {
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
                let response = ui.interact(bounding_box, egui::Id::new(format!("Test {i}")), egui::Sense::click());
                if response.hovered() {
                    let pos = response.hover_pos();
                    egui::show_tooltip_at(ui.ctx(), ui.layer_id(), egui::Id::new(format!("Test {i}")), pos.unwrap(), |ui| {
                        ui.vertical(|ui| {
                            if symbol.attached_information().len() == 0 {
                                ui.label(RichText::new("None").italics());
                            } else {
                                for info in symbol.attached_information() {
                                    ui.label(format!("{}", info));
                                }
                            }
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