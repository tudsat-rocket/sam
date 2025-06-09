use crate::{backend::Backend, system_diagram_components::{core::{display_value::DisplayValue, flow_painter::{Line1D, Symbol}}, math::transform::Transform}, utils::mesh::register_textures};
use egui::RichText;
use nalgebra::{Rotation2, Scale2, Translation2};
use telemetry::Metric;


//TODO MOVE 
#[derive(Clone)]
pub struct Component {
    name: String,
    properties: Vec<DisplayValue>,
    attached_metrics: Vec<Metric>,
}

impl Component {
    pub fn new(name: String, properties: Vec<DisplayValue>, attached_metrics: Vec<Metric>) -> Self {
        Self { name, properties, attached_metrics }
    }

    fn as_tooltip(&self, ui: &mut egui::Ui, backend: &Backend) {
        ui.heading(self.name.clone());
        ui.separator();
        ui.add_space(10.0);

        egui::Grid::new("tooltip_grid")
            .striped(true)
            .show(ui, |ui| {

                ui.label(RichText::new("Metrics").strong().underline());
                ui.end_row();
                for metric in &self.attached_metrics {
                    let val = backend.current_value(*metric).map(|v| format!("{}", v)).unwrap_or("N/A".to_string());
                    let (name, unit) = match metric {
                        Metric::Pressure(_) => ("Pressure", "Bar"),
                        _ => todo!(),
                    };
                    ui.label(name);
                    ui.label(val);
                    ui.label(unit);
                    ui.end_row();
                }
                ui.end_row();


                ui.label(RichText::new("Properties").strong().underline());
                ui.end_row();
                for property in &self.properties {
                    ui.label(property.desc.clone());
                    ui.label(property.val.value.clone().map(|v| format!("{v}")).unwrap_or("N/A".to_string()));
                    ui.label(property.unit.clone().unwrap_or("N/A".to_string()));
                }
            });
    }
}


pub struct SystemDiagram<'a> {
    symbols: Vec<Symbol>,
    lines: Vec<Line1D>,
    backend: &'a Backend,
}


impl<'a> SystemDiagram<'a> {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new(symbols: Vec<Symbol>, lines: Vec<Line1D>, backend: &'a Backend) -> Self {
        Self { symbols, lines, backend }
    }
}


impl<'a> egui::Widget for SystemDiagram<'a> {
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
                            match symbol.component() {
                                Some(component) => component.as_tooltip(ui, self.backend),                       
                                None =>  {ui.label(RichText::new("None").italics());},
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