use std::time::{Duration, Instant};

use crate::{backend::Backend, system_diagram_components::{core::{constants::{GAMMA_MUL_ON_HOVER, TOOLTIP_DURATION}, display_value::DisplayValue, flow_painter::{Line1D, Symbol}}, math::transform::Transform}, utils::mesh::register_textures, widgets::plot::{Plot, SharedPlotState}};
use egui::{Color32, Id, Pos2, Rect, RichText, Sense, Vec2};
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

    fn as_tooltip(&self, ui: &mut egui::Ui, backend: &Backend, shared_plot_state: &mut SharedPlotState, parent_id: Id) {
        ui.heading(self.name.clone());
        ui.separator();
        ui.add_space(10.0);

        egui::CollapsingHeader::new(RichText::new("Metrics").strong().underline())
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("tooltip_grid_metrics")
                    .striped(true)
                    .show(ui, |ui| {
                        for metric in &self.attached_metrics {
                            let val = backend.current_value(*metric).map(|v| format!("{0:.2}", v)).unwrap_or("N/A".to_string());
                            let (name, unit) = match metric {
                                Metric::Pressure(_) => ("Pressure", "bar"),
                                Metric::Temperature(_) => ("Temperature", "°C"),
                                _ => todo!(),
                            };
                            ui.label(name);
                            ui.label(val);
                            ui.label(unit);
                        
                            let parent_id_string = parent_id.short_debug_format();
                            let tooltip_id = ui.make_persistent_id(format!("{name}, {parent_id_string}"));// egui::Id::new(name);
                            let is_tooltip_open = ui.ctx().data_mut(|d| d.get_temp::<Instant>(tooltip_id)).map(|inst| inst.elapsed().as_secs_f32() < TOOLTIP_DURATION).unwrap_or(false);
                            let row_response = ui.interact(Rect::from_min_max(Pos2::new(ui.min_rect().min.x, ui.available_rect_before_wrap().min.y), ui.min_rect().max), tooltip_id, egui::Sense::click());
                            let is_hovered = row_response.hovered();
                            if is_hovered || is_tooltip_open {
                                ui.painter().rect_filled(
                                    row_response.rect, 
                                    0.0, 
                                    ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
                                );
                                let nested_tooltip_pos = Pos2::new(row_response.rect.max.x, row_response.rect.min.y);
                                egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip_id, nested_tooltip_pos, |ui| {
                                    ui.vertical(|ui| {
                                    ui.style_mut().interaction.selectable_labels = true;
                                    ui.label(format!("Justification: Missing"));
                                        let config = super::plot::PlotConfig {
                                            lines: vec![
                                                (*metric, Color32::RED),
                                            ],
                                            ylimits: (None, None),
                                        };
                                        ui.add(Plot::new(&config, shared_plot_state, backend));
                                    });
                                    let tooltip_id_string = tooltip_id.short_debug_format();
                                    let tooltip_response = ui.interact(ui.max_rect(), egui::Id::new(format!("Nested with parent {tooltip_id_string}")), Sense::hover());
                                    let is_tooltip_hovered = tooltip_response.hovered();
                                    //The entry in the IdTypeMap is never deleted, we could do that if necessary
                                    if is_hovered || is_tooltip_hovered {
                                        ui.ctx().data_mut(|d| d.insert_temp(parent_id, Instant::now()));
                                        ui.ctx().data_mut(|d| d.insert_temp(tooltip_id, Instant::now()));
                                    }
                                });
                                ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
                            }   
                        ui.end_row();
                    }
                });
        });
        egui::CollapsingHeader::new(RichText::new("Properties").strong().underline())
            .show(ui, |ui| {
                egui::Grid::new("tooltip_grid_properties")
                    .striped(true)
                    .show(ui, |ui| {
                        for property in &self.properties {
                            ui.label(property.desc.clone());
                            ui.label(property.val.value.clone().map(|v| format!("{v}")).unwrap_or("N/A".to_string()));
                            ui.label(property.unit.clone().unwrap_or("N/A".to_string()));

                            let row_response = ui.interact(Rect::from_min_max(Pos2::new(ui.min_rect().min.x, ui.available_rect_before_wrap().min.y), ui.min_rect().max), egui::Id::new(property.desc.clone()), egui::Sense::click());
                            if row_response.hovered() {
                                    ui.painter().rect_filled(
                                    row_response.rect, 
                                    0.0, 
                                    ui.visuals().weak_text_color().gamma_multiply(GAMMA_MUL_ON_HOVER)
                                );
                                let nested_tooltip_pos = Pos2::new(row_response.rect.max.x, row_response.rect.min.y);
                                egui::show_tooltip_at(ui.ctx(), ui.layer_id(), row_response.id, nested_tooltip_pos, |ui| {
                                    ui.vertical(|ui| {
                                        ui.label(format!("Justification: Missing"));
                                    });
                                });
                            }
                            ui.end_row();
                        }
                    })
            });
    }
}


pub struct SystemDiagram<'a> {
    symbols: Vec<Symbol>,
    lines: Vec<Line1D>,
    backend: &'a Backend,
    shared_plot_state: &'a mut SharedPlotState,
}


impl<'a> SystemDiagram<'a> {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new(symbols: Vec<Symbol>, lines: Vec<Line1D>, backend: &'a Backend, shared_plot_state: &'a mut SharedPlotState) -> Self {
        Self { symbols, lines, backend, shared_plot_state }
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
                let tooltip_id = ui.make_persistent_id(format!("ID {i}")); //TODO Improve ID
                let response = ui.interact(bounding_box, egui::Id::new(tooltip_id), egui::Sense::hover());
                let is_hovered = response.hovered();
                let is_tooltip_open = ui.ctx().data_mut(|d| d.get_temp::<Instant>(tooltip_id)).map(|inst| inst.elapsed().as_secs_f32() < TOOLTIP_DURATION).unwrap_or(false);

                if is_hovered || is_tooltip_open {
                    let tooltip_pos = bounding_box.min + bounding_box.size() * Vec2::new(0.75, 0.75);
                    egui::show_tooltip_at(ui.ctx(), ui.layer_id(), tooltip_id, tooltip_pos, |ui| {
                        let tooltip_response = ui.interact(ui.max_rect(), tooltip_id, Sense::click_and_drag());
                        let is_tooltip_hovered = tooltip_response.hovered();
                        //The entry in the IdTypeMap is never deleted, we could do that if necessary
                        if is_hovered || is_tooltip_hovered {
                            ui.ctx().data_mut(|d| d.insert_temp(tooltip_id, Instant::now()));
                        }
                        ui.vertical(|ui| {
                            match symbol.component() {
                                Some(component) => component.as_tooltip(ui, self.backend, self.shared_plot_state, tooltip_id),                       
                                None =>  {ui.label(RichText::new("N/A").italics());},
                            }
                        });
                    });
                    ui.ctx().request_repaint_after(Duration::from_secs_f32(TOOLTIP_DURATION));
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