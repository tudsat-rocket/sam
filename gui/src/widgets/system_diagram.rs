
use crate::{backend::Backend, frontend::{metric_monitor::MetricMonitor, popup_manager::{PopupManager, PopupTrigger}}, system_diagram_components::{core::{constants::{IMG_MISSING_FILE, IMG_OPEN_LINK}, display_value::DisplayValue, flow_painter::{Line1D, Symbol}}, math::transform::Transform}, utils::{mesh::register_textures, theme::ThemeColors}, widgets::plot::{Plot, SharedPlotState}};
use egui::{Button, Color32, Image, Pos2, Rect, RichText, Vec2};
use nalgebra::{Rotation2, Scale2, Translation2};
use telemetry::Metric;


//TODO MOVE 
#[derive(Clone)]
pub struct Component {
    name: String,
    link: Option<String>,
    properties: Vec<DisplayValue>,
    attached_metrics: Vec<Metric>,
}

pub trait Flatten{
    fn flatten(self) -> egui::Response;
}

impl Flatten for egui::InnerResponse<egui::Response> {
    fn flatten(self) -> egui::Response{
        return self.response.union(self.inner);
    }
}

impl Flatten for egui::CollapsingResponse<egui::Response> {
    fn flatten(self) -> egui::Response{
        return self.body_response.map(|br| self.header_response.union(br)).unwrap_or(self.header_response);
    }
}

impl Component {
    pub fn new(name: String, link: Option<String>, properties: Vec<DisplayValue>, attached_metrics: Vec<Metric>) -> Self {
        Self { name, link, properties, attached_metrics }
    }

    fn as_tooltip(&self, ui: &mut egui::Ui, backend: &Backend, shared_plot_state: &mut SharedPlotState, popup_manager: &mut PopupManager, metric_monitor: &mut MetricMonitor) -> egui::Response {
        let theme = &ThemeColors::new(ui.ctx());
        ui.style_mut().interaction.selectable_labels = true;

        let mut tooltip_response = ui.horizontal(|ui|{
            let inner_tooltip_response = ui.heading(self.name.clone());
            let click_response = match &self.link {
                Some(_) => ui.add(Button::image(Image::new(IMG_OPEN_LINK).tint(theme.foreground_weak)).small()),
                None => ui.add_enabled(false, Button::image(Image::new(IMG_MISSING_FILE).tint(theme.foreground_weak)).small()),
            };
            #[cfg(not(target_arch = "wasm32"))]
            if click_response.clicked() {
                self.link.clone().map(|url| open::that(url));
            }
            return inner_tooltip_response.union(click_response);
        }).flatten();
    
        tooltip_response = ui.separator().union(tooltip_response);
        ui.add_space(10.0);

        tooltip_response = egui::CollapsingHeader::new(RichText::new("Metrics").strong().underline()).default_open(true).show(ui, |ui| {
            egui::Grid::new("tooltip_grid_metrics").striped(true).show(ui, |ui| {
                for metric in &self.attached_metrics {
                    let val = backend.current_value(*metric).map(|v| format!("{0:.2}", v)).unwrap_or("N/A".to_string());
                    let (name, unit) = match metric {
                        Metric::Pressure(_) => ("Pressure", "bar"),
                        Metric::Temperature(_) => ("Temperature", "°C"),
                        _ => todo!(),
                    };
            
                    tooltip_response = tooltip_response.union(ui.label(name));
                    tooltip_response = tooltip_response.union(ui.label(val));
                    tooltip_response = tooltip_response.union(ui.label(unit));
                
                    let bounding_box = Rect::from_min_max(Pos2::new(ui.min_rect().min.x, ui.available_rect_before_wrap().min.y), ui.min_rect().max);
                    let tooltip_pos = Pos2::new(popup_manager.get_active_frame_bounds(ui.min_rect()).max.x, bounding_box.min.y);
                    let tooltip_id = ui.make_persistent_id(format!("ID Metric {name}"));
                    let trigger = PopupTrigger::new(ui, tooltip_id, bounding_box);
                    popup_manager.add_tooltip(&trigger, tooltip_pos, ui, |ui, _popup_manager| {
                        let inner_tooltip_response = ui.label(format!("Justification: Missing"));
                        let config = super::plot::PlotConfig {
                            lines: vec![
                                (*metric, Color32::RED),
                            ],
                            ylimits: (None, None),
                        };
                        return ui.add(Plot::new(&config, shared_plot_state, backend)).union(inner_tooltip_response);
                    });
                    popup_manager.add_context_menu(&trigger, tooltip_pos, ui, |ui, _popup_manager| {
                        #[cfg(target_arch = "wasm32")]
                        let to_clipboard_response = ui.add_enabled(false, egui::Button::new("Copy to clipboard"));
                        #[cfg(not(target_arch = "wasm32"))]
                        let to_clipboard_response = ui.button("Copy to clipboard");
                        #[cfg(not(target_arch = "wasm32"))]
                        if to_clipboard_response.clicked() {
                            let mut clipboard =  clippers::Clipboard::get();
                            let _ = clipboard.write_text(backend.current_value(*metric).map(|v| format!("{}", v)).unwrap_or("N/A".to_string()));
                        }
                        let pin_or_unpin_response = 
                        if metric_monitor.is_pinned(metric) {
                            let unpin_response = ui.button("Unpin from Monitor");
                            if unpin_response.clicked() {
                                metric_monitor.unpin(*metric);
                            }
                            unpin_response
                        } else {
                            let pin_response = ui.button("Pin to monitor");
                            if pin_response.clicked() {
                                metric_monitor.pin(*metric);
                            }
                            pin_response
                        };
                        return to_clipboard_response.union(pin_or_unpin_response);
                    });
                    tooltip_response = tooltip_response.union(trigger.response());
                    ui.end_row();
                }
            });
        }).header_response.union(tooltip_response);

        tooltip_response = egui::CollapsingHeader::new(RichText::new("Properties").strong().underline()).show(ui, |ui| {
            egui::Grid::new("tooltip_grid_properties").striped(true).show(ui, |ui| {
                for property in &self.properties {
                    tooltip_response = tooltip_response.union(ui.label(property.desc.clone()));
                    tooltip_response = tooltip_response.union(ui.label(property.val.value.clone().map(|v| format!("{v}")).unwrap_or("N/A".to_string())));
                    tooltip_response = tooltip_response.union(ui.label(property.unit.clone().unwrap_or("N/A".to_string())));

                    let bounding_box = Rect::from_min_max(Pos2::new(ui.min_rect().min.x, ui.available_rect_before_wrap().min.y), ui.min_rect().max);
                    let tooltip_pos = Pos2::new(popup_manager.get_active_frame_bounds(ui.min_rect()).max.x, bounding_box.min.y);
                    let tooltip_id = ui.make_persistent_id(format!("ID Metric {}", property.desc));
                    let trigger = PopupTrigger::new(ui, tooltip_id, bounding_box);
                    popup_manager.add_tooltip(&trigger, tooltip_pos, ui, |ui, _popup_manager| {
                        return ui.label(format!("Justification: Missing"));
                    });
                    popup_manager.add_context_menu(&trigger, tooltip_pos, ui, |ui, _popup_manager| {
                        #[cfg(target_arch = "wasm32")]
                        let to_clipboard_response = ui.add_enabled(false, egui::Button::new("Copy to clipboard"));
                        #[cfg(not(target_arch = "wasm32"))]
                        let to_clipboard_response = ui.button("Copy to clipboard");
                        #[cfg(not(target_arch = "wasm32"))]
                        if to_clipboard_response.clicked() {
                            let mut clipboard =  clippers::Clipboard::get();
                            let _ = clipboard.write_text(property.val.value.clone().map(|v| format!("{v}")).unwrap_or("N/A".to_string()));
                        }
                        return to_clipboard_response;
                    });
                    tooltip_response = tooltip_response.union(trigger.response());
                    ui.end_row();
                }
            });
        }).header_response.union(tooltip_response);
        return tooltip_response;
    }
}


pub struct SystemDiagram<'a> {
    symbols: Vec<Symbol>,
    lines: Vec<Line1D>,
    backend: &'a Backend,
    shared_plot_state: &'a mut SharedPlotState,
    popup_manager: &'a mut PopupManager,
    metric_monitor: &'a mut MetricMonitor,
}


impl<'a> SystemDiagram<'a> {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new(symbols: Vec<Symbol>, lines: Vec<Line1D>, backend: &'a Backend, shared_plot_state: &'a mut SharedPlotState, popup_manager: &'a mut PopupManager, metric_monitor: &'a mut MetricMonitor) -> Self {
        Self { symbols, lines, backend, shared_plot_state, popup_manager, metric_monitor }
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
            for (idx, symbol) in self.symbols.iter().enumerate() {
                let bounding_box= symbol.paint(&global_transform, ui.painter(), ui.ctx());
                let trigger_id = ui.make_persistent_id(format!("Base Layer Trigger ID {idx}")); //TODO Improve ID
                let popup_trigger = PopupTrigger::new(ui, trigger_id, bounding_box);
                let popup_pos = bounding_box.min + bounding_box.size() * Vec2::new(1.0, 0.5);
                self.popup_manager.add_tooltip(&popup_trigger, popup_pos, ui, |ui, popup_manager| {
                    let mut tooltip_response = ui.allocate_response(egui::Vec2::ZERO, egui::Sense::click_and_drag());
                    return ui.vertical(|ui| {
                        tooltip_response = match symbol.component() {
                            Some(component) => component.as_tooltip(ui, self.backend, self.shared_plot_state, popup_manager, self.metric_monitor),                       
                            None =>  ui.label(RichText::new("N/A").italics()),
                        }.union(tooltip_response.clone())
                    }).response.union(tooltip_response);
                });
            }
            for line in self.lines{
                line.paint(&global_transform, ui.painter(), ui.ctx());
            }
        })
        .response;
    }
}