use crate::{
    backend::{Backend, storage::static_metrics::debug_name_dyn},
    frontend::{
        Frontend,
        popup_manager::{ContextMenu, PopupContentData, PopupPosition, Tooltip, TriggerBuilder},
    },
    system_diagram_components::{
        core::flow_painter::Line1D, diagrams::hyacinth::SystemComponent, math::transform::Transform,
    },
    utils::mesh::register_textures,
    widgets::plot::Plot,
};
use egui::{Color32, Pos2, Rect, RichText, Sense, Vec2};
use nalgebra::{Rotation2, Scale2, Translation2};

pub trait FlattenInnerResponse<R> {
    //Flatten the outer layer of the response
    fn flatten(self) -> egui::InnerResponse<R>;
}

impl<R> FlattenInnerResponse<R> for egui::InnerResponse<egui::InnerResponse<R>> {
    fn flatten(self) -> egui::InnerResponse<R> {
        return egui::InnerResponse::new(self.inner.inner, self.response.union(self.inner.response));
    }
}

pub trait FlattenResponse {
    //Flatten the outer layer of the response
    fn flatten(self) -> egui::Response;
}

impl FlattenResponse for egui::InnerResponse<egui::Response> {
    fn flatten(self) -> egui::Response {
        return self.response.union(self.inner);
    }
}

impl FlattenResponse for egui::InnerResponse<()> {
    fn flatten(self) -> egui::Response {
        return self.response;
    }
}

impl FlattenResponse for egui::CollapsingResponse<egui::Response> {
    fn flatten(self) -> egui::Response {
        return self.body_response.map(|br| self.header_response.union(br)).unwrap_or(self.header_response);
    }
}

fn as_tooltip<Comp: SystemComponent>(
    comp: &Comp,
    ui: &mut egui::Ui,
    frontend: &mut Frontend,
    backend: &mut Backend,
) -> egui::Response {
    //let theme = &ThemeColors::new(ui.ctx());
    ui.style_mut().interaction.selectable_labels = true;

    let mut tooltip_response = ui
        .horizontal(|ui| {
            let inner_tooltip_response = ui.heading(comp.name());
            // let click_response = match &self.link {
            //     Some(_) => ui.add(Button::image(Image::new(IMG_OPEN_LINK).tint(theme.foreground_weak)).small()),
            //     None => ui.add_enabled(
            //         false,
            //         Button::image(Image::new(IMG_MISSING_FILE).tint(theme.foreground_weak)).small(),
            //     ),
            // };
            // #[cfg(not(target_arch = "wasm32"))]
            // if click_response.clicked() {
            //     self.link.clone().map(|url| open::that(url));
            // }
            return inner_tooltip_response/*.union(click_response)*/;
        })
        .flatten();

    tooltip_response = ui.separator().union(tooltip_response);
    ui.add_space(10.0);

    if comp.metrics().len() > 0 {
        tooltip_response = egui::CollapsingHeader::new(RichText::new("Metrics").strong().underline())
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("tooltip_grid_metrics").striped(true).show(ui, |ui| {
                    for metric in comp.metrics() {
                        let val = backend.current_value_dynamic_as_string(&metric);
                        let name = debug_name_dyn(&metric);

                        tooltip_response = tooltip_response.union(ui.label(name.clone()));
                        tooltip_response = tooltip_response.union(ui.label(val));

                        let bounding_box = Rect::from_min_max(
                            Pos2::new(ui.min_rect().min.x, ui.available_rect_before_wrap().min.y),
                            ui.min_rect().max,
                        );
                        let popup_pos = Pos2::new(
                            frontend.popup_manager().get_active_frame_bounds(ui.min_rect()).max.x,
                            bounding_box.min.y,
                        );
                        let tooltip_id = ui.make_persistent_id(format!("ID Metric {name}"));
                        TriggerBuilder::new(tooltip_id, bounding_box)
                            .add(
                                PopupPosition::Position(popup_pos),
                                PopupContentData::<Tooltip, _>::new(|ui, frontend, backend| {
                                    let config = super::plot::PlotConfig {
                                        lines: vec![(metric, Color32::RED)],
                                        ylimits: (None, None),
                                    };
                                    return ui.add(Plot::new(&config, frontend.shared_plot_mut(), backend));
                                }),
                            )
                            .add(
                                PopupPosition::Position(popup_pos),
                                PopupContentData::<ContextMenu, _>::new(|ui, frontend, _backend| {
                                    return if frontend.metric_monitor().is_pinned(&metric) {
                                        let unpin_response = ui.button("Unpin from Monitor");
                                        if unpin_response.clicked() {
                                            frontend.metric_monitor_mut().unpin(metric);
                                        }
                                        unpin_response
                                    } else {
                                        let pin_response = ui.button("Pin to monitor");
                                        if pin_response.clicked() {
                                            frontend.metric_monitor_mut().pin(metric);
                                        }
                                        pin_response
                                    };
                                }),
                            )
                            .show_active(ui, frontend, backend);
                        ui.end_row();
                    }
                });
            })
            .header_response
            .union(tooltip_response);
    } else {
        tooltip_response = ui.label(RichText::new("No metrics available").italics()).union(tooltip_response);
    }
    return tooltip_response;
}
// }

pub struct SystemDiagram<'a, Comp: SystemComponent> {
    components: Vec<Comp>,
    lines: Vec<Line1D>,
    backend: &'a mut Backend,
    frontend: &'a mut Frontend,
}

impl<'a, Comp: SystemComponent> SystemDiagram<'a, Comp> {
    pub fn init(ctx: &egui::Context) {
        register_textures(ctx);
    }

    pub fn new(
        components: Vec<Comp>,
        lines: Vec<Line1D>,
        backend: &'a mut Backend,
        frontend: &'a mut Frontend,
    ) -> Self {
        Self {
            components,
            lines,
            backend,
            frontend,
        }
    }
}

impl<'a, Comp: SystemComponent> egui::Widget for SystemDiagram<'a, Comp> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let available_space = ui.available_rect_before_wrap();
        let global_transform = Transform::new(
            Rotation2::new(0f32),
            Scale2::new(available_space.width(), available_space.height()),
            Translation2::new(available_space.min.x, available_space.min.y),
        )
        .to_affine2();
        return ui
            .vertical(|ui| {
                for (idx, comp) in self.components.iter().enumerate() {
                    let bounding_box = comp.symbol().paint(&global_transform, ui, self.backend);
                    let trigger_id = ui.make_persistent_id(format!("Base Layer Trigger ID {idx}")); //TODO Improve ID
                    let popup_pos = bounding_box.min + bounding_box.size() * Vec2::new(1.0, 0.5);
                    TriggerBuilder::new(trigger_id, bounding_box)
                        .add(
                            PopupPosition::Position(popup_pos),
                            PopupContentData::<Tooltip, _>::new(|ui, frontend, backend| {
                                let mut tooltip_response =
                                    ui.allocate_response(egui::Vec2::ZERO, egui::Sense::click_and_drag());
                                return ui
                                    .vertical(|ui| {
                                        tooltip_response =
                                            as_tooltip(comp, ui, frontend, backend).union(tooltip_response.clone())
                                    })
                                    .response
                                    .union(tooltip_response);
                            }),
                        )
                        .add(
                            PopupPosition::Position(popup_pos),
                            PopupContentData::<ContextMenu, _>::new(|ui, _frontend, backend| {
                                let mut tooltip_response =
                                    ui.allocate_response(egui::Vec2::ZERO, egui::Sense::click_and_drag());
                                return if comp.interactions().len() > 0 {
                                    ui.vertical(|ui| {
                                        for interaction in comp.interactions() {
                                            let interaction_response = if interaction.is_possible(&backend) {
                                                ui.button(interaction.description(backend))
                                            } else {
                                                ui.allocate_response(Vec2::ZERO, Sense::hover())
                                            };
                                            tooltip_response = interaction_response.union(tooltip_response.clone());
                                            if interaction_response.clicked() {
                                                interaction.interact(backend);
                                            }
                                        }
                                    })
                                    .response
                                    .union(tooltip_response)
                                } else {
                                    ui.label(RichText::new("No intractions available").italics())
                                        .union(tooltip_response)
                                };
                            }),
                        )
                        .show_active(ui, self.frontend, self.backend);
                }
                for line in self.lines {
                    line.paint(&global_transform, ui.painter(), ui.ctx());
                }
            })
            .response;
    }
}
