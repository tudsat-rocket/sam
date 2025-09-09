use egui::{Align2, Color32, FontId, InnerResponse, Layout, Rect, Stroke, Vec2, epaint::CircleShape};
use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{
    backend::Backend,
    frontend::{
        Frontend,
        popup_manager::{ModalDialog, Tooltip, TriggerBuilder},
    },
    system_diagram_components::math::{
        conversions::{to_pos, to_vec},
        transform::Transform,
        utils::get_basis_x,
    },
    utils::theme::ThemeColors,
    widgets::system_diagram::FlattenResponse,
};

pub static COLOR_NOMINAL_ON_GROUND: Color32 = Color32::from_rgb(0x45, 0x85, 0x88);
pub static COLOR_NOMINAL_IN_FLIGHT: Color32 = Color32::from_rgb(0xfe, 0x80, 0x19);
pub static COLOR_NOMINAL_IGNITION: Color32 = Color32::from_rgb(0xb1, 0x62, 0x86);
pub static COLOR_HOLD: Color32 = Color32::from_rgb(0xd5, 0xc4, 0xa1);
pub static COLOR_ABORT: Color32 = Color32::from_rgb(0xcc, 0x24, 0x1d);

static RELATIVE_PADDING_HORIZONAL: f32 = 0.03;
static ABSOLUTE_CIRCLE_RADIUS: f32 = 8.0;
static ABSOLUTE_LINE_WIDTH_REACHED: f32 = 5.0;
static ABSOLUTE_LINE_WIDTH_PENDING: f32 = 2.0;
static ABSOLUTE_LINE_TO_STATE_GAP: f32 = 2.0;

pub struct TimelineState {
    label: &'static str,
    color: Color32,
    is_active: bool,
}

impl TimelineState {
    pub fn new(label: &'static str, color: Color32) -> Self {
        Self {
            label,
            color,
            is_active: false,
        }
    }

    fn fill(&self, bg_color: Color32) -> Color32 {
        return if self.is_active { self.color } else { bg_color };
    }

    fn stroke(&self) -> egui::Stroke {
        return if self.is_active {
            egui::Stroke {
                width: ABSOLUTE_LINE_WIDTH_REACHED,
                color: self.color,
            }
        } else {
            egui::Stroke {
                width: ABSOLUTE_LINE_WIDTH_PENDING,
                color: self.color,
            }
        };
    }

    ///The real diameter of the circle including the stroke
    fn real_diameter(&self) -> f32 {
        return 2f32 * (ABSOLUTE_CIRCLE_RADIUS + self.stroke().width);
    }
}

pub struct Timeline<'a> {
    milestones: Vec<TimelineState>,
    anomalous_states: Vec<TimelineState>,
    rotation: Rotation2<f32>,
    frontend: &'a mut Frontend,
    backend: &'a mut Backend,
}

impl<'a> Timeline<'a> {
    pub fn new(
        mut milestones: Vec<TimelineState>,
        anomalous_states: Vec<TimelineState>,
        num_milestones_reached: usize,
        rotation: Rotation2<f32>,
        frontend: &'a mut Frontend,
        backend: &'a mut Backend,
    ) -> Self {
        milestones.iter_mut().enumerate().for_each(|(i, m)| m.is_active = i < num_milestones_reached);

        Self {
            milestones,
            anomalous_states,
            rotation,
            frontend,
            backend,
        }
    }

    pub fn paint(&mut self, transform: &Affine2<f32>, ui: &mut egui::Ui) {
        let vertical_offset = if self.anomalous_states.len() == 0 { 0f32 } else { 0.35 };
        let line_offset = (ABSOLUTE_LINE_TO_STATE_GAP + ABSOLUTE_CIRCLE_RADIUS + ABSOLUTE_LINE_WIDTH_REACHED)
            * to_vec(get_basis_x(transform)).normalized();
        let distance_between_milestones = if self.milestones.len() > 0 {
            (1f32 - 2f32 * RELATIVE_PADDING_HORIZONAL) / ((self.milestones.len() - 1) as f32)
        } else {
            0.0
        };
        let bg_color = ThemeColors::new(ui.ctx()).background_weak;
        let fg_color = ThemeColors::new(ui.ctx()).foreground_weak;

        let milestone_positions = std::iter::successors(
            Some(Point2::new(-0.5f32 + RELATIVE_PADDING_HORIZONAL, vertical_offset * 0.3f32)),
            |p| Some(Point2::new(p.x + distance_between_milestones, p.y)),
        )
        .take(self.milestones.len())
        .map(|p| to_pos(transform * p))
        .collect::<Vec<_>>();

        let lines = milestone_positions
            .windows(2)
            .map(|ps| Some((ps[0] + line_offset, ps[1] - line_offset)))
            .chain(std::iter::once(None))
            .collect::<Vec<_>>();

        for ((milestone, p), l) in std::iter::zip(std::iter::zip(self.milestones.iter(), milestone_positions), lines) {
            let modal_dialog_size = ui.ctx().screen_rect().size() * Vec2::new(0.3, 0.15);
            TriggerBuilder::new(
                egui::Id::new(format!("FlightState {}", milestone.label)),
                Rect::from_center_size(p, Vec2::new(milestone.real_diameter(), milestone.real_diameter())),
            )
            .add::<ModalDialog, _>(ui.ctx().screen_rect().center() - modal_dialog_size / 2f32, |ui, _, _| {
                let mut button_clicked = false;
                let response = ui
                    .allocate_ui_with_layout(modal_dialog_size, Layout::top_down(egui::Align::Center), |ui| {
                        ui.heading("Confirm State Transition");
                        ui.separator();
                        ui.allocate_space(Vec2::new(0f32, 10f32));
                        ui.label(format!("Are you sure you want to enter the state {}? ", milestone.label));
                        ui.horizontal(|ui| {
                            ui.allocate_space(modal_dialog_size / 3f32);
                            if ui.button("Proceed").clicked() {
                                button_clicked = true
                            };
                            ui.allocate_space(Vec2::new(50f32, 0f32));
                            if ui.button("Cancel").clicked() {
                                button_clicked = true;
                            };
                            ui.allocate_space(modal_dialog_size / 3f32);
                        });
                    })
                    .flatten();
                return InnerResponse::new(button_clicked, response);
            })
            .show_active(ui, self.frontend, self.backend);

            let painter = ui.painter();
            l.map(|(start, end)| {
                painter.line(
                    vec![start, end],
                    if milestone.is_active {
                        Stroke::new(ABSOLUTE_LINE_WIDTH_REACHED, fg_color)
                    } else {
                        Stroke::new(ABSOLUTE_LINE_WIDTH_PENDING, fg_color)
                    },
                )
            });
            painter.add(egui::Shape::Circle(CircleShape {
                center: p,
                radius: ABSOLUTE_CIRCLE_RADIUS,
                fill: milestone.fill(bg_color),
                stroke: milestone.stroke(),
            }));

            let label_layout = painter.layout_no_wrap(milestone.label.to_string(), FontId::default(), fg_color);
            painter.text(
                p + Vec2::new(0f32, label_layout.size().y + 5f32),
                Align2::CENTER_CENTER,
                label_layout.text(),
                FontId::default(),
                fg_color,
            );
        }

        let anomalous_state_positions =
            std::iter::successors(Some(Point2::new(-0.5f32 + RELATIVE_PADDING_HORIZONAL, -vertical_offset)), |p| {
                Some(Point2::new(p.x + distance_between_milestones, p.y))
            })
            .take(self.anomalous_states.len())
            .map(|p| to_pos(transform * p))
            .collect::<Vec<_>>();

        for (anomalous_state, p) in std::iter::zip(self.anomalous_states.iter(), anomalous_state_positions) {
            TriggerBuilder::new(
                egui::Id::new(format!("FlightState {}", anomalous_state.label)),
                Rect::from_center_size(p, Vec2::new(anomalous_state.real_diameter(), anomalous_state.real_diameter())),
            )
            .add::<Tooltip, _>(p, |ui, _, _| {
                return ui.label(anomalous_state.label);
            })
            .show_active(ui, self.frontend, self.backend);

            let painter = ui.painter();
            painter.add(egui::Shape::Circle(CircleShape {
                center: p,
                radius: ABSOLUTE_CIRCLE_RADIUS,
                fill: anomalous_state.fill(bg_color),
                stroke: anomalous_state.stroke(),
            }));

            let label_layout = painter.layout_no_wrap(anomalous_state.label.to_string(), FontId::default(), fg_color);

            painter.text(
                p + Vec2::new(0f32, label_layout.size().y + 5f32),
                Align2::CENTER_CENTER,
                label_layout.text(),
                FontId::default(),
                fg_color,
            );
        }
    }
}

impl<'a> egui::Widget for Timeline<'a> {
    fn ui(mut self, ui: &mut egui::Ui) -> egui::Response {
        let (available_space, response) = ui.allocate_at_least(
            Vec2::new(
                ui.available_rect_before_wrap().width(),
                2.6f32 * (ABSOLUTE_CIRCLE_RADIUS + ABSOLUTE_LINE_WIDTH_REACHED + 30f32),
            ),
            egui::Sense::click_and_drag(),
        );
        let global_transform = Transform::new(
            Rotation2::new(0f32),
            Scale2::new(available_space.width(), available_space.height()),
            Translation2::new(available_space.min.x, available_space.min.y),
        )
        .to_affine2()
            * Transform::new(self.rotation, Scale2::identity(), Translation2::new(0.5, 0.5)).to_affine2();
        self.paint(&global_transform, ui);
        return response;
    }
}
