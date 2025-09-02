use egui::{Align2, Color32, FontId, Stroke, Vec2, epaint::CircleShape};
use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{
    system_diagram_components::math::{
        conversions::{to_pos, to_vec},
        transform::Transform,
        utils::get_basis_x,
    },
    utils::theme::ThemeColors,
};

pub struct Milestone {
    label: &'static str,
}

impl Milestone {
    pub fn new(label: &'static str) -> Self {
        Self { label }
    }
}

pub struct Timeline {
    milestones: Vec<Milestone>,
    num_milestones_reached: usize,
    rotation: Rotation2<f32>,
}

static RELATIVE_PADDING_HORIZONAL: f32 = 0.03;
static ABSOLUTE_CIRCLE_RADIUS: f32 = 8.0;
static ABSOLUTE_LINE_WIDTH_REACHED: f32 = 5.0;
static ABSOLUTE_LINE_WIDTH_PENDING: f32 = 2.0;
static ABSOLUTE_LINE_TO_STATE_GAP: f32 = 2.0;

impl Timeline {
    pub fn new(milestones: Vec<Milestone>, num_milestones_reached: usize, rotation: Rotation2<f32>) -> Self {
        Self {
            milestones,
            num_milestones_reached,
            rotation,
        }
    }

    pub fn paint(&self, transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
        let distance_between_milestones = if self.milestones.len() > 0 {
            (1f32 - 2f32 * RELATIVE_PADDING_HORIZONAL) / ((self.milestones.len() - 1) as f32)
        } else {
            0.0
        };
        let positions = std::iter::successors(Some(Point2::new(-0.5f32 + RELATIVE_PADDING_HORIZONAL, 0.0)), |p| {
            Some(Point2::new(p.x + distance_between_milestones, p.y))
        })
        .take(self.milestones.len())
        .map(|p| to_pos(transform * p))
        .collect::<Vec<_>>();

        for (milestone, p) in positions.iter().enumerate() {
            if milestone > 0 {
                painter.line(
                    vec![
                        positions[milestone - 1] + self.get_milestone_line_offset(transform, milestone - 1),
                        *p - self.get_milestone_line_offset(transform, milestone),
                    ],
                    self.get_milestone_line_stroke(milestone, theme),
                );
            }
            painter.add(egui::Shape::Circle(CircleShape {
                center: *p,
                radius: ABSOLUTE_CIRCLE_RADIUS,
                fill: self.get_milestone_state_color(milestone, theme),
                stroke: self.get_milestone_state_stroke(milestone, theme),
            }));

            let label_layout = painter.layout_no_wrap(
                self.milestones[milestone].label.to_string(),
                FontId::default(),
                theme.foreground_weak,
            );

            painter.text(
                *p + Vec2::new(0f32, label_layout.size().y + 5f32),
                Align2::CENTER_CENTER,
                label_layout.text(),
                FontId::default(),
                theme.foreground_weak,
            );
        }
    }

    fn get_milestone_line_offset(&self, transform: &Affine2<f32>, milestone: usize) -> Vec2 {
        return if milestone < self.num_milestones_reached {
            (ABSOLUTE_LINE_TO_STATE_GAP + ABSOLUTE_CIRCLE_RADIUS + ABSOLUTE_LINE_WIDTH_REACHED)
                * to_vec(get_basis_x(transform)).normalized()
        } else {
            (ABSOLUTE_LINE_TO_STATE_GAP + ABSOLUTE_CIRCLE_RADIUS + ABSOLUTE_LINE_WIDTH_REACHED)
                * to_vec(get_basis_x(transform)).normalized()
        };
    }

    fn get_milestone_state_color(&self, milestone: usize, theme: &ThemeColors) -> Color32 {
        return if milestone < self.num_milestones_reached {
            theme.foreground_weak
        } else {
            theme.background_weak
        };
    }

    fn get_milestone_state_stroke(&self, milestone: usize, theme: &ThemeColors) -> Stroke {
        return if milestone < self.num_milestones_reached {
            egui::Stroke {
                width: ABSOLUTE_LINE_WIDTH_REACHED,
                color: theme.foreground_weak,
            }
        } else {
            egui::Stroke {
                width: ABSOLUTE_LINE_WIDTH_PENDING,
                color: theme.foreground_weak,
            }
        };
    }

    fn get_milestone_line_stroke(&self, milestone: usize, theme: &ThemeColors) -> Stroke {
        return if milestone < self.num_milestones_reached {
            egui::Stroke {
                width: ABSOLUTE_LINE_WIDTH_REACHED,
                color: theme.foreground_weak,
            }
        } else {
            egui::Stroke {
                width: ABSOLUTE_LINE_WIDTH_PENDING,
                color: theme.foreground_weak,
            }
        };
    }
}

impl egui::Widget for Timeline {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let (available_space, response) = ui.allocate_at_least(
            Vec2::new(
                ui.available_rect_before_wrap().width(),
                3f32 * (ABSOLUTE_CIRCLE_RADIUS + ABSOLUTE_LINE_WIDTH_REACHED + 30f32),
            ),
            egui::Sense::click_and_drag(),
        );
        //let available_space = ui.available_rect_before_wrap();
        let global_transform = Transform::new(
            Rotation2::new(0f32),
            Scale2::new(available_space.width(), available_space.height()),
            Translation2::new(available_space.min.x, available_space.min.y),
        )
        .to_affine2()
            * Transform::new(self.rotation, Scale2::identity(), Translation2::new(0.5, 0.5)).to_affine2();
        self.paint(&global_transform, ui.painter(), &ThemeColors::new(ui.ctx()));
        //let response = ui.interact(available_space, egui::Id::new("blabliblu"), egui::Sense::click_and_drag());
        return response;
    }
}
