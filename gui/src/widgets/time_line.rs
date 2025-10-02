use egui::{Align2, Color32, FontId, Layout, Pos2, Rect, Stroke, Vec2, epaint::CircleShape};
use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};
use shared_types::{Command, FlightMode};
use strum::IntoStaticStr;

use crate::{
    backend::Backend,
    frontend::{
        Frontend,
        popup_manager::{ModalDialog, PopupContentData, PopupContentList, TriggerBuilder},
    },
    system_diagram_components::math::{
        conversions::{to_pos, to_vec},
        transform::Transform,
        utils::get_basis_x,
    },
    utils::theme::ThemeColors,
    widgets::system_diagram::FlattenResponse,
};

#[derive(Clone, Copy, PartialEq, PartialOrd, IntoStaticStr)]
pub enum HyacinthNominalState {
    Verification,
    IdlePassivated,
    N2Filling,
    IdleActive,
    HarwareArmed,
    N2OFilling,
    SoftwareArmed,
    Ignition,
    BurnPhase,
    CoastPhase,
    DroguePhase,
    MainPhase,
    LandedActive,
    Passivation,
    LandedPassivated,
}

impl HyacinthNominalState {
    fn associated_flight_mode(&self) -> FlightMode {
        match self {
            HyacinthNominalState::Verification => FlightMode::Idle,
            HyacinthNominalState::IdlePassivated => FlightMode::Idle,
            HyacinthNominalState::N2Filling => FlightMode::Idle,
            HyacinthNominalState::IdleActive => FlightMode::Idle,
            HyacinthNominalState::HarwareArmed => FlightMode::HardwareArmed,
            HyacinthNominalState::N2OFilling => FlightMode::HardwareArmed,
            HyacinthNominalState::SoftwareArmed => FlightMode::Armed,
            HyacinthNominalState::Ignition => FlightMode::Armed,
            HyacinthNominalState::BurnPhase => FlightMode::Burn,
            HyacinthNominalState::CoastPhase => FlightMode::Coast,
            HyacinthNominalState::DroguePhase => FlightMode::RecoveryDrogue,
            HyacinthNominalState::MainPhase => FlightMode::RecoveryMain,
            HyacinthNominalState::LandedActive => FlightMode::Landed,
            HyacinthNominalState::Passivation => FlightMode::Landed,
            HyacinthNominalState::LandedPassivated => FlightMode::Landed,
        }
    }

    fn associated_color(&self) -> Color32 {
        match self {
            HyacinthNominalState::Verification => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::IdlePassivated => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::N2Filling => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::IdleActive => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::HarwareArmed => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::N2OFilling => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::SoftwareArmed => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::Ignition => COLOR_NOMINAL_IGNITION,
            HyacinthNominalState::BurnPhase => COLOR_NOMINAL_IN_FLIGHT,
            HyacinthNominalState::CoastPhase => COLOR_NOMINAL_IN_FLIGHT,
            HyacinthNominalState::DroguePhase => COLOR_NOMINAL_IN_FLIGHT,
            HyacinthNominalState::MainPhase => COLOR_NOMINAL_IN_FLIGHT,
            HyacinthNominalState::LandedActive => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::Passivation => COLOR_NOMINAL_ON_GROUND,
            HyacinthNominalState::LandedPassivated => COLOR_NOMINAL_ON_GROUND,
        }
    }

    pub fn as_timeline(frontend: &Frontend) -> impl TimelineStateSequence + 'static {
        return (
            Self::Verification.as_timeline_state(frontend),
            (
                Self::IdlePassivated.as_timeline_state(frontend),
                (
                    Self::N2Filling.as_timeline_state(frontend),
                    (
                        Self::IdleActive.as_timeline_state(frontend),
                        (
                            Self::HarwareArmed.as_timeline_state(frontend),
                            (
                                Self::N2OFilling.as_timeline_state(frontend),
                                (
                                    Self::SoftwareArmed.as_timeline_state(frontend),
                                    (
                                        Self::Ignition.as_timeline_state(frontend),
                                        (
                                            Self::BurnPhase.as_timeline_state(frontend),
                                            (
                                                Self::CoastPhase.as_timeline_state(frontend),
                                                (
                                                    Self::DroguePhase.as_timeline_state(frontend),
                                                    (
                                                        Self::MainPhase.as_timeline_state(frontend),
                                                        (
                                                            Self::LandedActive.as_timeline_state(frontend),
                                                            (
                                                                Self::Passivation.as_timeline_state(frontend),
                                                                (
                                                                    Self::LandedPassivated.as_timeline_state(frontend),
                                                                    (),
                                                                ),
                                                            ),
                                                        ),
                                                    ),
                                                ),
                                            ),
                                        ),
                                    ),
                                ),
                            ),
                        ),
                    ),
                ),
            ),
        );
    }

    fn as_timeline_state(self, frontend: &Frontend) -> impl TimelineState + 'static {
        return TimelineStateData::new(
            <Self as Into<&'static str>>::into(self),
            self.associated_color(),
            self <= frontend.hyacinth_nominal_state(),
            (
                PopupContentData::<ModalDialog, _>::new(move |ui, captured_frontend, backend| {
                    let modal_dialog_size = ui.ctx().screen_rect().size() * Vec2::new(0.3, 0.15);
                    let mut button_clicked = false;
                    let response = ui
                        .allocate_ui_with_layout(modal_dialog_size, Layout::top_down(egui::Align::Center), |ui| {
                            ui.heading("Confirm State Transition");
                            ui.separator();
                            ui.allocate_space(Vec2::new(0f32, 10f32));
                            ui.label(format!(
                                "Are you sure you want to enter the state {}? ",
                                <Self as Into<&'static str>>::into(self)
                            ));
                            ui.horizontal(|ui| {
                                ui.allocate_space(modal_dialog_size / 3f32);
                                if ui.button("Proceed").clicked() {
                                    if backend
                                        .current_value::<crate::backend::storage::static_metrics::FlightMode>()
                                        .map(|fm| fm != self.associated_flight_mode())
                                        .unwrap_or(true)
                                    {
                                        backend
                                            .send_command(Command::SetFlightMode(self.associated_flight_mode()))
                                            .unwrap();
                                    }
                                    captured_frontend.set_hyacinth_anomalous_state(None);
                                    captured_frontend.set_hyacinth_nominal_state(self);
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
                    return egui::InnerResponse::new(button_clicked, response);
                }),
                (),
            ),
        );
    }
}

#[derive(Clone, Copy, PartialEq, IntoStaticStr)]
pub enum HyacinthAnomalousState {
    Hold,
    Abort,
}

impl HyacinthAnomalousState {
    fn associated_color(&self) -> Color32 {
        match self {
            HyacinthAnomalousState::Hold => COLOR_HOLD,
            HyacinthAnomalousState::Abort => COLOR_ABORT,
        }
    }

    pub fn as_timeline(frontend: &Frontend) -> impl TimelineStateSequence + 'static {
        return (Self::Hold.as_timeline_state(frontend), (Self::Abort.as_timeline_state(frontend), ()));
    }

    fn as_timeline_state(self, frontend: &Frontend) -> impl TimelineState + 'static {
        return TimelineStateData::new(
            <Self as Into<&'static str>>::into(self),
            self.associated_color(),
            frontend.hyacinth_anomolous_state().map(|a| a == self).unwrap_or(false),
            (
                PopupContentData::<ModalDialog, _>::new(move |ui, frontend, _backend| {
                    let modal_dialog_size = ui.ctx().screen_rect().size() * Vec2::new(0.3, 0.15);
                    let mut button_clicked = false;
                    let response = ui
                        .allocate_ui_with_layout(modal_dialog_size, Layout::top_down(egui::Align::Center), |ui| {
                            ui.heading("Confirm State Transition");
                            ui.separator();
                            ui.allocate_space(Vec2::new(0f32, 10f32));
                            ui.label(format!(
                                "Are you sure you want to enter the state {}? ",
                                <Self as Into<&'static str>>::into(self)
                            ));
                            ui.horizontal(|ui| {
                                ui.allocate_space(modal_dialog_size / 3f32);
                                if ui.button("Proceed").clicked() {
                                    frontend.set_hyacinth_anomalous_state(Some(self));
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
                    return egui::InnerResponse::new(button_clicked, response);
                }),
                (),
            ),
        );
    }
}

pub static COLOR_NOMINAL_ON_GROUND: Color32 = Color32::from_rgb(0x45, 0x85, 0x88);
pub static COLOR_NOMINAL_IN_FLIGHT: Color32 = Color32::from_rgb(0xfe, 0x80, 0x19);
pub static COLOR_NOMINAL_IGNITION: Color32 = Color32::from_rgb(0xb1, 0x62, 0x86);
pub static COLOR_HOLD: Color32 = Color32::from_rgb(0xd5, 0xc4, 0xa1);
pub static COLOR_ABORT: Color32 = Color32::from_rgb(0xcc, 0x24, 0x1d);

static RELATIVE_PADDING_HORIZONAL: f32 = 0.03;
static ABSOLUTE_CIRCLE_RADIUS: f32 = 16.0;
static ABSOLUTE_LINE_WIDTH_REACHED: f32 = 5.0;
static ABSOLUTE_LINE_WIDTH_PENDING: f32 = 2.0;
static ABSOLUTE_LINE_TO_STATE_GAP: f32 = 2.0;

pub trait AsDynVec {
    fn as_dyn_vec<'a>(&'a self) -> Vec<&'a dyn TimelineState>;
}

impl AsDynVec for () {
    fn as_dyn_vec<'a>(&'a self) -> Vec<&'a dyn TimelineState> {
        Vec::new()
    }
}

impl<H, T> AsDynVec for (H, T)
where
    H: TimelineState,
    T: AsDynVec + TimelineStateSequence,
{
    fn as_dyn_vec<'a>(&'a self) -> Vec<&'a dyn TimelineState> {
        let (ref head, ref tail) = *self;
        let mut v = Vec::with_capacity(1);
        v.push(head as &dyn TimelineState);
        v.extend(tail.as_dyn_vec());
        v
    }
}

pub trait TimelineState {
    fn active(&self) -> bool;
    fn name(&self) -> &'static str;
    fn stroke(&self) -> egui::Stroke;
    fn outer_diameter(&self) -> f32;
    fn fill(&self, bg_color: Color32) -> Color32;
    fn create_popups(
        &self,
        ui: &mut egui::Ui,
        backend: &mut Backend,
        frontend: &mut Frontend,
        trigger: TriggerBuilder<()>,
        position: Pos2,
    );
}
impl<L: PopupContentList> TimelineState for TimelineStateData<L> {
    fn active(&self) -> bool {
        return self.is_active;
    }

    fn name(&self) -> &'static str {
        return self.label;
    }

    fn stroke(&self) -> egui::Stroke {
        return self.stroke();
    }

    fn outer_diameter(&self) -> f32 {
        return self.outer_diameter();
    }

    fn fill(&self, bg_color: Color32) -> Color32 {
        return self.fill(bg_color);
    }

    fn create_popups(
        &self,
        ui: &mut egui::Ui,
        backend: &mut Backend,
        frontend: &mut Frontend,
        trigger: TriggerBuilder<()>,
        position: Pos2,
    ) {
        //TODO Hans: We should avoid cloning here, but cannot move out of self because trait must be dyn compatible for iteration
        trigger.add_all(self.popups.clone(), position).show_active(ui, frontend, backend);
        //TriggerBuilder::new(egui::Id::new(format!("TimelineState {}", self.label)), bounding_box)
    }
}

pub trait TimelineStateSequence: AsDynVec + Sized {
    fn add<S: TimelineState>(self, state: S) -> (Self, S) {
        return (self, state);
    }
    fn num_states() -> usize;
    fn max_active(&self) -> usize;
    fn dyn_iter<'a>(&'a self) -> std::vec::IntoIter<&'a dyn TimelineState> {
        self.as_dyn_vec().into_iter()
    }
}
impl TimelineStateSequence for () {
    fn max_active(&self) -> usize {
        return 0;
    }

    fn num_states() -> usize {
        return 0;
    }
}

impl<H, T> TimelineStateSequence for (H, T)
where
    H: TimelineState,
    T: TimelineStateSequence,
{
    fn max_active(&self) -> usize {
        let active_in_tail = self.1.max_active();
        if active_in_tail > 0 {
            return active_in_tail + 1;
        } else if self.0.active() {
            return 1;
        } else {
            return 0;
        }
    }

    fn num_states() -> usize {
        return T::num_states() + 1;
    }
}

pub struct TimelineStateData<PopupContents: PopupContentList> {
    label: &'static str,
    color: Color32,
    is_active: bool,
    popups: PopupContents,
}

impl<PopupContents: PopupContentList> TimelineStateData<PopupContents> {
    pub fn new(label: &'static str, color: Color32, is_active: bool, popups: PopupContents) -> Self {
        Self {
            label,
            color,
            is_active,
            popups,
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
    fn outer_diameter(&self) -> f32 {
        return 2f32 * (ABSOLUTE_CIRCLE_RADIUS + self.stroke().width);
    }
}

pub struct Timeline<'a, Milestones: TimelineStateSequence, AnomalousStates: TimelineStateSequence> {
    milestones: Milestones,
    anomalous_states: AnomalousStates,
    rotation: Rotation2<f32>,
    frontend: &'a mut Frontend,
    backend: &'a mut Backend,
}

impl<'a, Milestones: TimelineStateSequence, AnomalousStates: TimelineStateSequence>
    Timeline<'a, Milestones, AnomalousStates>
{
    pub fn new(
        milestones: Milestones,
        anomalous_states: AnomalousStates,
        rotation: Rotation2<f32>,
        frontend: &'a mut Frontend,
        backend: &'a mut Backend,
    ) -> Self {
        Self {
            milestones,
            anomalous_states,
            rotation,
            frontend,
            backend,
        }
    }

    pub fn paint(&mut self, transform: &Affine2<f32>, ui: &mut egui::Ui) {
        let vertical_offset = if AnomalousStates::num_states() == 0 { 0f32 } else { 0.35 };
        let line_offset = (ABSOLUTE_LINE_TO_STATE_GAP + ABSOLUTE_CIRCLE_RADIUS + ABSOLUTE_LINE_WIDTH_REACHED)
            * to_vec(get_basis_x(transform)).normalized();
        let distance_between_milestones = if Milestones::num_states() > 0 {
            (1f32 - 2f32 * RELATIVE_PADDING_HORIZONAL) / ((Milestones::num_states() - 1) as f32)
        } else {
            0.0
        };
        let bg_color = ThemeColors::new(ui.ctx()).background_weak;
        let fg_color = ThemeColors::new(ui.ctx()).foreground_weak;

        let milestone_positions = std::iter::successors(
            Some(Point2::new(-0.5f32 + RELATIVE_PADDING_HORIZONAL, vertical_offset * 0.3f32)),
            |p| Some(Point2::new(p.x + distance_between_milestones, p.y)),
        )
        .take(Milestones::num_states())
        .map(|p| to_pos(transform * p))
        .collect::<Vec<_>>();

        let lines = milestone_positions
            .windows(2)
            .map(|ps| Some((ps[0] + line_offset, ps[1] - line_offset)))
            .chain(std::iter::once(None))
            .collect::<Vec<_>>();

        for (i, milestone) in self.milestones.dyn_iter().enumerate() {
            let modal_dialog_size = ui.ctx().screen_rect().size() * Vec2::new(0.3, 0.15);
            let position = ui.ctx().screen_rect().center() - modal_dialog_size / 2f32;
            let trigger = TriggerBuilder::new(
                egui::Id::new(format!("FlightState {}", milestone.name())),
                Rect::from_center_size(
                    milestone_positions[i],
                    Vec2::new(milestone.outer_diameter(), milestone.outer_diameter()),
                ),
            );
            milestone.create_popups(ui, self.backend, self.frontend, trigger, position);
            let painter = ui.painter();
            lines[i].map(|(start, end)| {
                painter.line(
                    vec![start, end],
                    if i < self.milestones.max_active() - 1 {
                        Stroke::new(ABSOLUTE_LINE_WIDTH_REACHED, fg_color)
                    } else {
                        Stroke::new(ABSOLUTE_LINE_WIDTH_PENDING, fg_color)
                    },
                )
            });
            painter.add(egui::Shape::Circle(CircleShape {
                center: milestone_positions[i],
                radius: ABSOLUTE_CIRCLE_RADIUS,
                fill: milestone.fill(bg_color),
                stroke: milestone.stroke(),
            }));

            let label_layout = painter.layout_no_wrap(milestone.name().to_string(), FontId::default(), fg_color);
            painter.text(
                milestone_positions[i] + Vec2::new(0f32, label_layout.size().y + 5f32),
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
            .take(AnomalousStates::num_states())
            .map(|p| to_pos(transform * p))
            .collect::<Vec<_>>();

        for (anomalous_state, p) in std::iter::zip(self.anomalous_states.dyn_iter(), anomalous_state_positions) {
            let trigger = TriggerBuilder::new(
                egui::Id::new(format!("FlightState {}", anomalous_state.name())),
                Rect::from_center_size(
                    p,
                    Vec2::new(anomalous_state.outer_diameter(), anomalous_state.outer_diameter()),
                ),
            );
            let modal_dialog_size = ui.ctx().screen_rect().size() * Vec2::new(0.3, 0.15);
            let position = ui.ctx().screen_rect().center() - modal_dialog_size / 2f32;
            anomalous_state.create_popups(ui, self.backend, self.frontend, trigger, position);

            let painter = ui.painter();
            painter.add(egui::Shape::Circle(CircleShape {
                center: p,
                radius: ABSOLUTE_CIRCLE_RADIUS,
                fill: anomalous_state.fill(bg_color),
                stroke: anomalous_state.stroke(),
            }));

            let label_layout = painter.layout_no_wrap(anomalous_state.name().to_string(), FontId::default(), fg_color);

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

impl<'a, Milestones: TimelineStateSequence, AnomalousStates: TimelineStateSequence> egui::Widget
    for Timeline<'a, Milestones, AnomalousStates>
{
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
