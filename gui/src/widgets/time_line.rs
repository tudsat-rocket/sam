use std::iter::once;

use chrono::offset;
use egui::{
    Align2, Color32, Context, FontFamily, FontId, Key, Modifiers, Rect, TextFormat, Vec2, epaint::RectShape,
    text::LayoutJob,
};
use itertools::Itertools;
use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2, Vector2};
use shared_types::{Command, FlightMode, ProcedureStep};
use strum::IntoStaticStr;

use crate::{
    backend::Backend,
    frontend::{
        Frontend,
        popup_manager::{ModalDialog, PopupContentData, PopupContentList, PopupPosition, TriggerBuilder},
    },
    system_diagram_components::{
        core::constants::{self, CORNER_RADIUS},
        math::{conversions::to_pos, transform::Transform, utils::get_basis_y},
    },
    utils::theme::ThemeColors,
    widgets::system_diagram::FlattenResponse,
};

trait ShortCutAsString {
    fn as_string(&self) -> String;
}

impl ShortCutAsString for (Modifiers, Key) {
    fn as_string(&self) -> String {
        let modifiers = self.0;
        let mut key_strings = vec![];
        if modifiers.alt {
            key_strings.push("ALT");
        }
        if modifiers.ctrl {
            key_strings.push("CTRL");
        }
        if modifiers.shift {
            key_strings.push("SHIFT");
        }
        key_strings.push(self.1.symbol_or_name());
        return std::iter::once("[")
            .chain(itertools::intersperse(key_strings, " + "))
            .chain(std::iter::once("]"))
            .map(|s| s.to_string())
            .reduce(|acc, e| acc + &e)
            .unwrap_or("".to_string());
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum HyacinthNominalState {
    Verification,
    IdlePassivated,
    N2Filling,
    IdleActive,
    HardwareArmed,
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
    fn as_string(&self) -> &'static str {
        match self {
            HyacinthNominalState::Verification => "Verification",
            HyacinthNominalState::IdlePassivated => "Idle Passivated",
            HyacinthNominalState::N2Filling => "N₂ Filling",
            HyacinthNominalState::IdleActive => "Idle Active",
            HyacinthNominalState::HardwareArmed => "Hardware Armed",
            HyacinthNominalState::N2OFilling => "N₂O Filling",
            HyacinthNominalState::SoftwareArmed => "Software Armed",
            HyacinthNominalState::Ignition => "Ignition",
            HyacinthNominalState::BurnPhase => "Burn Phase",
            HyacinthNominalState::CoastPhase => "Coast Phase",
            HyacinthNominalState::DroguePhase => "Drogue Phase",
            HyacinthNominalState::MainPhase => "Main Phase",
            HyacinthNominalState::LandedActive => "Landed Active",
            HyacinthNominalState::Passivation => "Passivation",
            HyacinthNominalState::LandedPassivated => "Landed Passivated",
        }
    }

    fn associated_procedure_step(&self) -> ProcedureStep {
        use HyacinthNominalState as S;
        use ProcedureStep as P;
        match self {
            S::Verification => P::Verification,
            S::IdlePassivated => P::IdlePassivated,
            S::N2Filling => P::N2Filling,
            S::IdleActive => P::IdleActive,
            S::HardwareArmed => P::HardwareArmed,
            S::N2OFilling => P::N2OFilling,
            S::SoftwareArmed => P::SoftwareArmed,
            S::Ignition => P::Ignition,
            S::BurnPhase => P::BurnPhase,
            S::CoastPhase => P::CoastPhase,
            S::DroguePhase => P::DroguePhase,
            S::MainPhase => P::MainPhase,
            S::LandedActive => P::LandedActive,
            S::Passivation => P::Passivation,
            S::LandedPassivated => P::LandedPassivated,
        }
    }

    fn associated_flight_mode(&self) -> FlightMode {
        match self {
            HyacinthNominalState::Verification => FlightMode::Idle,
            HyacinthNominalState::IdlePassivated => FlightMode::Idle,
            HyacinthNominalState::N2Filling => FlightMode::Idle,
            HyacinthNominalState::IdleActive => FlightMode::Idle,
            HyacinthNominalState::HardwareArmed => FlightMode::HardwareArmed,
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
            HyacinthNominalState::HardwareArmed => COLOR_NOMINAL_ON_GROUND,
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

    fn hotkey(&self) -> Option<(Modifiers, Key)> {
        match self {
            HyacinthNominalState::Verification => None,
            HyacinthNominalState::IdlePassivated => None,
            HyacinthNominalState::N2Filling => None,
            HyacinthNominalState::IdleActive => None,
            HyacinthNominalState::HardwareArmed => Some((Modifiers::SHIFT, Key::F5)),
            HyacinthNominalState::N2OFilling => Some((Modifiers::SHIFT, Key::F6)),
            HyacinthNominalState::SoftwareArmed => Some((Modifiers::SHIFT, Key::F7)),
            HyacinthNominalState::Ignition => Some((Modifiers::SHIFT, Key::F8)),
            HyacinthNominalState::BurnPhase => Some((Modifiers::SHIFT, Key::F9)),
            HyacinthNominalState::CoastPhase => Some((Modifiers::SHIFT, Key::F10)),
            HyacinthNominalState::DroguePhase => Some((Modifiers::SHIFT, Key::F11)),
            HyacinthNominalState::MainPhase => Some((Modifiers::SHIFT, Key::F12)),
            HyacinthNominalState::LandedActive => None,
            HyacinthNominalState::Passivation => None,
            HyacinthNominalState::LandedPassivated => None,
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
                            Self::HardwareArmed.as_timeline_state(frontend),
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
            self.as_string(),
            self.associated_color(),
            self <= frontend.hyacinth_nominal_state(),
            self.hotkey(),
            (
                PopupContentData::<ModalDialog, _>::new(move |ui, captured_frontend, backend| {
                    let mut button_clicked = false;
                    let theme = ThemeColors::new(ui.ctx());
                    let response = ui
                        .vertical_centered(|ui| {
                            ui.heading("Confirm State Transition");
                            ui.separator();
                            ui.allocate_space(Vec2::new(0f32, 10f32));
                            let mut text_layout = LayoutJob::default();
                            text_layout.append(
                                "Are you sure you want to enter the state \n",
                                0f32,
                                TextFormat {
                                    color: theme.foreground,
                                    ..Default::default()
                                },
                            );
                            text_layout.append(
                                self.as_string(),
                                0f32,
                                TextFormat {
                                    color: self.associated_color(),
                                    ..Default::default()
                                },
                            );
                            ui.label(text_layout);
                            ui.horizontal_wrapped(|ui| {
                                let mut proceed_layout = LayoutJob::default();
                                proceed_layout.append(
                                    "Proceed\n",
                                    0f32,
                                    TextFormat {
                                        color: theme.foreground,
                                        ..Default::default()
                                    },
                                );
                                proceed_layout.append("[⏎]", 14f32, Default::default());
                                let proceed_button =
                                    egui::Button::new(proceed_layout).min_size(Vec2::new(200f32, 50f32));
                                if ui.add(proceed_button).clicked()
                                    || ui.ctx().input_mut(|i| i.consume_key(Modifiers::NONE, Key::Enter))
                                {
                                    if let Err(e) =
                                        backend.send_command(Command::SetDisplayStep(self.associated_procedure_step()))
                                    {
                                        println!("Error sending command via backend: {}", e);
                                    }
                                    captured_frontend.set_hyacinth_anomalous_state(None);
                                    captured_frontend.set_hyacinth_nominal_state(self);
                                    button_clicked = true
                                };

                                let mut cancel_layout = LayoutJob::default();
                                cancel_layout.append(
                                    "Cancel\n",
                                    0f32,
                                    TextFormat {
                                        color: theme.foreground,
                                        ..Default::default()
                                    },
                                );
                                cancel_layout.append("[ESC]", 4f32, Default::default());
                                let cancel_button = egui::Button::new(cancel_layout).min_size(Vec2::new(200f32, 50f32));
                                if ui.add(cancel_button).clicked()
                                    || ui.ctx().input_mut(|i| i.consume_key(Modifiers::NONE, Key::Escape))
                                {
                                    button_clicked = true;
                                };
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

    fn hotkey(&self) -> Option<(Modifiers, Key)> {
        match self {
            HyacinthAnomalousState::Hold => None,
            HyacinthAnomalousState::Abort => None,
        }
    }

    fn as_timeline_state(self, frontend: &Frontend) -> impl TimelineState + 'static {
        return TimelineStateData::new(
            <Self as Into<&'static str>>::into(self),
            self.associated_color(),
            frontend.hyacinth_anomolous_state().map(|a| a == self).unwrap_or(false),
            self.hotkey(),
            (
                PopupContentData::<ModalDialog, _>::new(move |ui, frontend, _backend| {
                    let theme = ThemeColors::new(ui.ctx());
                    let mut button_clicked = false;
                    let response = ui
                        .vertical_centered(|ui| {
                            ui.heading("Confirm State Transition");
                            ui.separator();
                            ui.allocate_space(Vec2::new(0f32, 10f32));
                            let mut text_layout = LayoutJob::default();
                            text_layout.append(
                                "Are you sure you want to enter the state \n",
                                0f32,
                                TextFormat {
                                    color: theme.foreground,
                                    ..Default::default()
                                },
                            );
                            text_layout.append(
                                <Self as Into<&'static str>>::into(self),
                                0f32,
                                TextFormat {
                                    color: self.associated_color(),
                                    ..Default::default()
                                },
                            );
                            ui.label(text_layout);
                            ui.horizontal_wrapped(|ui| {
                                let mut proceed_layout = LayoutJob::default();
                                proceed_layout.append(
                                    "Proceed\n",
                                    0f32,
                                    TextFormat {
                                        color: theme.foreground,
                                        ..Default::default()
                                    },
                                );
                                proceed_layout.append("[⏎]", 14f32, Default::default());
                                let proceed_button =
                                    egui::Button::new(proceed_layout).min_size(Vec2::new(200f32, 50f32));
                                if ui.add(proceed_button).clicked()
                                    || ui.ctx().input_mut(|i| i.consume_key(Modifiers::NONE, Key::Enter))
                                {
                                    frontend.set_hyacinth_anomalous_state(Some(self));
                                    button_clicked = true
                                };

                                let mut cancel_layout = LayoutJob::default();
                                cancel_layout.append(
                                    "Cancel\n",
                                    0f32,
                                    TextFormat {
                                        color: theme.foreground,
                                        ..Default::default()
                                    },
                                );
                                cancel_layout.append("[ESC]", 4f32, Default::default());
                                let cancel_response =
                                    ui.add(egui::Button::new(cancel_layout).min_size(Vec2::new(200f32, 50f32)));
                                if cancel_response.clicked()
                                    || ui.ctx().input_mut(|i| i.consume_key(Modifiers::NONE, Key::Escape))
                                {
                                    button_clicked = true;
                                };
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

static RELATIVE_PADDING_HORIZONAL: f32 = 0.01;
static RELATIVE_HEIGHT_MILESTONE: f32 = 0.5;
static RELATIVE_HEIGHT_ANOMALOUS_STATE: f32 = 0.3;

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
    fn fill(&self, bg_color: Color32) -> Color32;
    fn hotkey(&self) -> Option<(Modifiers, Key)>;
    fn create_popups(
        &self,
        ui: &mut egui::Ui,
        backend: &mut Backend,
        frontend: &mut Frontend,
        trigger: TriggerBuilder<()>,
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

    fn fill(&self, bg_color: Color32) -> Color32 {
        return self.fill(bg_color);
    }

    fn create_popups(
        &self,
        ui: &mut egui::Ui,
        backend: &mut Backend,
        frontend: &mut Frontend,
        trigger: TriggerBuilder<()>,
    ) {
        //TODO Hans: We should avoid cloning here, but cannot move out of self because trait must be dyn compatible for iteration
        let mut filled_trigger = trigger.add_all(self.popups.clone(), PopupPosition::Centered);
        if let Some(hotkey) = self.hotkey {
            if ui.ctx().input_mut(|i| i.consume_key(hotkey.0, hotkey.1)) {
                filled_trigger.set_active();
            }
        }
        filled_trigger.show_active(ui, frontend, backend);
    }

    fn hotkey(&self) -> Option<(Modifiers, Key)> {
        return self.hotkey;
    }
}

/// Convert an sRGB component (0–255) to linearized value for luminance.
fn linearize(c: u8) -> f32 {
    let cs = c as f32 / 255.0;
    if cs <= 0.03928 {
        cs / 12.92
    } else {
        ((cs + 0.055) / 1.055).powf(2.4)
    }
}

/// Compute relative luminance for an egui Color32.
fn luminance(color: &Color32) -> f32 {
    let [r, g, b, _] = color.to_array();
    0.2126 * linearize(r) + 0.7152 * linearize(g) + 0.0722 * linearize(b)
}

/// Compute contrast ratio between two colors (always ≥ 1.0).
fn contrast_ratio(a: &Color32, b: &Color32) -> f32 {
    let l1 = luminance(a);
    let l2 = luminance(b);
    let (light, dark) = if l1 > l2 { (l1, l2) } else { (l2, l1) };
    (light + 0.05) / (dark + 0.05)
}

/// Return the color with better contrast against the background.
pub fn better_contrast_color(background: &Color32, color1: Color32, color2: Color32) -> Color32 {
    let contrast1 = contrast_ratio(background, &color1);
    let contrast2 = contrast_ratio(background, &color2);
    if contrast1 >= contrast2 { color1 } else { color2 }
}

enum ColorStrength {
    VeryWeak,
    Weak,
    Normal,
    Strong,
}

fn text_color_for_bg(bg_color: &Color32, ctx: &Context, strength: ColorStrength) -> Color32 {
    let theme = ThemeColors::new(ctx);
    let color_fg = match strength {
        ColorStrength::VeryWeak => theme.foreground_very_weak,
        ColorStrength::Weak => theme.foreground_weak,
        ColorStrength::Normal => theme.foreground,
        ColorStrength::Strong => theme.foreground_strong,
    };
    let color_bg = match strength {
        ColorStrength::VeryWeak => theme.background_weak,
        ColorStrength::Weak => theme.background,
        ColorStrength::Normal => theme.background_strong,
        ColorStrength::Strong => theme.background_very_strong,
    };
    return better_contrast_color(bg_color, color_bg, color_fg);
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

    fn paint_timeline(
        &self,
        transform: &Affine2<f32>,
        ui: &mut egui::Ui,
        frontend: &mut Frontend,
        backend: &mut Backend,
    ) {
        let relative_width_anomalous_state = if Self::num_states() > 0 {
            (1f32 - (Self::num_states() + 1) as f32 * RELATIVE_PADDING_HORIZONAL) / (Self::num_states() as f32)
        } else {
            0.0
        };

        let anomalous_state_positions =
            std::iter::successors(Some(Point2::new(-0.5 + RELATIVE_PADDING_HORIZONAL, -0.5)), |p| {
                Some(Point2::new(p.x + relative_width_anomalous_state + RELATIVE_PADDING_HORIZONAL, p.y))
            })
            .take(Self::num_states())
            .map(|p| to_pos(transform * p))
            .collect::<Vec<_>>();

        //These only work when the timeline is horizontal
        let absolute_width_anomalous_state =
            transform.transform_vector(&Vector2::new(relative_width_anomalous_state, 0f32)).x;
        let absolute_height_anomalous_state = get_basis_y(transform).y;

        for (anomalous_state, p) in std::iter::zip(self.dyn_iter(), anomalous_state_positions) {
            let fill_color = anomalous_state.fill(ThemeColors::new(ui.ctx()).background_weak);
            ui.painter().add(egui::Shape::Rect(RectShape::new(
                Rect::from_min_size(p, Vec2::new(absolute_width_anomalous_state, absolute_height_anomalous_state)),
                CORNER_RADIUS,
                fill_color,
                anomalous_state.stroke(),
                egui::StrokeKind::Middle,
            )));

            let trigger = TriggerBuilder::new(
                egui::Id::new(format!("FlightState {}", anomalous_state.name())),
                Rect::from_min_size(p, Vec2::new(absolute_width_anomalous_state, absolute_height_anomalous_state)),
            );
            anomalous_state.create_popups(ui, backend, frontend, trigger);

            let string_height = 17f32;
            let button_center = p + Vec2::new(absolute_width_anomalous_state, absolute_height_anomalous_state) / 2f32;
            let state_name = anomalous_state.name().to_string();
            let num_strings = state_name.chars().filter(|c| c.is_whitespace()).count();
            //+ if anomalous_state.hotkey().is_some() { 1 } else { 0 };
            let mut offset = -(num_strings as f32 * string_height) / 2f32;

            for string in state_name.split_whitespace() {
                ui.painter().text(
                    button_center + Vec2::new(0f32, offset),
                    Align2::CENTER_CENTER,
                    string,
                    FontId::new(14f32, FontFamily::Proportional),
                    text_color_for_bg(&fill_color, ui.ctx(), ColorStrength::Weak),
                );
                offset += string_height;
            }

            if let Some(hotkey) = anomalous_state.hotkey() {
                ui.painter().text(
                    button_center + Vec2::new(0f32, offset),
                    Align2::CENTER_CENTER,
                    hotkey.as_string(),
                    FontId::new(10f32, FontFamily::Proportional),
                    text_color_for_bg(&fill_color, ui.ctx(), ColorStrength::VeryWeak),
                );
            }
        }
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
    hotkey: Option<(Modifiers, Key)>,
    popups: PopupContents,
}

impl<PopupContents: PopupContentList> TimelineStateData<PopupContents> {
    pub fn new(
        label: &'static str,
        color: Color32,
        is_active: bool,
        hotkey: Option<(Modifiers, Key)>,
        popups: PopupContents,
    ) -> Self {
        Self {
            label,
            color,
            is_active,
            hotkey,
            popups,
        }
    }

    fn fill(&self, bg_color: Color32) -> Color32 {
        return if self.is_active { self.color } else { bg_color };
    }

    fn stroke(&self) -> egui::Stroke {
        return egui::Stroke {
            width: constants::STROKE_WIDTH,
            color: self.color,
        };
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
        let mut height_timelines = 0f32;
        let mut num_timelines = 0;
        if Milestones::num_states() > 0 {
            height_timelines += RELATIVE_HEIGHT_MILESTONE;
            num_timelines += 1;
        }
        if AnomalousStates::num_states() > 0 {
            height_timelines += RELATIVE_HEIGHT_ANOMALOUS_STATE;
            num_timelines += 1;
        }
        let relative_padding_vertical = (1f32 - height_timelines) / (num_timelines + 1) as f32;
        let milestone_transform = transform
            * Transform::new(
                Rotation2::identity(),
                Scale2::new(1f32, RELATIVE_HEIGHT_MILESTONE),
                Translation2::new(0f32, 0.5 - relative_padding_vertical - RELATIVE_HEIGHT_MILESTONE / 2f32),
            )
            .to_affine2();
        self.milestones.paint_timeline(&milestone_transform, ui, self.frontend, self.backend);
        let anomalous_state_transform = transform
            * Transform::new(
                Rotation2::identity(),
                Scale2::new(1f32, RELATIVE_HEIGHT_ANOMALOUS_STATE),
                Translation2::new(0f32, -0.5 + relative_padding_vertical + RELATIVE_HEIGHT_ANOMALOUS_STATE / 2f32),
            )
            .to_affine2();
        self.anomalous_states.paint_timeline(&anomalous_state_transform, ui, self.frontend, self.backend);
    }
}

impl<'a, Milestones: TimelineStateSequence, AnomalousStates: TimelineStateSequence> egui::Widget
    for Timeline<'a, Milestones, AnomalousStates>
{
    fn ui(mut self, ui: &mut egui::Ui) -> egui::Response {
        let (available_space, response) = ui.allocate_exact_size(
            Vec2::new(ui.available_rect_before_wrap().width(), ui.available_rect_before_wrap().width() * 0.1),
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
