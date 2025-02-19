use crate::drag::*;

#[derive(Default, Clone, Debug, PartialEq)]
pub enum ParachuteTrigger {
    /// Deploy at apogee
    #[default]
    AtApogee,
    /// Deploy below a certain altitude in m AGL
    BelowAltitude(f32),
    // TODO: SITL
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParachuteSettings {
    pub drag_coef: DragCoefficient,
    pub area: f32,
    pub trigger: ParachuteTrigger,
    pub delay: f32,
}

impl Default for ParachuteSettings {
    fn default() -> Self {
        Self {
            drag_coef: DragCoefficient::Constant(1.5),
            area: 0.15,
            trigger: ParachuteTrigger::AtApogee,
            delay: 1.0,
        }
    }
}

pub struct Parachute {
    pub settings: ParachuteSettings,
    pub open: bool,
}

impl Parachute {
    pub fn new(settings: &ParachuteSettings) -> Self {
        Self {
            settings: settings.clone(),
            open: false,
        }
    }

    pub fn drag(&self, velocity: f32, density: f32) -> f32 {
        if self.open {
            self.settings.drag_coef.pressure(velocity, density) *
                self.settings.area
        } else {
            0.0
        }
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut ParachuteSettings {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            if self.trigger == ParachuteTrigger::AtApogee {
                ui.label("☂ Drogue");
            } else {
                ui.label("☂ Main");
            }

            // TODO: ids
            egui::Grid::new(format!("parachute_{:?}", self.trigger))
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Cross section");
                    ui.add(
                        egui::DragValue::new(&mut self.area)
                            .suffix(" m²")
                            .speed(0.001)
                            .range(0.001..=100.0),
                    );
                    ui.end_row();
                    ui.label("Drag coef.");
                    ui.add(&mut self.drag_coef);
                    ui.end_row();
                });
        }).response
    }
}
