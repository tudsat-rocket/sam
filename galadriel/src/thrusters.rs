use shared_types::telemetry::ThrusterValveState;

#[derive(Clone, Debug, PartialEq)]
pub struct ThrusterSettings {
    /// Propellant mass (not included in rocket dry mass) [kg]
    pub propellant_mass: f32,
    // add more parameters here, add them to GUI below if needed
}

impl Default for ThrusterSettings {
    fn default() -> Self {
        Self {
            propellant_mass: 1.0,
        }
    }
}

pub struct Thrusters {
    valve_state: ThrusterValveState,
    last_valve_state: ThrusterValveState,
    pub propellant_mass: f32,
}

impl Thrusters {
    pub fn new(settings: &ThrusterSettings) -> Self {
        Self {
            valve_state: ThrusterValveState::Closed,
            last_valve_state: ThrusterValveState::Closed,
            propellant_mass: settings.propellant_mass,
        }
    }

    pub fn tick(&mut self, delta_time: u32) {
        let lost = self.flow_rate() * (delta_time as f32) / 1000.0;
        self.propellant_mass -= lost / 1000.0;
    }

    pub fn set_valve(&mut self, valve_state: ThrusterValveState) {
        if valve_state != self.valve_state {
            self.last_valve_state = self.valve_state;
            self.valve_state = valve_state;
        }
    }

    pub fn tank_pressure(&self) -> f32 {
        todo!()
    }

    /// Current flow rate [g/s]
    pub fn flow_rate(&self) -> f32 {
        // TODO: delays, curve for opening/closing
        match self.valve_state {
            ThrusterValveState::Closed => 0.0,
            _ => 15.0,
        }
    }

    /// Thrust [N]
    pub fn thrust(&self) -> f32 {
        let direction: f32 = self.valve_state.into();
        direction * 10.0
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut ThrusterSettings {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            ui.label("💨 Thrusters");

            egui::Grid::new("galadriel_thrusters")
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Propellant mass");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.propellant_mass)
                                .suffix(" kg")
                                .speed(0.001)
                                .clamp_range(0.0..=200.0),
                        );
                    });
                    ui.end_row();
                });
        }).response
    }
}
