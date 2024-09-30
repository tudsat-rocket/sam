use shared_types::telemetry::ThrusterValveState;

#[derive(Clone, Debug, PartialEq)]
pub struct ThrusterSettings {
    /// Propellant mass (not included in rocket dry mass) [kg]
    pub propellant_mass: f32,
    /// Tank volume [L]
    pub tank_volume: f32,
    /// Thrust at nominal nozzle pressure [N]
    pub nominal_thrust: f32,
    /// Nominal nozzle_pressure [bar]
    pub nominal_nozzle_pressure: f32,
    /// Influence of nozzle pressure no thrust [N / bar]
    pub thrust_pressure_slope: f32,
}

impl Default for ThrusterSettings {
    fn default() -> Self {
        Self {
            propellant_mass: 1.0,
            tank_volume: 1.5,
            nominal_thrust: 10.0,
            nominal_nozzle_pressure: 30.0,
            thrust_pressure_slope: 0.0, // TODO
        }
    }
}

pub struct Thrusters {
    valve_state: ThrusterValveState,
    last_valve_state: ThrusterValveState,
    pub propellant_mass: f32,
    settings: ThrusterSettings,
}

impl Thrusters {
    pub fn new(settings: &ThrusterSettings) -> Self {
        Self {
            valve_state: ThrusterValveState::Closed,
            last_valve_state: ThrusterValveState::Closed,
            propellant_mass: settings.propellant_mass,
            settings: settings.clone(),
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
        direction * self.settings.nominal_thrust
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
                    ui.label("Propellant");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.propellant_mass)
                                .suffix(" kg")
                                .speed(0.001)
                                .clamp_range(0.0..=200.0),
                        );
                        ui.weak(" in ");
                        ui.add(
                            egui::DragValue::new(&mut self.tank_volume)
                                .suffix(" L")
                                .speed(0.01)
                                .clamp_range(0.0..=200.0),
                        );
                    });
                    ui.end_row();

                    ui.label("Nozzle pressure");
                    ui.add(
                        egui::DragValue::new(&mut self.nominal_nozzle_pressure)
                            .suffix(" bar")
                            .speed(0.1)
                            .clamp_range(0.0..=1000.0),
                    );
                    ui.end_row();

                    ui.label("Thrust");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.nominal_thrust)
                                .suffix(" N")
                                .speed(0.01)
                                .clamp_range(0.0..=1000.0),
                        );
                        ui.weak(format!(" @ {}bar, ", self.nominal_nozzle_pressure));
                        ui.add(
                            egui::DragValue::new(&mut self.thrust_pressure_slope)
                                .suffix(" N/bar")
                                .speed(0.01)
                                .clamp_range(0.0..=1000.0),
                        );
                    });
                    ui.end_row();
                });
        }).response
    }
}
