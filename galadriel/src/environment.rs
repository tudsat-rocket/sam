#[derive(Clone, Debug, PartialEq)]
pub enum AtmosphericModel {
    /// Constant air density (kg/m^3)
    Constant(f32),
    // TODO: draw the rest of the atmospheric model
}

impl Default for AtmosphericModel {
    fn default() -> Self {
        Self::Constant(1.2) // TODO
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Environment {
    pub atmospheric_model: AtmosphericModel,
    pub launch_location: (f32, f32),
    pub launch_altitude: f32,
    pub launch_rail_length: f32,
    pub azimuth: f32,
    pub elevation: f32,
}

impl Default for Environment {
    fn default() -> Self {
        Self {
            atmospheric_model: AtmosphericModel::default(),
            launch_location: (39.390104, -8.289324),
            launch_altitude: 100.0,
            launch_rail_length: 6.0,
            azimuth: 135.0, // south east
            elevation: 82.0,
        }
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut Environment {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            ui.label("⛅ Environment");

            egui::Grid::new("galadriel_environment")
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Launch coords.");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.launch_location.0)
                                .speed(0.000001)
                                .clamp_range(-90.0..=90.0)
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.launch_location.1)
                                .speed(0.000001)
                                .clamp_range(-180.0..=180.0)
                        );
                    });
                    ui.end_row();

                    ui.label("Launch altitude");
                    ui.add(
                        egui::DragValue::new(&mut self.launch_altitude)
                            .suffix(" m")
                            .speed(0.1)
                            .clamp_range(-100.0..=6000.0)
                    );
                    ui.end_row();

                    ui.label("Launch rail length");
                    ui.add(
                        egui::DragValue::new(&mut self.launch_rail_length)
                            .suffix(" m")
                            .speed(0.1)
                            .clamp_range(1.0..=100.0)
                    );
                    ui.end_row();

                    ui.label("Launch azimuth");
                    ui.add(
                        egui::DragValue::new(&mut self.azimuth)
                            .suffix(" °")
                            .speed(0.1)
                            .clamp_range(0.0..=360.0)
                    );
                    ui.end_row();

                    ui.label("Launch elevation");
                    ui.add(
                        egui::DragValue::new(&mut self.elevation)
                            .suffix(" °")
                            .speed(0.1)
                            .clamp_range(0.0..=90.0)
                    );
                    ui.end_row();
                });
        }).response
    }
}
