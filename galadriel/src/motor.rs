#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum KnownMotors {
    #[default]
    CesaroniPro988088M1790P
}

impl KnownMotors {
    pub fn all() -> impl Iterator<Item=KnownMotors> {
        vec![Self::CesaroniPro988088M1790P].into_iter()
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::CesaroniPro988088M1790P => "Cesaroni Pro98 8088M1790-P",
        }
    }
}

// TODO: load these from eng files at build time?
impl Into<MotorSettings> for KnownMotors {
    fn into(self) -> MotorSettings {
        match self {
            Self::CesaroniPro988088M1790P => MotorSettings {
                selected_motor: self,
                dry_mass: 8.273 - 4.706,
                wet_mass: 8.273,
                thrust_curve: vec![
                    (0.0, 0.0),
                    (0.059, 1791.514),
                    (0.199, 1596.375),
                    (0.6, 1782.109),
                    (1.215, 1913.769),
                    (1.973, 2021.918),
                    (2.742, 1970.195),
                    (3.387, 1833.833),
                    (3.812, 1652.801),
                    (4.28, 1556.407),
                    (4.385, 1295.438),
                    (4.476, 355.011),
                    (4.541, 91.692),
                    (4.597, 0.0),
                ],
                impulse: 8088.9,
                performance: 1.0, 
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MotorSettings {
    pub selected_motor: KnownMotors,
    pub dry_mass: f32,
    pub wet_mass: f32,
    pub thrust_curve: Vec<(f32, f32)>,
    pub impulse: f32,
    pub performance: f32,
}

impl Default for MotorSettings {
    fn default() -> Self {
        KnownMotors::default().into()
    }
}

#[derive(Default)]
pub struct Motor {
    settings: MotorSettings,
    pub last_mass: f32,
}

impl Motor {
    pub fn new(settings: &MotorSettings) -> Self {
        Self {
            settings: settings.clone(),
            ..Default::default()
        }
    }

    fn burn_time(&self) -> f32 {
        self.settings.thrust_curve.last().unwrap().0
    }

    fn propellant_mass(&self) -> f32 {
        self.settings.wet_mass - self.settings.dry_mass
    }

    pub fn thrust(&self, time_since_ignition: f32) -> f32 {
        if time_since_ignition > 0.0 && time_since_ignition < self.burn_time() {
            let i = self.settings.thrust_curve.iter()
                .position(|(x, _y)| *x > time_since_ignition)
                .unwrap_or_default();
            let a = self.settings.thrust_curve[i-1];
            let b = self.settings.thrust_curve[i];
            let f = (time_since_ignition - a.0) / (b.0 - a.0);
            self.settings.performance * (a.1 + f*(b.1 - a.1))
        } else {
            0.0
        }
    }

    pub fn burned_out(&self, time: f32) -> bool {
        time > self.burn_time()
    }

    pub fn mass(&mut self, time: f32) -> f32 {
        self.last_mass = if self.burned_out(time) {
            self.settings.dry_mass
        } else { 
            let i = self.settings.thrust_curve.iter()
                .position(|(x, _y)| *x > time)
                .unwrap_or_default();
            let mut burned_impulse = 0.0;
            if i > 0 {
                let mut c = 0;
                while c < i - 1 {
                    let a = self.settings.thrust_curve[c+1];
                    let b = self.settings.thrust_curve[c];
                    burned_impulse += (a.0 - b.0) * ((a.1 + b.1)/2.0);
                    c += 1;
                }
                let a = self.settings.thrust_curve[c+1];
                let b = self.settings.thrust_curve[c];
                // interpolate thrust value between data points 
                let n_interp = b.1 + ((time - b.0) / (a.0 - b.0)) * (a.1-b.1);
                burned_impulse += (time - b.0) * ((b.1 + n_interp)/2.0);
            }
            self.settings.dry_mass + self.propellant_mass() * (1.0 - (burned_impulse / self.settings.impulse))
        };
        self.last_mass
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut MotorSettings {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            ui.label("🔥 Motor");

            egui::Grid::new("galadriel_motor")
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("");
                    ui.horizontal(|ui| {
                        egui::ComboBox::from_id_source("galadriel_known_motor")
                            .selected_text(self.selected_motor.name())
                            .show_ui(ui, |ui| {
                                for m in KnownMotors::all() {
                                    ui.selectable_value(&mut self.selected_motor, m, m.name());
                                }
                            }
                        );
                    });
                    ui.end_row();

                    ui.label("Mass");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.wet_mass)
                                .suffix(" kg")
                                .speed(0.001)
                                .clamp_range(0.0..=200.0),
                        );
                        ui.weak("wet, ");
                        ui.add(
                            egui::DragValue::new(&mut self.dry_mass)
                                .suffix(" kg")
                                .speed(0.001)
                                .clamp_range(0.0..=200.0),
                        );
                        ui.weak("dry");
                    });
                    ui.end_row();

                    ui.label("Performance");
                    ui.add(
                        egui::DragValue::new(&mut self.performance)
                            .speed(0.001)
                            .clamp_range(0.0..=10.0),
                    );
                    ui.end_row();

                    ui.label("Impulse");
                    ui.add(
                        egui::DragValue::new(&mut self.impulse)
                            .suffix(" Ns")
                            .speed(0.1)
                            .clamp_range(0.0..=15000.0),
                    );
                    ui.end_row();

                    ui.label("Thrust curve");
                    egui_plot::Plot::new("galadriel_thrust_curve")
                        .set_margin_fraction(egui::Vec2::new(0.1, 0.3))
                        .allow_scroll(false)
                        .allow_drag(false)
                        .allow_zoom(false)
                        .allow_boxed_zoom(false)
                        .show_axes(false)
                        .show(ui, move |plot_ui| {
                            let points: Vec<_> = self.thrust_curve.iter()
                                .map(|(x, y)| [*x as f64, *y as f64])
                                .collect();
                            plot_ui.line(egui_plot::Line::new(points));
                        });
                });
        }).response
    }
}
