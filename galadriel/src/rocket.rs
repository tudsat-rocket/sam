use std::f32::consts::PI;

use nalgebra::Vector3;

use crate::drag::*;
use crate::motor::*;
use crate::parachute::*;
use crate::thrusters::*;

#[derive(Clone, Debug, PartialEq)]
pub struct RocketSettings {
    /// Mass without motor casing [kg]
    pub dry_mass: f32,
    drag_coef: DragCoefficient,
    area: f32,
    motor: MotorSettings,
    thrusters: Option<ThrusterSettings>,
    parachutes: Vec<ParachuteSettings>,
}

impl Default for RocketSettings {
    fn default() -> Self {
        let propellant_mass = 0.5;
        Self {
            dry_mass: 14.157 - propellant_mass,
            drag_coef: DragCoefficient::Variable,
            area: 0.018,
            motor: MotorSettings {
                performance: 0.95,
                ..Default::default()
            },
            thrusters: Some(ThrusterSettings {
                propellant_mass
            }),
            parachutes: vec![
                ParachuteSettings {
                    area: PI * (0.457/2.0f32).powi(2),
                    ..Default::default()
                },
                ParachuteSettings {
                    area: PI * (2.44/2.0f32).powi(2),
                    trigger: ParachuteTrigger::BelowAltitude(300.0),
                    ..Default::default()
                },
            ],
        }
    }
}

pub struct Rocket {
    settings: RocketSettings,
    pub motor: Motor,
    pub thrusters: Option<Thrusters>,
    pub parachutes: Vec<Parachute>,
    // TODO: this is slightly awkward, maybe store time and recompute when needed?
    pub last_mass: f32,
    pub last_thrust: Vector3<f32>,
    pub last_drag: Vector3<f32>,
}

impl Rocket {
    pub fn new(settings: &RocketSettings) -> Self {
        let motor = Motor::new(&settings.motor);
        let thrusters = settings.thrusters.as_ref().map(|s| Thrusters::new(s));
        let parachutes = settings.parachutes.iter()
            .map(|s| Parachute::new(s))
            .collect();
        Self {
            settings: settings.clone(),
            motor,
            thrusters,
            parachutes,
            last_mass: 0.0,
            last_thrust: Vector3::default(),
            last_drag: Vector3::default()
        }
    }

    pub fn mass(&mut self, time_since_ignition: f32) -> f32 {
        self.last_mass = self.settings.dry_mass +
            self.motor.mass(time_since_ignition) +
            self.thrusters.as_ref()
                .map(|t| t.propellant_mass)
                .unwrap_or_default();
        self.last_mass
    }

    pub fn thrust(&mut self, time_since_ignition: f32) -> Vector3<f32> {
        let t = self.motor.thrust(time_since_ignition) +
            self.thrusters.as_ref()
                .map(|t| t.thrust())
                .unwrap_or_default();
        self.last_thrust = Vector3::new(0.0, 0.0, t);
        self.last_thrust
    }

    pub fn drag(&mut self, velocity: Vector3<f32>, density: f32) -> Vector3<f32> {
        let velmag = velocity.magnitude();
        let vehicle = self.settings.drag_coef.pressure(velmag, density) * self.settings.area;
        let parachutes: f32 = self.parachutes.iter()
            .map(|p| p.drag(velmag, density))
            .sum();
        let mut direction = velocity.normalize() * -1.0;
        if direction.x.is_nan() {
            direction = Vector3::new(0.0, 0.0, 0.0);
        }
        self.last_drag = direction * (vehicle + parachutes);
        self.last_drag
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut RocketSettings {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            ui.label("🚀 Rocket");

            egui::Grid::new("galadriel_rocket")
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Dry mass");
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.dry_mass)
                                .suffix(" kg")
                                .speed(0.001)
                                .clamp_range(0.0..=200.0),
                        );
                        ui.weak("(without motor casing)");
                    });
                    ui.end_row();

                    ui.label("Cross section");
                    ui.add(
                        egui::DragValue::new(&mut self.area)
                            .suffix(" m²")
                            .speed(0.001)
                            .clamp_range(0.0..=1.0),
                    );
                    ui.end_row();

                    ui.label("Drag coef.");
                    ui.add(&mut self.drag_coef);
                    ui.end_row();
                });

            ui.add_space(10.0);
            ui.add(&mut self.motor);

            if let Some(thrusters) = self.thrusters.as_mut() {
                ui.add_space(10.0);
                ui.add(thrusters);
            }

            for p in &mut self.parachutes {
                ui.add_space(10.0);
                ui.add(p);
            }
        }).response
    }
}
