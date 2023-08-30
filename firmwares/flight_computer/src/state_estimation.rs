use nalgebra::*;
use num_traits::Pow;
use ahrs::Ahrs;
use filter::kalman::kalman_filter::KalmanFilter;

use crate::{telemetry::FlightMode::{*, self}, settings::{Settings, MainOutputMode}};


const GRAVITY: f32 = 9.80665;

#[derive(Debug)]
pub struct StateEstimator {
    time: u32,
    mode: FlightMode,
    mode_time: u32,
    condition_true_since: Option<u32>,
    settings: Settings,
    ahrs: ahrs::Mahony<f32>,
    kalman: KalmanFilter<f32, U3, U2, U0>,
    pub orientation: Option<Unit<Quaternion<f32>>>,
    acceleration: Option<Vector3<f32>>,
    acceleration_world: Option<Vector3<f32>>,
    pub altitude_ground: f32,
    pub altitude_max: f32,
}

impl StateEstimator {
    pub fn new(
        main_loop_freq_hertz: f32,
        settings: Settings,
    ) -> Self {
        let dt = 1.0 / main_loop_freq_hertz;
        let ahrs = ahrs::Mahony::new(dt, settings.mahony_kp, settings.mahony_ki);

        let mut kalman = KalmanFilter::default();
        kalman.x = Vector3::new(0.0, 0.0, 0.0);
        kalman.F = Matrix3::new(
            1.0, dt, dt * dt * 0.5,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0
        );
        kalman.H = Matrix2x3::new(
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        );
        kalman.P = Matrix3::new(
            settings.std_dev_barometer.pow(2), 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, settings.std_dev_accelerometer.pow(2),
        );
        kalman.Q = Matrix3::new(
            0.25f32 * dt.pow(4), 0.5f32 * dt.pow(3), 0.5f32 * dt.pow(2),
            0.5f32 * dt.pow(3), dt.pow(2), dt,
            0.5f32 * dt.pow(2), dt, 1.0f32,
        ) * settings.std_dev_process.pow(2);

        kalman.R *= Matrix2::new(
            settings.std_dev_barometer.pow(2), 0.0,
            0.0, settings.std_dev_accelerometer.pow(2)
        );

        Self {
            time: 0,
            mode: Idle,
            mode_time: 0,
            condition_true_since: None,
            settings,
            ahrs,
            kalman,
            orientation: None,
            acceleration: None,
            acceleration_world: None,
            altitude_ground: 0.0,
            altitude_max: -10_000.0,
        }
    }

    pub fn update(
        &mut self,
        time: u32,
        mode: FlightMode,
        gyroscope: Option<Vector3<f32>>,
        accelerometer1: Option<Vector3<f32>>,
        accelerometer2: Option<Vector3<f32>>,
        magnetometer: Option<Vector3<f32>>,
        barometer: Option<f32>
    ) {
        self.time = time;
        if mode != self.mode {
            self.mode = mode;
            self.mode_time = self.time;
            self.condition_true_since = None;
        }

        // Determine accelerometer to use. We prefer the primary because it is less noisy,
        // but have to switch to the secondary if we exceed +-16G (or get close enough) on any axis.
        self.acceleration = match (accelerometer1, accelerometer2) {
            (Some(acc1), Some(acc2)) if acc1.max() > 14.0 * GRAVITY && acc2.max() > 14.0 * GRAVITY => Some(acc2),
            (Some(acc1), _)                                                                        => Some(acc1),
            (None, Some(acc2))                                                                     => Some(acc2),
            (None, None)                                                                           => None
        };

        if let (Some(gyro), Some(acc), Some(mag)) = (&gyroscope, &self.acceleration, &magnetometer) {
            // Update the orientation estimator with IMU data
            self.orientation = self
                .ahrs
                .update(&(gyro * 3.14159 / 180.0), acc, mag)
                .ok()
                .map(|q| *q);

            // Rotate acceleration vector to get world-space acceleration
            // (where Z is straight up) and subtract gravity.
            self.acceleration_world = self.orientation
                .map(|quat| quat.transform_vector(acc) - Vector3::new(0.0, 0.0, GRAVITY));
        } else {
            self.orientation = None;
            self.acceleration_world = None;
        }

        // Update the Kalman filter with barometric altitude and world-space vertical acceleration
        let altitude_baro = barometer.and_then(|a| (!a.is_nan()).then(|| a));
        let accel_z = self.acceleration_world.map(|a| a.z)
            .and_then(|a| (!a.is_nan()).then(|| a));

        match (accel_z, altitude_baro) {
            (Some(accel_z), Some(altitude_baro)) => {
                let z = Vector2::new(altitude_baro, accel_z);
                self.kalman.update(&z, None, None);
                self.kalman.predict(None, None, None, None);
            },
            // TODO: handle error cases
            _ => {}
        }

        // Continuously reset ground altitude before arming.
        if mode < Armed {
            self.altitude_ground = self.altitude();
        }

        // Only track maximum height during flight
        self.altitude_max = match mode {
            Idle | HardwareArmed | Armed => self.altitude(),
            Flight | RecoveryDrogue | RecoveryMain => f32::max(self.altitude_max, self.altitude()),
            Landed => self.altitude_max,
        }
    }

    pub fn acceleration_world(&self) -> Option<&Vector3<f32>> {
        self.acceleration_world.as_ref()
    }

    pub fn altitude(&self) -> f32 {
        self.kalman.x.x
    }

    pub fn vertical_speed(&self) -> f32 {
        self.kalman.x.y
    }

    pub fn vertical_accel(&self) -> f32 {
        self.kalman.x.z
    }

    pub fn new_mode(&mut self, arm_voltage: u16) -> Option<FlightMode> {
        let armv = arm_voltage;
        let vacc = self.acceleration.map(|acc| acc.z).unwrap_or(0.0);
        let elapsed = self.time.checked_sub(self.mode_time).unwrap_or(0);
        let recovery_duration = self.settings.outputs_warning_time + self.settings.outputs_high_time;

        match self.mode {
            FlightMode::Idle => self.true_since(armv >= 100, 100).then(|| FlightMode::HardwareArmed),
            FlightMode::HardwareArmed => self.true_since(armv < 10, 100).then(|| FlightMode::Idle),
            FlightMode::Armed => self.true_since(vacc > self.settings.min_takeoff_acc, self.settings.min_takeoff_acc_time).then(|| FlightMode::Flight),
            FlightMode::Flight => {
                let falling = self.true_since(self.vertical_speed() < 0.0, self.settings.apogee_min_falling_time);
                let min_exceeded = elapsed > self.settings.min_time_to_apogee;
                let max_exceeded = elapsed > self.settings.max_time_to_apogee;
                ((min_exceeded && falling) || max_exceeded).then(|| FlightMode::RecoveryDrogue)
            }
            FlightMode::RecoveryDrogue => {
                match self.settings.main_output_mode {
                    MainOutputMode::AtApogee => (elapsed > recovery_duration).then(|| FlightMode::RecoveryMain),
                    MainOutputMode::BelowAltitude => {
                        let below_alt = self.altitude() < self.settings.main_output_deployment_altitude;
                        (elapsed > recovery_duration && below_alt).then(|| FlightMode::RecoveryMain)
                    },
                    MainOutputMode::Never => {
                        let condition = self.acceleration.map(|acc| (GRAVITY*0.95..GRAVITY*1.05).contains(&acc.magnitude())).unwrap_or(true);
                        let landed = self.true_since(condition, 1000);
                        (elapsed > recovery_duration && landed).then(|| FlightMode::Landed)
                    },
                }
            },
            FlightMode::RecoveryMain => {
                let condition = self.acceleration.map(|acc| (GRAVITY*0.95..GRAVITY*1.05).contains(&acc.magnitude())).unwrap_or(true) && self.vertical_speed().abs() < 1.0;
                let landed = self.true_since(condition, 1000);
                (elapsed > recovery_duration && landed).then(|| FlightMode::Landed)
            },
            FlightMode::Landed => None
        }
    }

    fn true_since(&mut self, cond: bool, duration: u32) -> bool {
        self.condition_true_since = match (cond, self.condition_true_since) {
            (true, None) => Some(self.time),
            (true, Some(t)) => Some(t),
            (false, _) => None
        };

        self.condition_true_since
            .map(|t| self.time.wrapping_sub(t) > duration)
            .unwrap_or(false)
    }
}
