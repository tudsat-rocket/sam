use core::num::Wrapping;

use nalgebra::*;
use num_traits::Pow;
use ahrs::Ahrs;
use filter::kalman::kalman_filter::KalmanFilter;

use crate::telemetry::FlightMode::{*, self};
use crate::settings::*;

const GRAVITY: f32 = 9.80665;

#[derive(Debug)]
pub struct StateEstimator {
    time: Wrapping<u32>,
    mode: FlightMode,
    mode_time: Wrapping<u32>,
    condition_true_since: Option<Wrapping<u32>>,
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
            time: Wrapping(0),
            mode: Idle,
            mode_time: Wrapping(0),
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
        time: Wrapping<u32>,
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
        let acc = match (accelerometer1, accelerometer2) {
            (Some(acc1), Some(acc2)) if acc1.amax() > 14.0 * GRAVITY && acc2.amax() > 14.0 * GRAVITY => Some(acc2),
            (Some(acc1), _)                                                                          => Some(acc1),
            (None, Some(acc2))                                                                       => Some(acc2),
            (None, None)                                                                             => None
        };
        self.acceleration = acc.map(|a| self.correct_orientation(&a));

        if let (Some(gyro), Some(acc), Some(mag)) = (&gyroscope, &self.acceleration, &magnetometer) {
            let gyro = self.correct_orientation(gyro);
            let mag = self.correct_orientation(mag);

            // Update the orientation estimator with IMU data
            self.orientation = self
                .ahrs
                .update(&(gyro * 3.14159 / 180.0), &acc, &mag)
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
        let altitude_baro = barometer
            .and_then(|a| (!a.is_nan()).then(|| a)) // NaN is not a valid altitude
            .and_then(|a| (a > -100.0 && a < 12_000.0).then(|| a)); // neither is -13000
        let accel_z = self.acceleration_world.map(|a| a.z)
            .and_then(|a| (!a.is_nan()).then(|| a));

        match (accel_z, altitude_baro) {
            (Some(accel_z), Some(altitude_baro)) => {
                self.kalman.predict(None, None, None, None);
                let z = Vector2::new(altitude_baro, accel_z);
                self.kalman.update(&z, None, None);
            },
            (Some(accel_z), None) => {
                // Use predicted altitude values, basically attempting to do inertial navigation.
                self.kalman.predict(None, None, None, None);
                let z = Vector2::new(self.altitude_asl(), accel_z);
                self.kalman.update(&z, None, None);
            }
            (None, Some(altitude_baro)) => {
                // Just assume acceleration is zero.
                self.kalman.predict(None, None, None, None);
                let z = Vector2::new(altitude_baro, 0.0);
                self.kalman.update(&z, None, None);
            },
            (None, None) => {
                // Do nothing, as long as this gap isn't too big and barometer values come back,
                // the Kalman filter should be able to recover from this.
            }
        }

        // Continuously reset ground altitude before arming.
        if mode < Armed {
            self.altitude_ground = self.altitude_asl();
        }

        // Only track maximum height during flight
        self.altitude_max = match mode {
            Idle | HardwareArmed | Armed => self.altitude_asl(),
            Flight | RecoveryDrogue | RecoveryMain => f32::max(self.altitude_max, self.altitude_asl()),
            Landed => self.altitude_max,
        }
    }

    pub fn acceleration_world(&self) -> Option<&Vector3<f32>> {
        self.acceleration_world.as_ref()
    }

    pub fn altitude_asl(&self) -> f32 {
        self.kalman.x.x
    }

    pub fn altitude_agl(&self) -> f32 {
        self.altitude_asl() - self.altitude_ground
    }

    pub fn vertical_speed(&self) -> f32 {
        self.kalman.x.y
    }

    pub fn vertical_accel(&self) -> f32 {
        self.kalman.x.z
    }

    pub fn time_in_mode(&self) -> u32 {
        (self.time - self.mode_time).0
    }

    /// Main flight logic. This function is responsible for deciding whether to switch to a new
    /// flight mode based on sensor data and therefore controls when the important events of the
    /// flight will take place.. Generally, all conditions have to be true for a given
    /// amount of time to avoid spurious decisions in case of weird sensor spikes/glitches.
    pub fn new_mode(&mut self, arm_voltage: u16, breakwire_open: Option<bool>) -> Option<FlightMode> {
        let vertical_accel = self.acceleration.map(|acc| acc.z).unwrap_or(0.0);
        let elapsed = self.time_in_mode();
        let recovery_duration = self.settings.outputs_warning_time + self.settings.outputs_high_time;

        let gravity_present = self.acceleration
            .map(|acc| (GRAVITY*0.95..GRAVITY*1.05).contains(&acc.magnitude()))
            .unwrap_or(true);
        let condition_landed = gravity_present && self.vertical_speed().abs() < 1.0;

        match self.mode {
            // We switch between Idle and HwArmed based on the voltage behind the arm switch
            FlightMode::Idle => self.true_since(arm_voltage >= 100, 100).then(|| FlightMode::HardwareArmed),
            FlightMode::HardwareArmed => self.true_since(arm_voltage < 10, 100).then(|| FlightMode::Idle),
            // Takeoff detection
            FlightMode::Armed => {
                // In the AND mode, we assume the breakwire to be open (pulled-out) when we lose
                // contact with the power module. This way, if the CAN bus becomes damaged
                // during/before takeoff, we simply fall back to acceleration-only detection.
                let acceleration = vertical_accel > self.settings.min_takeoff_acc;
                let condition = match self.settings.takeoff_detection_mode {
                    TakeoffDetectionMode::Acceleration             => acceleration,
                    TakeoffDetectionMode::Breakwire                => breakwire_open.unwrap_or(false),
                    TakeoffDetectionMode::AccelerationAndBreakwire => acceleration && breakwire_open.unwrap_or(true),
                    TakeoffDetectionMode::AccelerationOrBreakwire  => acceleration || breakwire_open.unwrap_or(false),
                };

                self.true_since(condition, self.settings.min_takeoff_acc_time).then(|| FlightMode::Flight)
            }
            // Deployment of drogue parachute, i.e. apogee detection
            FlightMode::Flight => {
                let falling = self.true_since(self.vertical_speed() < 0.0, self.settings.apogee_min_falling_time);
                let min_exceeded = elapsed > self.settings.min_time_to_apogee;
                let max_exceeded = elapsed > self.settings.max_time_to_apogee;
                ((min_exceeded && falling) || max_exceeded).then(|| FlightMode::RecoveryDrogue)
            }
            // Main parachute deployment, if required
            FlightMode::RecoveryDrogue => match self.settings.main_output_mode {
                MainOutputMode::AtApogee => (elapsed > recovery_duration).then(|| FlightMode::RecoveryMain),
                MainOutputMode::BelowAltitude => {
                    let condition = self.altitude_agl() < self.settings.main_output_deployment_altitude;
                    let below_alt = self.true_since(condition, 100);
                    let min_time = u32::max(recovery_duration, self.settings.min_time_to_main);
                    (elapsed > min_time && below_alt).then(|| FlightMode::RecoveryMain)
                },
                MainOutputMode::Never => {
                    let landed = self.true_since(condition_landed, 1000);
                    (elapsed > recovery_duration && landed).then(|| FlightMode::Landed)
                },
            },
            // Landing detection
            FlightMode::RecoveryMain => {
                let landed = self.true_since(condition_landed, 1000);
                (elapsed > recovery_duration && landed).then(|| FlightMode::Landed)
            },
            // Once in the Landed mode, the vehicle will not do anything by itself
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
            .map(|t| (self.time - t).0 > duration)
            .unwrap_or(false)
    }

    fn correct_orientation(&self, raw: &Vector3<f32>) -> Vector3<f32> {
        match self.settings.orientation {
            Orientation::ZUp => *raw,
            Orientation::ZDown => Vector3::new(-raw.x, raw.y, -raw.z),
        }
    }
}
