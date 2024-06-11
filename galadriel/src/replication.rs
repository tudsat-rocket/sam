use std::collections::VecDeque;

use crate::SensorData;

pub struct Replication {
    time: u32,
    delta_time: u32,
    last_measurement: SensorData,
    remaining_measurements: VecDeque<SensorData>,
}

impl Replication {
    pub fn new(delta_time: u32, mut measurements: VecDeque<SensorData>) -> Self {
        let last_measurement = measurements.pop_front().unwrap();

        Self {
            time: last_measurement.time,
            delta_time,
            last_measurement,
            remaining_measurements: measurements
        }
    }
}

impl Iterator for Replication {
    type Item = SensorData;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining_measurements.is_empty() {
            return None;
        }

        let last = &self.last_measurement;
        Some(if self.remaining_measurements.front().map(|n| self.time + self.delta_time >= n.time).unwrap_or(false) {
            let next = self.remaining_measurements.pop_front().unwrap();
            self.time += self.delta_time;
            SensorData {
                time: self.time,
                gyroscope: next.gyroscope,
                accelerometer1: next.accelerometer1,
                accelerometer2: next.accelerometer2,
                magnetometer: next.magnetometer,
                lp_filtered_pressure: next.pressure,
                pressure: next.pressure,
            }
        } else {
            let next = self.remaining_measurements.front().unwrap();
            let t = ((self.time - last.time) as f32) / ((next.time - last.time) as f32);
            self.time += self.delta_time;
            SensorData {
                time: self.time,
                gyroscope: last.gyroscope.zip(next.gyroscope).map(|(l, n)| l.lerp(&n, t)),
                accelerometer1: last.accelerometer1.zip(next.accelerometer1).map(|(l, n)| l.lerp(&n, t)),
                accelerometer2: last.accelerometer2.zip(next.accelerometer2).map(|(l, n)| l.lerp(&n, t)),
                magnetometer: last.magnetometer.zip(next.magnetometer).map(|(l, n)| l.lerp(&n, t)),
                lp_filtered_pressure: next.pressure,
                pressure: last.pressure.zip(next.pressure).map(|(l, n)| l + (n - l)*t),
            }
            // TODO: linear interpolation is probably not the best choice here.
        })
    }
}
