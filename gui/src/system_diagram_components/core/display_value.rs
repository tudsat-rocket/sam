use telemetry::{Metric, PressureSensorId, TemperatureSensorId};

use crate::backend::Backend;

#[derive(Clone)]
pub struct DisplayValue {
    pub desc: String,
    pub unit: Option<String>,
    pub val: JustifiedValue,
}

impl DisplayValue {

    pub fn new(desc: String, unit: Option<String>, val: JustifiedValue) -> Self {
        Self { desc, unit, val }
    }

    pub fn from_metric(metric: Metric, backend: &Backend) -> Self {
        match metric {
            Metric::Pressure(sensor_id) => DisplayValue::new("Pressure".to_string(), Some("bar".to_string()), JustifiedValue::new(backend.current_value(metric).map(|val| Value::F64(val)), Justification::Measured(Sensor::Pressure(sensor_id)))),
            _ => todo!(),
        }
    }

}

impl std::fmt::Display for DisplayValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}   {} [{}] ({})",
            self.desc,
            self.val.value.as_ref().map(|v| format!("{}", v)).unwrap_or("N/A".to_string()),
            self.unit.as_ref().map(|u| format!("[{}]", u)).unwrap_or("".to_string()),
            self.val.justification
        )
    }
}

#[derive(Clone)]
pub enum Sensor {
    Pressure(PressureSensorId),
    Temperature(TemperatureSensorId),
}

impl std::fmt::Display for Sensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Sensor::Pressure(pressure_sensor_id) => format!("{pressure_sensor_id:?}"),
                Sensor::Temperature(temperature_sensor_id) => format!("{temperature_sensor_id:?}"),
            }
        )
    }
}

#[derive(Clone)]
pub enum Justification {
    Measured(Sensor),
    Tested,
    Datasheet,
    None,
}

impl std::fmt::Display for Justification {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Justification::Measured(sensor) => format!("Measured by {sensor}"),
                Justification::Tested => format!("Tested"),
                Justification::Datasheet => format!("Datasheet"),
                Justification::None => format!("None"),
            }
        )
    }
}

#[derive(Clone)]
pub enum Value {
    F64(f64),
    F32(f32),
    U32(u32),
}

impl std::fmt::Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Value::F32(v) => format!{"{0:.1}",v},
                Value::U32(v) => format!{"{0:.1}",v},
                Value::F64(v) => format!("{0:.1}",v),
            }
        )
    }
}

#[derive(Clone)]
pub struct JustifiedValue {
    pub value: Option<Value>,
    pub justification: Justification,
}

impl JustifiedValue {
    pub fn new(value: Option<Value>, justification: Justification) -> Self {
        Self { value, justification }
    }
}