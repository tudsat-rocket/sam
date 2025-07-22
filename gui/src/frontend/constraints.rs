use telemetry::Metric;

use crate::backend::Backend;

#[derive(Clone, Copy, PartialEq)]
pub enum ConstraintResult {
    NOMINAL,
    WARNING,
    DANGER
}

pub enum Constraint {
    Max(MaxConstraint),
    Min(MinConstraint),
    Some(SomeConstraint)
}

pub struct MaxConstraint {
    metric: Metric,
    max: f64,
    violation_result: ConstraintResult,
}

pub struct MinConstraint {
    metric: Metric,
    min: f64,
    violation_result: ConstraintResult,
}

pub struct SomeConstraint {
    metric: Metric,
    violation_result: ConstraintResult,
}

impl Constraint {

    pub fn check(&self, backend: &Backend) -> ConstraintResult{
        return match self {
            Constraint::Max(constraint) => constraint.check(backend),
            Constraint::Min(constraint) => constraint.check(backend),
            Constraint::Some(constraint) => constraint.check(backend),
        }
    }

    pub fn metric(&self) -> Metric{
        return match self {
            Constraint::Max(constraint) => constraint.metric,
            Constraint::Min(constraint) => constraint.metric,
            Constraint::Some(constraint) => constraint.metric,
        }
    }

}

impl MaxConstraint {
    pub fn new(metric: Metric, max: f64, violation_result: ConstraintResult) -> Self {
        Self { metric, max, violation_result }
    }

    pub fn check(&self, backend: &Backend) -> ConstraintResult {
        match backend.current_value(self.metric) {
            Some(value) => if value <= self.max {return ConstraintResult::NOMINAL} else {return self.violation_result}
            None => return ConstraintResult::NOMINAL,
        }
    }
}

impl MinConstraint {
    pub fn new(metric: Metric, min: f64, violation_result: ConstraintResult) -> Self {
        Self { metric, min, violation_result }
    }

    pub fn check(&self, backend: &Backend) -> ConstraintResult {
        match backend.current_value(self.metric) {
            Some(value) => if value >= self.min {return ConstraintResult::NOMINAL} else {return self.violation_result}
            None => return ConstraintResult::NOMINAL,
        }
    }
}

impl SomeConstraint {
    pub fn new(metric: Metric, violation_result: ConstraintResult) -> Self {
        Self { metric, violation_result }
    }

    pub fn check(&self, backend: &Backend) -> ConstraintResult {
        match backend.current_value(self.metric) {
            Some(_) => return ConstraintResult::NOMINAL,
            None => return self.violation_result,
        }
    }
}

