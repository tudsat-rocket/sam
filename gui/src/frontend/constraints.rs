use strum::{EnumIter, VariantNames};
use telemetry::Metric;

use crate::backend::Backend;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, EnumIter, VariantNames)]
pub enum ConstraintResult {
    NOMINAL,
    WARNING,
    DANGER,
}

pub struct EvaluatedConstraint {
    constraint: Box<dyn Constraint>,
    result: ConstraintResult,
}

impl EvaluatedConstraint {
    pub fn new(constraint: Box<dyn Constraint>) -> Self {
        Self {
            constraint,
            result: ConstraintResult::NOMINAL,
        }
    }

    pub fn evaluate(&mut self, backend: &Backend) -> ConstraintResult {
        self.result = self.constraint.check(backend);
        return self.result;
    }

    pub fn result(&self) -> &ConstraintResult {
        return &self.result;
    }
}

impl PartialEq for EvaluatedConstraint {
    fn eq(&self, other: &Self) -> bool {
        return self.result == other.result;
    }
}

impl Eq for EvaluatedConstraint {}

impl Ord for EvaluatedConstraint {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        return self.result.cmp(&other.result);
    }
}

impl PartialOrd for EvaluatedConstraint {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        return Some(self.cmp(other));
    }
}

pub trait Constraint {
    fn check(&self, backend: &Backend) -> ConstraintResult;
    fn metric(&self) -> Metric;
}

// pub struct MaxConstraint {
//     metric: Metric,
//     max: f64,
//     violation_result: ConstraintResult,
// }

// pub struct MinConstraint {
//     metric: Metric,
//     min: f64,
//     violation_result: ConstraintResult,
// }

// pub struct SomeConstraint {
//     metric: Metric,
//     violation_result: ConstraintResult,
// }

// pub struct EqualsConstraint<T> {
//     metric: Metric,
//     expected: T,
//     violation_result: ConstraintResult,
// }

// impl MaxConstraint {
//     pub fn new(metric: Metric, max: f64, violation_result: ConstraintResult) -> Self {
//         Self {
//             metric,
//             max,
//             violation_result,
//         }
//     }
// }

// impl Constraint for MaxConstraint {
//     fn check(&self, backend: &Backend) -> ConstraintResult {
//         match backend.current_value(self.metric) {
//             Some(value) => {
//                 if value <= self.max {
//                     return ConstraintResult::NOMINAL;
//                 } else {
//                     return self.violation_result;
//                 }
//             }
//             None => return ConstraintResult::NOMINAL,
//         }
//     }

//     fn metric(&self) -> Metric {
//         return self.metric;
//     }
// }

// impl MinConstraint {
//     pub fn new(metric: Metric, min: f64, violation_result: ConstraintResult) -> Self {
//         Self {
//             metric,
//             min,
//             violation_result,
//         }
//     }
// }

// impl Constraint for MinConstraint {
//     fn check(&self, backend: &Backend) -> ConstraintResult {
//         match backend.current_value(self.metric) {
//             Some(value) => {
//                 if value >= self.min {
//                     return ConstraintResult::NOMINAL;
//                 } else {
//                     return self.violation_result;
//                 }
//             }
//             None => return ConstraintResult::NOMINAL,
//         }
//     }

//     fn metric(&self) -> Metric {
//         return self.metric;
//     }
// }

// impl SomeConstraint {
//     pub fn new(metric: Metric, violation_result: ConstraintResult) -> Self {
//         Self {
//             metric,
//             violation_result,
//         }
//     }
// }

// impl Constraint for SomeConstraint {
//     fn check(&self, backend: &Backend) -> ConstraintResult {
//         match backend.current_value(self.metric) {
//             Some(_) => return ConstraintResult::NOMINAL,
//             None => return self.violation_result,
//         }
//     }

//     fn metric(&self) -> Metric {
//         return self.metric;
//     }
// }

// impl<T> EqualsConstraint<T> {
//     pub fn new(metric: Metric, expected: T, violation_result: ConstraintResult) -> Self {
//         Self {
//             metric,
//             expected,
//             violation_result,
//         }
//     }
// }

// impl<T> Constraint for EqualsConstraint<T>
// where
//     T: TryFrom<u8> + PartialEq,
//     <T as TryFrom<u8>>::Error: std::fmt::Debug,
// {
//     fn check(&self, backend: &Backend) -> ConstraintResult {
//         match backend.current_enum::<T>(self.metric) {
//             Some(val) => {
//                 if val == self.expected {
//                     return ConstraintResult::NOMINAL;
//                 } else {
//                     return self.violation_result;
//                 }
//             }
//             None => return ConstraintResult::NOMINAL,
//         }
//     }

//     fn metric(&self) -> Metric {
//         return self.metric;
//     }
// }
