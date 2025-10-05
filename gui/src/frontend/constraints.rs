use std::marker::PhantomData;

use egui::{Color32, RichText};
use strum::{EnumIter, VariantNames};
use telemetry::Metric;

use crate::backend::{Backend, storage::static_metrics::MetricTrait};

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, EnumIter, VariantNames)]
pub enum ConstraintResult {
    NOMINAL,
    WARNING,
    DANGER,
}

impl ConstraintResult {
    pub fn symbol(&self) -> RichText {
        match self {
            ConstraintResult::NOMINAL => RichText::new("✔").strong().color(Color32::GREEN),
            ConstraintResult::WARNING => RichText::new("❗").strong().color(Color32::ORANGE),
            ConstraintResult::DANGER => RichText::new("❌").strong().color(Color32::RED),
        }
    }
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

pub struct NotConstraint<C: Constraint> {
    constraint: C,
    violation_result: ConstraintResult,
}

impl<C: Constraint> NotConstraint<C> {
    pub fn new(constraint: C, violation_result: ConstraintResult) -> Self {
        Self {
            constraint,
            violation_result,
        }
    }
}

impl<C: Constraint> Constraint for NotConstraint<C> {
    fn check(&self, backend: &Backend) -> ConstraintResult {
        if self.constraint.check(backend) == ConstraintResult::NOMINAL {
            return self.violation_result;
        } else {
            return ConstraintResult::NOMINAL;
        }
    }

    fn metric(&self) -> Metric {
        return self.constraint.metric();
    }
}

pub struct MinConstraint<M: MetricTrait, C: MetricTrait> {
    metric: PhantomData<M>,
    min: PhantomData<C>,
    violation_result: ConstraintResult,
}

impl<M: MetricTrait, C: MetricTrait> MinConstraint<M, C> {
    pub fn new(violation_result: ConstraintResult) -> Self {
        Self {
            metric: PhantomData,
            min: PhantomData,
            violation_result,
        }
    }
}

impl<M: MetricTrait, C: MetricTrait<Value = M::Value>> Constraint for MinConstraint<M, C> {
    fn check(&self, backend: &Backend) -> ConstraintResult {
        let metric_val = backend.current_value::<M>();
        let constraint_val = backend.current_value::<C>();
        if metric_val >= constraint_val {
            return ConstraintResult::NOMINAL;
        } else {
            return self.violation_result;
        };
    }

    fn metric(&self) -> Metric {
        return M::metric();
    }
}

pub struct MaxConstraint<M: MetricTrait, C: MetricTrait> {
    metric: PhantomData<M>,
    max: PhantomData<C>,
    violation_result: ConstraintResult,
}

impl<M: MetricTrait, C: MetricTrait> MaxConstraint<M, C> {
    pub fn new(violation_result: ConstraintResult) -> Self {
        Self {
            metric: PhantomData,
            max: PhantomData,
            violation_result,
        }
    }
}

impl<M: MetricTrait, C: MetricTrait<Value = M::Value>> Constraint for MaxConstraint<M, C> {
    fn check(&self, backend: &Backend) -> ConstraintResult {
        let metric_val = backend.current_value::<M>();
        let constraint_val = backend.current_value::<C>();
        if metric_val <= constraint_val {
            return ConstraintResult::NOMINAL;
        } else {
            return self.violation_result;
        };
    }

    fn metric(&self) -> Metric {
        return M::metric();
    }
}

pub struct SomeConstraint<M: MetricTrait> {
    metric: PhantomData<M>,
    violation_result: ConstraintResult,
}

impl<M: MetricTrait> SomeConstraint<M> {
    pub fn new(violation_result: ConstraintResult) -> Self {
        Self {
            metric: PhantomData,
            violation_result,
        }
    }
}

impl<M: MetricTrait> Constraint for SomeConstraint<M> {
    fn check(&self, backend: &Backend) -> ConstraintResult {
        match backend.current_value::<M>() {
            Some(_) => return ConstraintResult::NOMINAL,
            None => return self.violation_result,
        }
    }

    fn metric(&self) -> Metric {
        return M::metric();
    }
}

pub struct EqualsConstraint<M: MetricTrait, C: MetricTrait> {
    metric: PhantomData<M>,
    value: PhantomData<C>,
    violation_result: ConstraintResult,
}

impl<M: MetricTrait, C: MetricTrait> EqualsConstraint<M, C> {
    pub fn new(violation_result: ConstraintResult) -> Self {
        Self {
            metric: PhantomData,
            value: PhantomData,
            violation_result,
        }
    }
}

impl<M: MetricTrait, C: MetricTrait<Value = M::Value>> Constraint for EqualsConstraint<M, C> {
    fn check(&self, backend: &Backend) -> ConstraintResult {
        let metric_val = backend.current_value::<M>();
        let constraint_val = backend.current_value::<C>();
        if metric_val == constraint_val {
            return ConstraintResult::NOMINAL;
        } else {
            return self.violation_result;
        };
    }

    fn metric(&self) -> Metric {
        return M::metric();
    }
}
