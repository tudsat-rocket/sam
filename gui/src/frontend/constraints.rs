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
    constraint: Box<dyn ValuedConstraint>,
    result: ConstraintResult,
}

impl EvaluatedConstraint {
    pub fn new(constraint: Box<dyn ValuedConstraint>) -> Self {
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

pub trait ValuedConstraint {
    fn check(&self, backend: &Backend) -> ConstraintResult;
    fn metric(&self) -> Metric;
}

pub struct ValuedConstraintData<C: Constraint> {
    constraint: C,
    violation_result: ConstraintResult,
}

impl<C: Constraint> ValuedConstraint for ValuedConstraintData<C> {
    fn check(&self, backend: &Backend) -> ConstraintResult {
        return if self.constraint.check(backend) {
            ConstraintResult::NOMINAL
        } else {
            self.violation_result
        };
    }

    fn metric(&self) -> Metric {
        return C::metric();
    }
}

pub trait ConstraintValue<V> {
    fn get(&self, backend: &Backend) -> Option<V>;
}

impl<V, M: MetricTrait<Value = V>> ConstraintValue<V> for M {
    fn get(&self, backend: &Backend) -> Option<V> {
        return backend.current_value::<M>();
    }
}

pub struct ConstValue<V: Clone> {
    value: V,
}

impl<V: Clone> ConstValue<V> {
    pub fn new(value: V) -> Self {
        Self { value }
    }
}

impl<V: Clone> ConstraintValue<V> for ConstValue<V> {
    fn get(&self, _backend: &Backend) -> Option<V> {
        return Some(self.value.clone());
    }
}

pub trait Constraint {
    type Metric: MetricTrait;

    fn check(&self, backend: &Backend) -> bool;
    fn on_violation(self, violation_result: ConstraintResult) -> ValuedConstraintData<Self>
    where
        Self: Sized,
    {
        return ValuedConstraintData {
            constraint: self,
            violation_result,
        };
    }
    fn metric() -> Metric {
        return <Self::Metric as MetricTrait>::metric();
    }

    fn implies<C: Constraint>(self, rhs: C) -> ImpliesConstraint<Self, C>
    where
        Self: Sized,
    {
        ImpliesConstraint { lhs: self, rhs }
    }
}

pub trait ConstraintBuilder {
    type Metric: MetricTrait;

    fn is_some() -> SomeConstraint<Self::Metric> {
        Default::default()
    }

    fn eq_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>() -> EqualsConstraint<Self::Metric, M> {
        Default::default()
    }

    fn leq_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>()
    -> LessOrEqualConstraint<Self::Metric, M> {
        Default::default()
    }

    fn lt_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>() -> LessThanConstraint<Self::Metric, M>
    {
        Default::default()
    }

    fn geq_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>()
    -> GreaterOrEqualConstraint<Self::Metric, M> {
        Default::default()
    }

    fn gt_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>()
    -> GreaterThanConstraint<Self::Metric, M> {
        Default::default()
    }

    fn eq_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> EqualsConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        EqualsConstraint {
            metric: Default::default(),
            value: ConstValue::new(value),
        }
    }

    fn leq_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> LessOrEqualConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        LessOrEqualConstraint {
            metric: Default::default(),
            value: ConstValue::new(value),
        }
    }

    fn lt_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> LessThanConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        LessThanConstraint {
            metric: Default::default(),
            value: ConstValue::new(value),
        }
    }

    fn geq_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> GreaterOrEqualConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        GreaterOrEqualConstraint {
            metric: Default::default(),
            value: ConstValue::new(value),
        }
    }

    fn gt_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> GreaterThanConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        GreaterThanConstraint {
            metric: Default::default(),
            value: ConstValue::new(value),
        }
    }
}

impl<M: MetricTrait> ConstraintBuilder for M {
    type Metric = M;
}

#[derive(Default)]
pub struct ImpliesConstraint<C1: Constraint, C2: Constraint + ?Sized> {
    lhs: C1,
    rhs: C2,
}

impl<C1: Constraint, C2: Constraint> Constraint for ImpliesConstraint<C1, C2> {
    type Metric = C2::Metric;

    fn check(&self, backend: &Backend) -> bool {
        return !self.lhs.check(backend) || self.rhs.check(backend);
    }
}

#[derive(Default)]
pub struct NotConstraint<C: Constraint> {
    constraint: C,
}

impl<C: Constraint> Constraint for NotConstraint<C> {
    type Metric = C::Metric;

    fn check(&self, backend: &Backend) -> bool {
        return self.constraint.check(backend);
    }
}

#[derive(Default)]
pub struct GreaterOrEqualConstraint<M: MetricTrait, C: ConstraintValue<M::Value>> {
    metric: PhantomData<M>,
    value: C,
}

impl<M: MetricTrait, C: ConstraintValue<M::Value>> Constraint for GreaterOrEqualConstraint<M, C> {
    type Metric = M;

    fn check(&self, backend: &Backend) -> bool {
        let metric_val = backend.current_value::<M>();
        let constraint_val = self.value.get(backend);
        return metric_val >= constraint_val;
    }
}

#[derive(Default)]
pub struct GreaterThanConstraint<M: MetricTrait, C: ConstraintValue<M::Value>> {
    metric: PhantomData<M>,
    value: C,
}

impl<M: MetricTrait, C: ConstraintValue<M::Value>> Constraint for GreaterThanConstraint<M, C> {
    type Metric = M;

    fn check(&self, backend: &Backend) -> bool {
        let metric_val = backend.current_value::<M>();
        let constraint_val = self.value.get(backend);
        return metric_val > constraint_val;
    }
}

#[derive(Default)]
pub struct LessOrEqualConstraint<M: MetricTrait, C: ConstraintValue<M::Value>> {
    metric: PhantomData<M>,
    value: C,
}

impl<M: MetricTrait, C: ConstraintValue<M::Value>> Constraint for LessOrEqualConstraint<M, C> {
    type Metric = M;

    fn check(&self, backend: &Backend) -> bool {
        let metric_val = backend.current_value::<M>();
        let constraint_val = self.value.get(backend);
        return metric_val <= constraint_val;
    }
}

#[derive(Default)]
pub struct LessThanConstraint<M: MetricTrait, C: ConstraintValue<M::Value>> {
    metric: PhantomData<M>,
    value: C,
}

impl<M: MetricTrait, C: ConstraintValue<M::Value>> Constraint for LessThanConstraint<M, C> {
    type Metric = M;

    fn check(&self, backend: &Backend) -> bool {
        let metric_val = backend.current_value::<M>();
        let constraint_val = self.value.get(backend);
        return metric_val < constraint_val;
    }
}

#[derive(Default)]
pub struct SomeConstraint<M: MetricTrait> {
    metric: PhantomData<M>,
}

impl<M: MetricTrait> Constraint for SomeConstraint<M> {
    type Metric = M;

    fn check(&self, backend: &Backend) -> bool {
        return backend.current_value::<M>().is_some();
    }
}

#[derive(Default)]
pub struct EqualsConstraint<M: MetricTrait, C: ConstraintValue<M::Value>> {
    metric: PhantomData<M>,
    value: C,
}

impl<M: MetricTrait, C: ConstraintValue<M::Value>> Constraint for EqualsConstraint<M, C> {
    type Metric = M;

    fn check(&self, backend: &Backend) -> bool {
        let metric_val = backend.current_value::<M>();
        let constraint_val = self.value.get(backend);
        return metric_val == constraint_val;
    }
}
