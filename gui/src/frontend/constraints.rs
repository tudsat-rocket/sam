use egui::{Color32, RichText};
use strum::{EnumIter, VariantNames};
use telemetry::Metric;

use crate::backend::{
    Backend,
    storage::{static_metrics::MetricTrait, storeable_value::StorableValue},
};

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, EnumIter, VariantNames)]
pub enum ConstraintResult {
    NOMINAL,
    WARNING,
    DANGER,
}

impl ConstraintResult {
    pub fn symbol(&self) -> RichText {
        match self {
            ConstraintResult::NOMINAL => RichText::new("✔"),
            ConstraintResult::WARNING => RichText::new("❗"),
            ConstraintResult::DANGER => RichText::new("❌"),
        }
        .strong()
        .color(self.color())
    }

    pub fn string(&self) -> &'static str {
        match self {
            ConstraintResult::NOMINAL => "Nominal",
            ConstraintResult::WARNING => "Warning",
            ConstraintResult::DANGER => "Danger",
        }
    }

    pub fn color(&self) -> Color32 {
        match self {
            ConstraintResult::NOMINAL => Color32::GREEN,
            ConstraintResult::WARNING => Color32::ORANGE,
            ConstraintResult::DANGER => Color32::RED,
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
        self.constraint.evaluate(backend);
        self.result = self.constraint.check();
        return self.result;
    }

    pub fn evaluation_as_string(&self) -> String {
        return self.constraint.evaluation_as_string();
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
    fn evaluate(&mut self, backend: &Backend);
    fn check(&self) -> ConstraintResult;
    fn metric(&self) -> Metric;
    fn evaluation_as_string(&self) -> String;
}

pub struct ValuedConstraintData<C: Constraint> {
    constraint: C,
    violation_result: ConstraintResult,
}

impl<C: Constraint> ValuedConstraint for ValuedConstraintData<C> {
    fn evaluate(&mut self, backend: &Backend) {
        self.constraint.evaluate(backend);
    }

    fn check(&self) -> ConstraintResult {
        return if self.constraint.check() {
            ConstraintResult::NOMINAL
        } else {
            self.violation_result
        };
    }

    fn evaluation_as_string(&self) -> String {
        return self.constraint.evaluation_as_string();
    }

    fn metric(&self) -> Metric {
        return C::metric();
    }
}

pub trait ConstraintValue: Sized {
    type Type: StorableValue;

    fn cached(self) -> CachedConstraintValue<Self>;
    fn get(&self, backend: &Backend) -> Option<Self::Type>;
    fn as_string(&self, backend: &Backend) -> String;
}

impl<M: MetricTrait> ConstraintValue for M {
    type Type = M::Value;

    fn get(&self, backend: &Backend) -> Option<Self::Type> {
        return backend.current_value::<M>();
    }

    fn as_string(&self, backend: &Backend) -> String {
        return backend.current_value_as_string::<M>();
    }

    fn cached(self) -> CachedConstraintValue<Self> {
        CachedConstraintValue {
            constraint_value: self,
            cached_result: None,
        }
    }
}

pub struct ConstValue<V: Clone + StorableValue> {
    value: V,
}

impl<V: Clone + StorableValue> ConstValue<V> {
    pub fn new(value: V) -> Self {
        Self { value }
    }
}

impl<V: Clone + StorableValue> ConstraintValue for ConstValue<V> {
    type Type = V;

    fn get(&self, _backend: &Backend) -> Option<Self::Type> {
        return Some(self.value.clone());
    }

    fn as_string(&self, _backend: &Backend) -> String {
        self.value.to_string()
    }

    fn cached(self) -> CachedConstraintValue<Self> {
        CachedConstraintValue {
            constraint_value: self,
            cached_result: None,
        }
    }
}

pub struct CachedConstraintValue<V: ConstraintValue> {
    constraint_value: V,
    cached_result: Option<V::Type>,
}

impl<V: ConstraintValue> CachedConstraintValue<V> {
    pub fn update(&mut self, backend: &Backend) {
        self.cached_result = self.constraint_value.get(backend);
    }
}

pub trait Constraint {
    type Metric: MetricTrait;

    fn evaluate(&mut self, backend: &Backend);
    fn check(&self) -> bool;

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

    fn evaluation_as_string(&self) -> String;

    fn implies<C: Constraint>(self, rhs: C) -> ImpliesConstraint<Self, C>
    where
        Self: Sized,
    {
        ImpliesConstraint { lhs: self, rhs }
    }

    fn invert(self) -> NotConstraint<Self>
    where
        Self: Sized,
    {
        NotConstraint { constraint: self }
    }
}

pub trait ConstraintBuilder {
    type Metric: MetricTrait;

    fn is_some() -> SomeConstraint<Self::Metric> {
        SomeConstraint {
            cached_metric: Self::Metric::default().cached(),
        }
    }

    fn eq_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>() -> EqualsConstraint<Self::Metric, M> {
        EqualsConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: M::default().cached(),
        }
    }

    fn leq_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>()
    -> LessOrEqualConstraint<Self::Metric, M> {
        LessOrEqualConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: M::default().cached(),
        }
    }

    fn lt_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>() -> LessThanConstraint<Self::Metric, M>
    {
        LessThanConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: M::default().cached(),
        }
    }

    fn geq_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>()
    -> GreaterOrEqualConstraint<Self::Metric, M> {
        GreaterOrEqualConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: M::default().cached(),
        }
    }

    fn gt_metric<M: MetricTrait<Value = <Self::Metric as MetricTrait>::Value>>()
    -> GreaterThanConstraint<Self::Metric, M> {
        GreaterThanConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: M::default().cached(),
        }
    }

    fn eq_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> EqualsConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        EqualsConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: ConstValue::new(value).cached(),
        }
    }

    fn leq_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> LessOrEqualConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        LessOrEqualConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: ConstValue::new(value).cached(),
        }
    }

    fn lt_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> LessThanConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        LessThanConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: ConstValue::new(value).cached(),
        }
    }

    fn geq_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> GreaterOrEqualConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        GreaterOrEqualConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: ConstValue::new(value).cached(),
        }
    }

    fn gt_const(
        value: <Self::Metric as MetricTrait>::Value,
    ) -> GreaterThanConstraint<Self::Metric, ConstValue<<Self::Metric as MetricTrait>::Value>>
    where
        <Self::Metric as MetricTrait>::Value: Clone,
    {
        GreaterThanConstraint {
            cached_metric: Self::Metric::default().cached(),
            cached_constraint: ConstValue::new(value).cached(),
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

    fn evaluate(&mut self, backend: &Backend) {
        self.lhs.evaluate(backend);
        self.rhs.evaluate(backend);
    }

    fn check(&self) -> bool {
        return !self.lhs.check() || self.rhs.check();
    }
    fn evaluation_as_string(&self) -> String {
        format! {"({}) while ({})", self.rhs.evaluation_as_string(), self.lhs.evaluation_as_string()}
    }
}

#[derive(Default)]
pub struct NotConstraint<C: Constraint> {
    constraint: C,
}

impl<C: Constraint> Constraint for NotConstraint<C> {
    type Metric = C::Metric;

    fn evaluate(&mut self, backend: &Backend) {
        self.constraint.evaluate(backend);
    }

    fn check(&self) -> bool {
        return !self.constraint.check();
    }

    fn evaluation_as_string(&self) -> String {
        self.constraint.evaluation_as_string()
    }
}

pub struct GreaterOrEqualConstraint<M: MetricTrait, C: ConstraintValue<Type = M::Value>> {
    cached_metric: CachedConstraintValue<M>,
    cached_constraint: CachedConstraintValue<C>,
}

impl<M: MetricTrait, C: ConstraintValue<Type = M::Value>> Constraint for GreaterOrEqualConstraint<M, C> {
    type Metric = M;

    fn evaluate(&mut self, backend: &Backend) {
        self.cached_metric.update(backend);
        self.cached_constraint.update(backend);
    }

    fn check(&self) -> bool {
        return self.cached_metric.cached_result >= self.cached_constraint.cached_result;
    }

    fn evaluation_as_string(&self) -> String {
        let constraint_str =
            self.cached_constraint.cached_result.as_ref().map(|v| v.to_string()).unwrap_or("N/A".to_string());
        if self.check() {
            format!("at least {}", constraint_str)
        } else {
            format!("below {}", constraint_str)
        }
    }
}

pub struct GreaterThanConstraint<M: MetricTrait, C: ConstraintValue<Type = M::Value>> {
    cached_metric: CachedConstraintValue<M>,
    cached_constraint: CachedConstraintValue<C>,
}

impl<M: MetricTrait, C: ConstraintValue<Type = M::Value>> Constraint for GreaterThanConstraint<M, C> {
    type Metric = M;

    fn evaluate(&mut self, backend: &Backend) {
        self.cached_metric.update(backend);
        self.cached_constraint.update(backend);
    }

    fn check(&self) -> bool {
        return self.cached_metric.cached_result > self.cached_constraint.cached_result;
    }

    fn evaluation_as_string(&self) -> String {
        let constraint_str =
            self.cached_constraint.cached_result.as_ref().map(|v| v.to_string()).unwrap_or("N/A".to_string());
        if self.check() {
            format!("above {}", constraint_str)
        } else {
            format!("below {}", constraint_str)
        }
    }
}

pub struct LessOrEqualConstraint<M: MetricTrait, C: ConstraintValue<Type = M::Value>> {
    cached_metric: CachedConstraintValue<M>,
    cached_constraint: CachedConstraintValue<C>,
}

impl<M: MetricTrait, C: ConstraintValue<Type = M::Value>> Constraint for LessOrEqualConstraint<M, C> {
    type Metric = M;

    fn evaluate(&mut self, backend: &Backend) {
        self.cached_metric.update(backend);
        self.cached_constraint.update(backend);
    }

    fn check(&self) -> bool {
        return self.cached_metric.cached_result <= self.cached_constraint.cached_result;
    }

    fn evaluation_as_string(&self) -> String {
        let constraint_str =
            self.cached_constraint.cached_result.as_ref().map(|v| v.to_string()).unwrap_or("N/A".to_string());
        if self.check() {
            format!("at most {}", constraint_str)
        } else {
            format!("exceeds {}", constraint_str)
        }
    }
}

pub struct LessThanConstraint<M: MetricTrait, C: ConstraintValue<Type = M::Value>> {
    cached_metric: CachedConstraintValue<M>,
    cached_constraint: CachedConstraintValue<C>,
}

impl<M: MetricTrait, C: ConstraintValue<Type = M::Value>> Constraint for LessThanConstraint<M, C> {
    type Metric = M;

    fn evaluate(&mut self, backend: &Backend) {
        self.cached_metric.update(backend);
        self.cached_constraint.update(backend);
    }

    fn check(&self) -> bool {
        return self.cached_metric.cached_result < self.cached_constraint.cached_result;
    }

    fn evaluation_as_string(&self) -> String {
        let constraint_str =
            self.cached_constraint.cached_result.as_ref().map(|v| v.to_string()).unwrap_or("N/A".to_string());
        if self.check() {
            format!("below {}", constraint_str)
        } else {
            format!("exceeds {}", constraint_str)
        }
    }
}

pub struct SomeConstraint<M: MetricTrait> {
    cached_metric: CachedConstraintValue<M>,
}

impl<M: MetricTrait> Constraint for SomeConstraint<M> {
    type Metric = M;

    fn evaluate(&mut self, backend: &Backend) {
        self.cached_metric.update(backend);
    }

    fn check(&self) -> bool {
        return self.cached_metric.cached_result.is_some();
    }

    fn evaluation_as_string(&self) -> String {
        if self.check() {
            format!("is known")
        } else {
            format!("is N/A")
        }
    }
}

pub struct EqualsConstraint<M: MetricTrait, C: ConstraintValue<Type = M::Value>> {
    cached_metric: CachedConstraintValue<M>,
    cached_constraint: CachedConstraintValue<C>,
}

impl<M: MetricTrait, C: ConstraintValue<Type = M::Value>> Constraint for EqualsConstraint<M, C> {
    type Metric = M;

    fn evaluate(&mut self, backend: &Backend) {
        self.cached_metric.update(backend);
        self.cached_constraint.update(backend);
    }

    fn check(&self) -> bool {
        return self.cached_metric.cached_result == self.cached_constraint.cached_result;
    }

    fn evaluation_as_string(&self) -> String {
        let constraint_str =
            self.cached_constraint.cached_result.as_ref().map(|v| v.to_string()).unwrap_or("N/A".to_string());
        if self.check() {
            format!("is {}", constraint_str)
        } else {
            format!("is not {}", constraint_str)
        }
    }
}
