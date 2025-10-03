use std::{collections::HashMap, hash::Hash, sync::LazyLock};

use bitflags::bitflags;
use telemetry::Metric;

use crate::{
    backend::Backend,
    frontend::constraints::{Constraint, ConstraintResult, EvaluatedConstraint},
};

bitflags! {
    pub struct MonitorFlags : u8 {
        const PINNED = 1 << 0;
    }
}

impl Default for MonitorFlags {
    fn default() -> Self {
        Self(Default::default())
    }
}

impl<'a> Default for &'a MonitorFlags {
    fn default() -> &'a MonitorFlags {
        static VALUE: LazyLock<MonitorFlags> = LazyLock::new(|| MonitorFlags::default());
        &VALUE
    }
}

pub struct MetricMonitor {
    active_constraint_mask: Vec<ConstraintResult>,
    flags: HashMap<Metric, MonitorFlags>,
    //TODO Hans: Should this be in the front or backend?
    constraints: HashMap<Metric, Vec<EvaluatedConstraint>>,
}

impl Default for MetricMonitor {
    fn default() -> Self {
        Self {
            active_constraint_mask: vec![ConstraintResult::WARNING, ConstraintResult::DANGER],
            flags: Default::default(),
            constraints: Default::default(),
        }
    }
}

impl MetricMonitor {
    pub fn is_pinned(&self, metric: &Metric) -> bool {
        return self.flags.get(metric).unwrap_or_default().contains(MonitorFlags::PINNED);
    }

    pub fn pin(&mut self, metric: Metric) {
        self.flags.entry(metric).or_default().insert(MonitorFlags::PINNED);
    }

    pub fn unpin(&mut self, metric: Metric) {
        self.flags.entry(metric).or_default().remove(MonitorFlags::PINNED);
    }

    pub fn pinned_metrics(&self) -> Vec<&Metric> {
        return self.flags.iter().filter(|(_m, f)| f.contains(MonitorFlags::PINNED)).map(|(m, _f)| m).collect();
    }

    pub fn evaluate_constraints(&mut self, backend: &Backend) {
        for ecs in self.constraints.values_mut() {
            for ec in ecs {
                ec.evaluate(backend);
            }
        }
    }

    pub fn constraint_results(&self) -> &HashMap<Metric, Vec<EvaluatedConstraint>> {
        return &self.constraints;
    }

    pub fn active_constraint_mask(&self) -> &Vec<ConstraintResult> {
        return &self.active_constraint_mask;
    }

    pub fn active_constraint_mask_mut(&mut self) -> &mut Vec<ConstraintResult> {
        return &mut self.active_constraint_mask;
    }

    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) {
        self.constraints.entry(constraint.metric()).or_default().push(EvaluatedConstraint::new(constraint));
    }

    pub fn strongest_constraint_result(&self, metric: &Metric) -> Option<&EvaluatedConstraint> {
        return self.constraints.get(metric).map(|res| res.iter().max()).unwrap_or_default();
    }
}
