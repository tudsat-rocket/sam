use std::{collections::HashMap, hash::Hash, sync::LazyLock};

use bitflags::bitflags;
use telemetry::Metric;

use crate::{backend::Backend, frontend::constraints::{Constraint, ConstraintResult}};

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

pub struct MetricMonitor
{
    flags: HashMap<Metric, MonitorFlags>,
    //TODO Hans: Should this be in the front or backend?
    constraints: HashMap<Metric, Vec<Constraint>>
}

impl Default for MetricMonitor 
{
    fn default() -> Self {
        Self { 
            flags: Default::default(),
            constraints: Default::default(),
        }
    }
}

impl MetricMonitor
{

    pub fn is_pinned(&self, metric: &Metric) -> bool {
        return self.flags.get(metric).unwrap_or_default().contains(MonitorFlags::PINNED);
    }

    pub fn pin(&mut self, metric: Metric) {
        self.flags.entry(metric).or_default().insert(MonitorFlags::PINNED);
    }

    pub fn unpin(&mut self, metric: Metric) {
        self.flags.entry(metric).or_default().remove(MonitorFlags::PINNED);
    }

    fn pinned_metrics(&self) -> impl Iterator<Item = Metric> + use<'_>{
        return self.flags.iter().filter(|(_m, f)| f.contains(MonitorFlags::PINNED)).map(|(m, _f)| *m);
    }

    pub fn violates_constraint(&self, metric: &Metric, backend: &Backend) -> bool {
        return self.constraints.get(metric).map(|cs| cs.iter().map(|c| c.check(backend)).any(|r| r != ConstraintResult::NOMINAL)).unwrap_or_default();
    } 

    fn metrics_with_violated_constraints<'a>(&self, backend: &'a Backend) -> impl Iterator<Item = Metric> + use<'_, 'a>{
        return self.constraints.keys().filter(|m| self.violates_constraint(m, backend)).map(|m| *m);
    }

    pub fn metrics_to_display(&self, backend: &Backend) -> Vec<Metric>{
        return self.pinned_metrics().chain(self.metrics_with_violated_constraints(backend)).collect::<Vec<_>>();
    }

    pub fn add_constraint(&mut self, constraint: Constraint) {
        self.constraints.entry(constraint.metric()).or_default().push(constraint);
    }

}