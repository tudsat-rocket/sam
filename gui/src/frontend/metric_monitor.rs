use std::{collections::HashMap, hash::Hash, sync::LazyLock};

use bitflags::bitflags;

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

pub struct MetricMonitor<M>
{
    map: HashMap<M, MonitorFlags>,
}

impl<M> Default for MetricMonitor<M> 
{
    fn default() -> Self {
        Self { map: Default::default() }
    }
}

impl<M> MetricMonitor<M> 
    where 
        M: Eq + Hash + Copy
{

    pub fn default() -> Self {
        Self { map: HashMap::default() }
    }

    pub fn is_pinned(&self, metric: &M) -> bool {
        return self.map.get(metric).unwrap_or_default().contains(MonitorFlags::PINNED);
    }

    pub fn pin(&mut self, metric: M) {
        self.map.entry(metric).or_default().insert(MonitorFlags::PINNED);
    }

    pub fn unpin(&mut self, metric: M) {
        self.map.entry(metric).or_default().remove(MonitorFlags::PINNED);
    }

    pub fn monitored_metrics(&self) -> Vec<M> {
        return self.map.iter().filter(|(_m, f)| f.contains(MonitorFlags::PINNED)).map(|(m, _f)| *m).collect::<Vec<_>>();
    }

}