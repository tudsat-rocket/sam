use std::ops::RangeInclusive;

use crate::backend::storage::{
    static_metrics::MetricTrait,
    store::{Data, DataDiscriminants, DataType},
    storeable_value::StorableValue,
};
pub struct Constant {
    data: u64,
}

impl DataType for Constant {
    fn init<M: MetricTrait>(value: M::Value, _time: f64) -> Result<Data, &'static str> {
        return Ok(Data::Constant(Self { data: value.to_bits() }));
    }

    fn append<M: MetricTrait>(&mut self, _value: M::Value, _time: f64) -> Result<(), &'static str> {
        return Err("Cannot change a constant value!");
    }

    fn current_value<M: MetricTrait>(&self, _end: Option<f64>) -> M::Value {
        return M::Value::from_bits(self.data);
    }

    fn first_time(&self) -> Option<f64> {
        return None;
    }

    fn last_time(&self) -> Option<f64> {
        return None;
    }

    fn data_type() -> DataDiscriminants {
        DataDiscriminants::Constant
    }

    fn plot_metric<'a, M: MetricTrait>(
        &self,
        visible_range: &RangeInclusive<f64>,
        _playback_end: Option<f64>,
    ) -> egui_plot::PlotPoints<'a> {
        return egui_plot::PlotPoints::Owned(vec![
            egui_plot::PlotPoint::new(*visible_range.start(), M::Value::from_bits(self.data).to_float()),
            egui_plot::PlotPoint::new(*visible_range.end(), M::Value::from_bits(self.data).to_float()),
        ]);
    }
}
