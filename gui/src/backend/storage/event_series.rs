use std::ops::RangeInclusive;

use egui_plot::PlotPoint;

use crate::backend::storage::{
    static_metrics::MetricTrait,
    store::{Data, DataDiscriminants, DataPoint, DataType},
    storeable_value::StorableValue,
};

pub struct EventSeries {
    data: Vec<DataPoint>,
}

impl DataType for EventSeries {
    fn init<M: MetricTrait>(value: M::Value, time: f64) -> Result<Data, &'static str> {
        return Ok(Data::EventSeries(Self {
            data: vec![DataPoint::new(time, value)],
        }));
    }

    fn append<M: MetricTrait>(&mut self, value: M::Value, time: f64) -> Result<(), &'static str> {
        self.data.push(DataPoint::new(time, value));
        Ok(())
    }

    fn current_value<M: MetricTrait>(&self, end: Option<f64>) -> M::Value {
        let last_value = self.data.last();
        let current_value = end
            .or(last_value.map(|p| p.time))
            .map(|end| {
                if end >= last_value.unwrap().time {
                    last_value.unwrap().value
                } else {
                    let partition_point = self.data.partition_point(|p| p.time < end);
                    self.data[usize::min(partition_point, self.data.len() - 1)].value
                }
            })
            .unwrap();
        return M::Value::from_bits(current_value);
    }

    fn first_time(&self) -> Option<f64> {
        return self.data.first().map(|data_point| data_point.time);
    }

    fn last_time(&self) -> Option<f64> {
        return self.data.last().map(|data_point| data_point.time);
    }

    fn data_type() -> DataDiscriminants {
        DataDiscriminants::EventSeries
    }

    fn plot_metric<'a, M: MetricTrait>(
        &self,
        visible_range: &RangeInclusive<f64>,
        playback_end: Option<f64>,
    ) -> egui_plot::PlotPoints<'a> {
        let display_start = playback_end.map(|end| end.min(*visible_range.start())).unwrap_or(*visible_range.start());
        let display_end = playback_end.map(|end| end.min(*visible_range.end())).unwrap_or(*visible_range.end());
        let start_i = self.data.partition_point(|p| p.time < display_start);
        let end_i = self.data.partition_point(|p| p.time < display_end);
        let mut raw: Vec<egui_plot::PlotPoint> =
            self.data[start_i..end_i].iter().map(|p| p.to_plot_point::<M::Value>()).collect();

        //Extend to display range if necessary
        if raw.len() == 0 {
            self.data
                .first()
                .map(|first| raw.push(PlotPoint::new(display_start, first.to_plot_point::<M::Value>().y)));
        }
        if playback_end.is_some() {
            raw.last().map(|last| PlotPoint::new(display_end, last.y)).map(|end_point| raw.push(end_point));
        }

        //Insert elements on change for correct pattern
        let mut processed = vec![];
        let mut raw_idx = 0;
        while raw_idx + 1 < raw.len() {
            processed.push(raw[raw_idx]);
            if raw[raw_idx] != raw[raw_idx + 1] {
                processed.push(PlotPoint::new(raw[raw_idx + 1].x, raw[raw_idx].y));
            }
            processed.push(raw[raw_idx + 1]);
            raw_idx += 1;
        }
        if let Some(last) = raw.last() {
            processed.push(*last);
        }
        return egui_plot::PlotPoints::Owned(processed);
    }
}
