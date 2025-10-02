use std::ops::RangeInclusive;

use crate::backend::storage::{
    static_metrics::MetricTrait,
    store::{Data, DataDiscriminants, DataPoint, DataType},
    storeable_value::StorableValue,
};

const DOWNSAMPLING_FACTOR: usize = 16;
const DOWNSAMPLING_LEVELS: usize = 5;
const DOWNSAMPLING_THRESHOLD: usize = 10_000;

pub struct SampleSeries {
    /// Increasingly downsampled plot data. The first entry is the full data, followed by
    /// smaller and smaller vectors.
    data: [Vec<DataPoint>; DOWNSAMPLING_LEVELS],
}

impl DataType for SampleSeries {
    fn init<M: MetricTrait>(value: M::Value, time: f64) -> Result<Data, &'static str> {
        let mut data: [Vec<DataPoint>; DOWNSAMPLING_LEVELS] = Default::default();
        data[0].push(DataPoint::new(time, value));
        return Ok(Data::SampleSeries(Self { data }));
    }

    fn append<M: MetricTrait>(&mut self, value: M::Value, time: f64) -> Result<(), &'static str> {
        // Extend the first cache level
        self.data[0].push(DataPoint::new(time, value));

        // For each of the downsampled ones, remove the last item and extend from previous level.
        for i in 1..DOWNSAMPLING_LEVELS {
            let j = usize::max(1, self.data[i].len()) - 1;
            let j = (j / 2) * 2;
            self.data[i].truncate(j);
            let downsampled_tail = Self::downsample_points::<M::Value>(&self.data[i - 1][j * DOWNSAMPLING_FACTOR..]);
            self.data[i].extend(downsampled_tail);
        }

        Ok(())
    }

    fn current_value<M: MetricTrait>(&self, end: Option<f64>) -> M::Value {
        let last_value = self.data[0].last();
        println!("{:?}, {:?}", last_value.unwrap().time, end);
        let current_value = end
            .or(last_value.map(|p| p.time))
            .map(|end| {
                if end >= last_value.unwrap().time {
                    last_value.unwrap().value
                } else {
                    let partition_point = self.data[0].partition_point(|p| p.time < end);
                    self.data[0][usize::min(partition_point, self.data[0].len() - 1)].value
                }
            })
            .unwrap();
        return M::Value::from_bits(current_value);
    }

    fn first_time(&self) -> Option<f64> {
        return self.data[0].first().map(|data_point| data_point.time);
    }

    fn last_time(&self) -> Option<f64> {
        return self.data[0].last().map(|data_point| data_point.time);
    }

    fn data_type() -> DataDiscriminants {
        DataDiscriminants::SampleSeries
    }

    fn plot_metric<'a, M: MetricTrait>(
        &self,
        visible_range: &RangeInclusive<f64>,
        playback_end: Option<f64>,
    ) -> egui_plot::PlotPoints<'a> {
        let display_start = playback_end.map(|end| end.min(*visible_range.start())).unwrap_or(*visible_range.start());
        let display_end = playback_end.map(|end| end.min(*visible_range.end())).unwrap_or(*visible_range.end());

        // We start with our first cache level, the original data
        let mut vec = &self.data[0];
        let mut start_i = vec.partition_point(|p| p.time < display_start);
        let mut end_i = vec.partition_point(|p| p.time < display_end);

        // As long as we have too many points, we go up in cache levels.
        for i in 1..DOWNSAMPLING_LEVELS {
            if end_i - start_i < DOWNSAMPLING_THRESHOLD {
                break;
            }

            vec = &self.data[i];
            start_i /= DOWNSAMPLING_FACTOR;
            end_i /= DOWNSAMPLING_FACTOR;
        }

        // Extend by one in either direction so the line from the last point
        // onscreen to the next point offscreen is still drawn.
        let start_i = usize::max(start_i, 1) - 1;
        let end_i = usize::min(end_i + 1, usize::max(1, vec.len()) - 1);
        return egui_plot::PlotPoints::Owned(
            vec[start_i..end_i].iter().map(|data_point| data_point.to_plot_point::<M::Value>()).collect(),
        );
    }
}

impl SampleSeries {
    pub fn from_file_entry<M: MetricTrait>(entry: Vec<DataPoint>) -> Self {
        let mut data: [Vec<DataPoint>; DOWNSAMPLING_LEVELS] = Default::default();
        data[0] = entry;
        for i in 1..DOWNSAMPLING_LEVELS {
            data[i] = Self::downsample_points::<M::Value>(&data[i - 1]);
        }
        return Self { data };
    }

    fn downsample_points<Value: StorableValue + PartialOrd>(data: &[DataPoint]) -> Vec<DataPoint> {
        data.chunks(DOWNSAMPLING_FACTOR * 2)
            .flat_map(|chunk| {
                let (min_i, min) = chunk
                    .iter()
                    .enumerate()
                    .reduce(|min, current| {
                        if Value::from_bits(min.1.value) < Value::from_bits(current.1.value) {
                            min
                        } else {
                            current
                        }
                    })
                    .unwrap();
                let (max_i, max) = chunk
                    .iter()
                    .enumerate()
                    .reduce(|min, current| {
                        if Value::from_bits(min.1.value) < Value::from_bits(current.1.value) {
                            min
                        } else {
                            current
                        }
                    })
                    .unwrap();
                match (min_i, max_i) {
                    (i, j) if i < j => [*min, *max],
                    _ => [*max, *min],
                }
            })
            .collect()
    }
}
