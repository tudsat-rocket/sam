use std::{
    collections::{HashMap, hash_map::Entry},
    ops::RangeInclusive,
    path::Path,
};

use serde::{Deserialize, Serialize};

use egui_plot::PlotPoint;
use strum::EnumDiscriminants;
use telemetry::{Metric, TelemetryMessageReader, TelemetrySchema};

use crate::{
    backend::storage::{
        constant::Constant, event_series::EventSeries, sample_series::SampleSeries, static_metrics::MetricTrait,
        storeable_value::StorableValue,
    },
    call_static_metric,
};

#[derive(Default)]
pub struct DataStore {
    has_initialized_local_metrics: bool,
    data: HashMap<Metric, Data>,
    first_time: Option<f64>,
    last_time: Option<f64>,
}

#[derive(Clone, Copy)]
pub struct DataPoint {
    pub time: f64,
    pub value: u64,
}

impl DataPoint {
    pub fn new<Value: StorableValue>(time: f64, value: Value) -> Self {
        Self {
            time: time,
            value: value.to_bits(),
        }
    }

    pub fn to_plot_point<Value: StorableValue>(&self) -> PlotPoint {
        return PlotPoint::new(self.time, Value::from_bits(self.value).to_float());
    }
}

pub trait DataType: Sized {
    fn init<M: MetricTrait>(value: M::Value, time: f64) -> Result<Data, &'static str>;
    fn append<M: MetricTrait>(&mut self, value: M::Value, time: f64) -> Result<(), &'static str>;
    fn current_value<M: MetricTrait>(&self, end: Option<f64>) -> M::Value; //TODO: Maybe add cutoff and return None afterwards, generally avoid uses of unwrap in the implementations
    fn first_time(&self) -> Option<f64>;
    fn last_time(&self) -> Option<f64>;
    fn data_type() -> DataDiscriminants;
    fn plot_metric<'a, M: MetricTrait>(
        &self,
        visible_range: &RangeInclusive<f64>,
        playback_end: Option<f64>,
    ) -> egui_plot::PlotPoints<'a>; //TODO: Create some kind of buffer to avoid copying every call
}

#[derive(EnumDiscriminants)]
pub enum Data {
    ///Data which is defined by a timeseries, where the value is unknown for timestamps which are not included in the series
    SampleSeries(SampleSeries),
    ///Data which can be changed by events (represented as PlotPoints) and stays constant until the next event/the current time
    EventSeries(EventSeries),
    ///A constant value, e.g., a property like MAX_RATED_TANK_PRESSURE
    Constant(Constant),
}

impl Data {
    fn from_file_entry<M: MetricTrait>(entry: Vec<DataPoint>) -> Result<Self, &'static str> {
        match M::DataType::data_type() {
            DataDiscriminants::SampleSeries => Ok(Data::SampleSeries(SampleSeries::from_file_entry::<M>(entry))),
            DataDiscriminants::EventSeries => Err("Loading event series from file is currently not supported"),
            DataDiscriminants::Constant => Err("Loading constants from file is currently not supported"),
        }
    }

    fn append<M: MetricTrait>(&mut self, value: M::Value, time: f64) -> Result<(), &'static str> {
        return match self {
            Data::SampleSeries(sample_series) => sample_series.append::<M>(value, time),
            Data::EventSeries(event_series) => event_series.append::<M>(value, time),
            Data::Constant(constant) => constant.append::<M>(value, time),
        };
    }

    fn current_value<M: MetricTrait>(&self, end: Option<f64>) -> M::Value {
        return match self {
            Data::SampleSeries(sample_series) => sample_series.current_value::<M>(end),
            Data::EventSeries(event_series) => event_series.current_value::<M>(end),
            Data::Constant(constant) => constant.current_value::<M>(end),
        };
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct DataStoreFile {
    timeseries: Vec<(Metric, Vec<(f64, f64)>)>,
    enums: Vec<(Metric, Vec<(f64, u8)>)>,
}

impl DataStore {
    fn ingest_value<M: MetricTrait>(&mut self, bits: u64, time: f64) -> Result<(), &'static str> {
        let value = M::Value::from_bits(bits);
        return self.set_value::<M>(value, time);
    }

    pub fn set_value<M: MetricTrait>(&mut self, value: M::Value, time: f64) -> Result<(), &'static str> {
        return match self.data.entry(M::metric()) {
            Entry::Occupied(mut occ) => occ.get_mut().append::<M>(value, time),
            Entry::Vacant(vac) => {
                vac.insert(M::DataType::init::<M>(value, time)?);
                Ok(())
            }
        };
    }

    pub fn current_value<M: MetricTrait>(&self, end: Option<f64>) -> Option<M::Value> {
        return self.data.get(&M::metric()).map(|data| data.current_value::<M>(end));
    }

    pub fn has_initialized_local_metrics(&self) -> bool {
        return self.has_initialized_local_metrics;
    }
    pub fn initialize_local_metrics(&mut self) {
        self.has_initialized_local_metrics = true;
    }

    pub fn ingest_message<const N: usize>(
        &mut self,
        schema: &'static TelemetrySchema,
        time: u32,
        message: heapless::Vec<u8, N>,
    ) {
        let t = (time as f64) / 1000.0;
        schema
            .receive(time, message, |metric, repr, reader: &mut TelemetryMessageReader<N>| {
                let _ = reader.read_value(repr).and_then(|bits| {
                    call_static_metric!(Self::ingest_value, <metric, >, self, bits, t).map_err(|s| print!("{s}"))
                });
            })
            .unwrap();

        self.last_time = Some(t);
    }

    pub fn first_time(&self) -> Option<f64> {
        self.first_time
    }

    pub fn last_time(&self) -> Option<f64> {
        self.last_time
    }

    pub fn plot_metric<'a>(
        &'a self,
        key: &Metric,
        bounds: egui_plot::PlotBounds,
        playback_end: Option<f64>,
    ) -> egui_plot::PlotPoints<'a> {
        // Since egui gives us the bounds from the last frame, we extend our
        // "visible" range slightly, to cover the edges while moving.

        let visible_range = bounds.range_x();
        let length = visible_range.end() - visible_range.start();
        let new_start = visible_range.start() - length * 0.2;
        let new_end = visible_range.end() + length * 0.2;
        let visible_range = new_start..=new_end;

        let Some(data) = self.data.get(key) else {
            return egui_plot::PlotPoints::Owned(vec![]);
        };

        return match data {
            Data::SampleSeries(sample_series) => {
                call_static_metric!(SampleSeries::plot_metric, <key, >, sample_series, &visible_range, playback_end)
            }
            Data::EventSeries(event_series) => {
                call_static_metric!(EventSeries::plot_metric, <key, >, event_series, &visible_range, playback_end)
            }
            Data::Constant(constant) => {
                call_static_metric!(Constant::plot_metric, <key, >, constant, &visible_range, playback_end)
            }
        };
    }

    pub fn enum_transitions<'a, E: TryFrom<u8>>(
        &'a self,
        key: &Metric,
        bounds: egui_plot::PlotBounds,
        playback_end: Option<f64>,
    ) -> Vec<(f64, E)> {
        // Since egui gives us the bounds from the last frame, we extend our
        // "visible" range slightly, to cover the edges while moving.

        return vec![];

        // let visible_range = bounds.range_x();
        // let length = visible_range.end() - visible_range.start();
        // let mut new_start = visible_range.start() - length * 0.2;
        // let mut new_end = visible_range.end() + length * 0.2;

        // if let Some(end) = playback_end {
        //     new_end = f64::min(new_end, end);
        //     new_start = f64::min(new_start, end);
        // }

        // let visible_range = new_start..=new_end;

        // let Some(ints) = self.enums.get(key) else {
        //     return vec![];
        // };

        // let start_i = ints.partition_point(|p| p.0 < *visible_range.start());
        // let end_i = ints.partition_point(|p| p.0 < *visible_range.end());
        // let start_i = usize::max(start_i, 1) - 1;
        // let end_i = usize::min(end_i + 1, usize::max(1, ints.len()) - 1);

        // ints[start_i..=end_i]
        //     .into_iter()
        //     .scan(None, |state: &mut Option<u8>, &(t, i)| {
        //         let keep = state.map(|s| s != i).unwrap_or(true);
        //         state.replace(i);
        //         Some((t, i, keep))
        //     })
        //     .filter_map(|(t, i, keep)| keep.then_some((t, i)).and_then(|(t, i)| i.try_into().ok().map(|e| (t, e))))
        //     .collect()
    }

    //TODO Hans: Timeseries and zip_timeseries are only used by map, maybe this should be handled differently?
    pub fn timeseries<'a>(
        &'a self,
        metric: &Metric,
        end: Option<f64>,
    ) -> Option<impl Iterator<Item = (f64, f64)> + ExactSizeIterator + std::fmt::Debug + use<'a>> {
        Some(vec![].into_iter())
        //     let Some(vec) = self
        //         .data
        //         .get(metric)
        //         .map(|data| match data {
        //             Data::SampleSeries(levels) => Some(levels),
        //             Data::EventSeries(_) => None,
        //             Data::Constant(_) => None,
        //         })
        //         .flatten()
        //         .map(|levels| levels[0].iter().map(|pp| (pp.x, pp.y)).collect::<Vec<_>>())
        //         .or(self.enums.get(metric).map(|series| series.iter().map(|(x, y)| (*x, *y as f64)).collect::<Vec<_>>()))
        //     else {
        //         return None;
        //     };

        //     let limited: Vec<(f64, f64)> = end
        //         .or(vec.last().map(|p| p.0))
        //         .map(move |end| {
        //             let partition_point = vec.partition_point(|p| p.0 < end);
        //             vec[..usize::min(partition_point, vec.len() - 1)].into_iter().copied().collect()
        //         })
        //         .unwrap_or_default();

        //     Some(limited.into_iter())
    }

    pub fn zip_timeseries<'a, const N: usize>(
        &'a self,
        metrics: [Metric; N],
        carry_forward: f64,
        end: Option<f64>,
    ) -> impl Iterator<Item = (f64, [Option<f64>; N])> + std::fmt::Debug + use<'a, N> {
        let iterators: Vec<_> = metrics.into_iter().map(|m| self.timeseries(&m, end)).collect();
        TimeseriesZipIterator::new(iterators.try_into().unwrap(), carry_forward)
    }

    pub fn save_to_file<P: AsRef<Path>>(&mut self, path: P) -> Result<(), std::io::Error> {
        todo!()
        // let ds = DataStoreFile {
        //     enums: self.enums.iter().map(|(k, v)| (*k, v.clone())).collect(),
        //     timeseries: self
        //         .data
        //         .iter()
        //         .map(|(k, d)| match d {
        //             Data::SampleSeries(timeseries) => Some((k, timeseries)),
        //             Data::EventSeries(_) => None,
        //             Data::Constant(_) => None,
        //         })
        //         .flatten()
        //         .map(|(k, v)| (*k, v[0].iter().map(|pp| (pp.x, pp.y)).collect()))
        //         .collect(),
        // };

        // std::fs::write(path, serde_json::to_string(&ds).unwrap())?;

        // Ok(())
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let raw: DataStoreFile = serde_json::from_slice(bytes).unwrap();

        let enum_data = raw.enums.into_iter().map(|(metric, entry)| {
            (metric, entry.into_iter().map(|(time, value)| DataPoint::new(time, value)).collect::<Vec<_>>())
        });
        let float_data = raw.timeseries.into_iter().map(|(metric, entry)| {
            (metric, entry.into_iter().map(|(time, value)| DataPoint::new(time, value)).collect::<Vec<_>>())
        });

        let data: HashMap<Metric, Data> = enum_data
            .chain(float_data)
            .map(|(metric, data_points)| {
                call_static_metric!(Data::from_file_entry, <metric, >, data_points)
                    .map(|data| (metric, data))
                    .map_err(|s| print!("{s}"))
            })
            .flatten()
            .collect();

        let first_time = data
            .iter()
            .map(|(_k, d)| match d {
                Data::SampleSeries(sample_series) => sample_series.first_time(),
                Data::EventSeries(event_series) => event_series.first_time(),
                Data::Constant(constant) => constant.first_time(),
            })
            .flatten()
            .fold(f64::INFINITY, |a, b| f64::min(a, b));

        let last_time = data
            .iter()
            .map(|(_k, d)| match d {
                Data::SampleSeries(sample_series) => sample_series.last_time(),
                Data::EventSeries(event_series) => event_series.last_time(),
                Data::Constant(constant) => constant.last_time(),
            })
            .flatten()
            .fold(f64::NEG_INFINITY, |a, b| f64::max(a, b));

        return Self {
            data,
            has_initialized_local_metrics: false,
            first_time: first_time.is_normal().then_some(first_time),
            last_time: last_time.is_normal().then_some(last_time),
        };
    }
}

#[derive(Debug)]
pub struct TimeseriesZipIterator<const N: usize, I>
where
    I: Iterator<Item = (f64, f64)> + std::fmt::Debug,
{
    iterators: [Option<std::iter::Peekable<I>>; N],
    carry_forward: f64,
    next_values: [Option<(f64, f64)>; N],
    last_values: [Option<(f64, f64)>; N],
}

impl<const N: usize, I> TimeseriesZipIterator<N, I>
where
    I: Iterator<Item = (f64, f64)>,
    I: std::fmt::Debug,
{
    pub fn new(iterators: [Option<I>; N], carry_forward: f64) -> Self {
        let mut iterators: Vec<_> = iterators.into_iter().map(|i| i.map(|i| i.peekable())).collect();

        let mut next_values: [Option<(f64, f64)>; N] = [None; N];
        for i in 0..N {
            next_values[i] = iterators[i].as_mut().map(|i| i.peek().copied()).flatten();
        }

        Self {
            iterators: iterators.try_into().unwrap(),
            carry_forward,
            next_values,
            last_values: [None; N],
        }
    }
}

impl<const N: usize, I: Iterator<Item = (f64, f64)> + std::fmt::Debug> Iterator for TimeseriesZipIterator<N, I> {
    type Item = (f64, [Option<f64>; N]);

    fn next(&mut self) -> Option<(f64, [Option<f64>; N])> {
        let next_time = self.next_values.iter().filter_map(|p| p.map(|p| p.0)).fold(f64::INFINITY, |a, b| a.min(b));
        if next_time.is_infinite() {
            return None;
        }

        let mut values: [Option<f64>; N] = [None; N];
        for i in 0..N {
            let Some(next) = self.next_values[i] else {
                continue;
            };

            if next.0 == next_time {
                // TODO: right now we assume that the "same time" has bit-identical f64 values.
                // if those values come from the same message, that should actually be the case. if
                // that ever is not the case, we might have to do a threshold comparison
                values[i] = self.iterators[i].as_mut().unwrap().next().map(|p| p.1);
                self.last_values[i] = Some(next);
                self.next_values[i] = self.iterators[i].as_mut().map(|i| i.peek()).flatten().copied();
            } else if self.last_values[i].map(|last| (next_time - last.0) < self.carry_forward).unwrap_or(false) {
                values[i] = self.last_values[i].map(|l| l.1);
            }
        }

        Some((next_time, values))
    }
}
