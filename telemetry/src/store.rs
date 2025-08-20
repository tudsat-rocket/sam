use std::{collections::HashMap, path::Path};

use serde::{Deserialize, Serialize};

use egui_plot::PlotPoint;

use crate::*;

const DOWNSAMPLING_FACTOR: usize = 16;
const DOWNSAMPLING_LEVELS: usize = 5;
const DOWNSAMPLING_THRESHOLD: usize = 10_000;

#[derive(Default)]
pub struct DataStore {
    /// Increasingly downsampled plot data. The first entry is the full data, followed by
    /// smaller and smaller vectors.
    timeseries: HashMap<Metric, Data>,
    enums: HashMap<Metric, Vec<(f64, u8)>>,
    first_time: Option<f64>,
    last_time: Option<f64>,
}

enum Data {
    Timeseries([Vec<PlotPoint>; DOWNSAMPLING_LEVELS]),
    Constant(f64),
}

impl Default for Data {
    fn default() -> Self {
        return Data::Timeseries(vec![vec![]; DOWNSAMPLING_LEVELS].try_into().unwrap());
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct DataStoreFile {
    timeseries: Vec<(Metric, Vec<(f64, f64)>)>,
    enums: Vec<(Metric, Vec<(f64, u8)>)>,
}

impl DataStore {
    pub fn set_const_float(&mut self, metric: Metric, values: f64) {
        self.timeseries.entry(metric).or_insert(Data::Constant(values));
    }

    fn ingest_enum(&mut self, metric: Metric, time: f64, value: u8) {
        let entry = self.enums.entry(metric).or_default();
        entry.push((time, value));
    }

    fn ingest_float(&mut self, metric: Metric, time: f64, value: impl Into<f64>) {
        let data = self.timeseries.entry(metric).or_insert(Data::default());

        let levels = match data {
            Data::Timeseries(timeseries) => timeseries,
            Data::Constant(_) => return,
        };

        // Extend the first cache level
        levels[0].push(PlotPoint::new(time, value));

        // For each of the downsampled ones, remove the last item and extend from previous level.
        for i in 1..DOWNSAMPLING_LEVELS {
            let j = usize::max(1, levels[i].len()) - 1;
            let j = (j / 2) * 2;
            levels[i].truncate(j);
            let downsampled_tail = Self::downsample_points(&levels[i - 1][j * DOWNSAMPLING_FACTOR..]);
            levels[i].extend(downsampled_tail);
        }
    }

    pub fn ingest_message<const N: usize>(
        &mut self,
        schema: &'static TelemetrySchema,
        time: u32,
        message: heapless::Vec<u8, N>,
    ) {
        let t = (time as f64) / 1000.0;
        schema
            .receive(time, message, |metric, repr, reader: &mut TelemetryMessageReader<N>| match repr {
                Representation::Enum { bits: _ } => {
                    self.ingest_enum(metric, t, reader.read_enum(repr).unwrap());
                }
                _ => {
                    self.ingest_float(metric, t, reader.read_float(repr).unwrap());
                }
            })
            .unwrap();

        self.last_time = Some(t);
    }

    fn downsample_points(data: &[PlotPoint]) -> Vec<PlotPoint> {
        data.chunks(DOWNSAMPLING_FACTOR * 2)
            .flat_map(|chunk| {
                let (min_i, min) = chunk.iter().enumerate().min_by(|(_, a), (_, b)| a.y.total_cmp(&b.y)).unwrap();
                let (max_i, max) = chunk.iter().enumerate().max_by(|(_, a), (_, b)| a.y.total_cmp(&b.y)).unwrap();
                match (min_i, max_i) {
                    (i, j) if i < j => [*min, *max],
                    _ => [*max, *min],
                }
            })
            .collect()
    }

    pub fn first_time(&self) -> Option<f64> {
        self.first_time
    }

    pub fn last_time(&self) -> Option<f64> {
        self.last_time
    }

    pub fn current_float_value(&self, metric: Metric, end: Option<f64>) -> Option<f64> {
        let Some(data) = self.timeseries.get(&metric) else {
            return None;
        };

        return match data {
            Data::Timeseries(timeseries) => end.or(timeseries[0].last().map(|p| p.x)).map(|end| {
                if end >= timeseries[0].last().unwrap().x {
                    timeseries[0].last().unwrap().y
                } else {
                    let partition_point = timeseries[0].partition_point(|p| p.x < end);
                    timeseries[0][usize::min(partition_point, timeseries[0].len() - 1)].y
                }
            }),
            Data::Constant(constant) => Some(*constant),
        };
    }

    pub fn current_enum_value<E: TryFrom<u8>>(&self, metric: Metric, end: Option<f64>) -> Option<E>
    where
        E: TryFrom<u8>,
        <E as TryFrom<u8>>::Error: std::fmt::Debug,
    {
        let Some(enums) = self.enums.get(&metric) else {
            return None;
        };

        end.or(enums.last().map(|p| p.0))
            .map(|end| {
                if end >= enums.last().unwrap().0 {
                    enums.last().unwrap().1
                } else {
                    let partition_point = enums.partition_point(|p| p.0 < end);
                    enums[usize::min(partition_point, enums.len() - 1)].1
                }
            })
            .map(|i| i.try_into().unwrap())
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
        let mut new_start = visible_range.start() - length * 0.2;
        let mut new_end = visible_range.end() + length * 0.2;

        if let Some(end) = playback_end {
            new_end = f64::min(new_end, end);
            new_start = f64::min(new_start, end);
        }

        let visible_range = new_start..=new_end;

        let Some(data) = self.timeseries.get(key) else {
            return egui_plot::PlotPoints::Borrowed(&[]);
        };

        match data {
            Data::Timeseries(levels) => {
                // We start with our first cache level, the original data
                let mut vec = &levels[0];
                let mut start_i = vec.partition_point(|p| p.x < *visible_range.start());
                let mut end_i = vec.partition_point(|p| p.x < *visible_range.end());

                // As long as we have too many points, we go up in cache levels.
                for i in 1..DOWNSAMPLING_LEVELS {
                    if end_i - start_i < DOWNSAMPLING_THRESHOLD {
                        break;
                    }

                    vec = &levels[i];
                    start_i /= DOWNSAMPLING_FACTOR;
                    end_i /= DOWNSAMPLING_FACTOR;
                }

                // Extend by one in either direction so the line from the last point
                // onscreen to the next point offscreen is still drawn.
                let start_i = usize::max(start_i, 1) - 1;
                let end_i = usize::min(end_i + 1, usize::max(1, vec.len()) - 1);

                egui_plot::PlotPoints::Borrowed(&vec[start_i..end_i])
            }
            Data::Constant(value) => {
                egui_plot::PlotPoints::Owned(vec![PlotPoint::new(new_start, *value), PlotPoint::new(new_end, *value)])
            }
        }
    }

    pub fn enum_transitions<'a, E: TryFrom<u8>>(
        &'a self,
        key: &Metric,
        bounds: egui_plot::PlotBounds,
        playback_end: Option<f64>,
    ) -> Vec<(f64, E)> {
        // Since egui gives us the bounds from the last frame, we extend our
        // "visible" range slightly, to cover the edges while moving.
        let visible_range = bounds.range_x();
        let length = visible_range.end() - visible_range.start();
        let mut new_start = visible_range.start() - length * 0.2;
        let mut new_end = visible_range.end() + length * 0.2;

        if let Some(end) = playback_end {
            new_end = f64::min(new_end, end);
            new_start = f64::min(new_start, end);
        }

        let visible_range = new_start..=new_end;

        let Some(ints) = self.enums.get(key) else {
            return vec![];
        };

        let start_i = ints.partition_point(|p| p.0 < *visible_range.start());
        let end_i = ints.partition_point(|p| p.0 < *visible_range.end());
        let start_i = usize::max(start_i, 1) - 1;
        let end_i = usize::min(end_i + 1, usize::max(1, ints.len()) - 1);

        ints[start_i..=end_i]
            .into_iter()
            .scan(None, |state: &mut Option<u8>, &(t, i)| {
                let keep = state.map(|s| s != i).unwrap_or(true);
                state.replace(i);
                Some((t, i, keep))
            })
            .filter_map(|(t, i, keep)| keep.then_some((t, i)).and_then(|(t, i)| i.try_into().ok().map(|e| (t, e))))
            .collect()
    }

    pub fn timeseries<'a>(
        &'a self,
        metric: &Metric,
        end: Option<f64>,
    ) -> Option<impl Iterator<Item = (f64, f64)> + ExactSizeIterator + std::fmt::Debug + use<'a>> {
        let Some(vec) = self
            .timeseries
            .get(metric)
            .map(|data| match data {
                Data::Timeseries(levels) => Some(levels),
                Data::Constant(_) => None,
            })
            .flatten()
            .map(|levels| levels[0].iter().map(|pp| (pp.x, pp.y)).collect::<Vec<_>>())
            .or(self.enums.get(metric).map(|series| series.iter().map(|(x, y)| (*x, *y as f64)).collect::<Vec<_>>()))
        else {
            return None;
        };

        let limited: Vec<(f64, f64)> = end
            .or(vec.last().map(|p| p.0))
            .map(move |end| {
                let partition_point = vec.partition_point(|p| p.0 < end);
                vec[..usize::min(partition_point, vec.len() - 1)].into_iter().copied().collect()
            })
            .unwrap_or_default();

        Some(limited.into_iter())
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
        let ds = DataStoreFile {
            enums: self.enums.iter().map(|(k, v)| (*k, v.clone())).collect(),
            timeseries: self
                .timeseries
                .iter()
                .map(|(k, d)| match d {
                    Data::Timeseries(timeseries) => Some((k, timeseries)),
                    Data::Constant(_) => None,
                })
                .flatten()
                .map(|(k, v)| (*k, v[0].iter().map(|pp| (pp.x, pp.y)).collect()))
                .collect(),
        };

        std::fs::write(path, serde_json::to_string(&ds).unwrap())?;

        Ok(())
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let raw: DataStoreFile = serde_json::from_slice(bytes).unwrap();

        let timeseries: HashMap<Metric, Data> = raw
            .timeseries
            .into_iter()
            .map(|(k, v)| {
                let mut levels: [Vec<PlotPoint>; DOWNSAMPLING_LEVELS] = Default::default();
                levels[0] = v.into_iter().map(|(x, y)| PlotPoint::new(x, y)).collect();
                for i in 1..DOWNSAMPLING_LEVELS {
                    levels[i] = Self::downsample_points(&levels[i - 1]);
                }
                (k, Data::Timeseries(levels))
            })
            .collect();

        let first_time = timeseries
            .iter()
            .map(|(_k, d)| match d {
                Data::Timeseries(timeseries) => Some(timeseries),
                Data::Constant(_) => None,
            })
            .flatten()
            .filter_map(|v| v[0].first())
            .map(|pp| pp.x)
            .fold(f64::INFINITY, |a, b| f64::min(a, b));

        let last_time = timeseries
            .iter()
            .map(|(_k, d)| match d {
                Data::Timeseries(timeseries) => Some(timeseries),
                Data::Constant(_) => None,
            })
            .flatten()
            .filter_map(|v| v[0].last())
            .map(|pp| pp.x)
            .fold(f64::NEG_INFINITY, |a, b| f64::max(a, b));

        let ds = Self {
            enums: raw.enums.into_iter().collect(),
            timeseries,
            first_time: first_time.is_normal().then_some(first_time),
            last_time: last_time.is_normal().then_some(last_time),
        };

        ds
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
