use crate::*;

pub trait MetricSource {
    type Error: core::error::Error;

    fn write_metric<const N: usize>(
        &mut self, // TODO: try to reduce this to non-mutable reference
        w: &mut TelemetryMessageWriter<N>,
        metric: Metric,
        repr: Representation,
    ) -> Result<(), Self::Error>;
}
