// trait MetricConsumer {
//     type MetricRepresentation;
// }

use crate::new_safe_metrics::metric_consumption::{ConsumableMetric, Lens};

struct NewDataStore {
    values: HashMap<Metric, Vec<bool>>,
}

// impl MetricConsumer for NewDataStore {
//     type MetricRepresentation = Vec<bool>;
// }

//Add additional layer (ConsumableRepresentation) and add it as a generic to the MetricConsumer
trait ConsumableMetricRepresentation {}
impl ConsumableMetricRepresentation for f32 {}
impl ConsumableMetricRepresentation for u8 {}
// impl ConsumableMetricRepresentation for ! {}



struct MetricA {}
struct MetricB {}


impl ConsumableMetric<NewDataStore, f32, uom::si::length::kilometer, _> for MetricA {
    type Conversion = Lens::new::<f32>(1f32);
}