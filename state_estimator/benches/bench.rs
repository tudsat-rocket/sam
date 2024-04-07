use std::num::Wrapping;

use criterion::{criterion_group, criterion_main, BatchSize, BenchmarkId, Criterion};

use nalgebra::Vector3;
use shared_types::{FlightMode, Settings};
use state_estimator::StateEstimator;

use FlightMode::*;

fn mocked_sensor_values() -> (
    Vector3<f32>,
    Vector3<f32>,
    Vector3<f32>,
    Vector3<f32>,
    f32
) {
    (
        Vector3::new(0.1, -0.5, 0.3),
        Vector3::new(0.6, -3.1, -14.0),
        Vector3::new(0.6, -3.1, -14.0),
        Vector3::new(50.0, -0.5, 0.1),
        123.4
    )
}

fn update(c: &mut Criterion) {
    let mut group = c.benchmark_group("update");
    for mode in [Armed, Burn, Coast, RecoveryDrogue, RecoveryMain] {
        group.bench_with_input(BenchmarkId::from_parameter(format!("{:?}", mode)), &mode, |b, mode| {
            let mut state_estimator = StateEstimator::new(1000.0, Settings::default());

            let t: u32 = 0;
            b.iter_batched(
                || mocked_sensor_values(),
                |(g, a1, a2, m, b)| {
                    state_estimator.update(Wrapping(t), *mode, Some(g), Some(a1), Some(a2), Some(m), Some(b), None)
                },
                BatchSize::PerIteration
            );
        });
    }
}

criterion_group!(benches, update);
criterion_main!(benches);
