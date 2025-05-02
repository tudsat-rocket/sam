use criterion::{criterion_group, criterion_main, Criterion};

use galadriel::*;

fn tick(c: &mut Criterion) {
    c.bench_function("tick", |b| {
        let mut sim = Simulation::new(&SimulationSettings::default());

        while sim.state.flight_phase != FlightPhase::Burn {
            sim.tick();
        }

        let state = sim.state.clone();
        b.iter(|| {
            sim.tick();
            sim.state = state.clone();
        })
    });
}

fn complete_sim_100hz(c: &mut Criterion) {
    c.bench_function("complete_sim_1000hz", |b| {
        b.iter(|| {
            let mut sim = Simulation::new(&SimulationSettings::default());
            while sim.state.flight_phase != FlightPhase::Touchdown {
                sim.tick();
            }
        })
    });
}

criterion_group!(benches, tick, complete_sim_100hz);
criterion_main!(benches);
