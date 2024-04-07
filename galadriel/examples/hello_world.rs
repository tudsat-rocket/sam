use galadriel::*;

fn main() {
    let settings = SimulationSettings::default();
    let mut sim = Simulation::new(&settings);
    while sim.state.flight_phase != FlightPhase::Touchdown {
        sim.tick();
        println!("{:?}", sim.state);
    }
}
