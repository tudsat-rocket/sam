pub mod engine;
pub mod ereg;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum ConnectivityStatus {
    Online,
    Offline,
    OfflineAgain,
}

pub trait Subsystem {
    fn conectivity_status(&self) -> ConnectivityStatus;
}
