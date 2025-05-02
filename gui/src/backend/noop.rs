use crate::BackendVariant;
use telemetry::DataStore;

#[derive(Default)]
pub struct NoopBackend {
    data_store: DataStore,
}

impl BackendVariant for NoopBackend {
    fn update(&mut self, _ctx: &egui::Context) {}

    fn data_store<'a>(&'a self) -> &'a DataStore {
        &self.data_store
    }

    fn reset(&mut self) {}

    fn end(&self) -> Option<f64> {
        None
    }
}
