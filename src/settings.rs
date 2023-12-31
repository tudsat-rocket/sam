use std::fs::File;

use serde::{Deserialize, Serialize};

use mithril::settings::LoRaSettings;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AppSettings {
    pub mapbox_access_token: String,
    pub lora: LoRaSettings,
}

impl AppSettings {
    pub fn load() -> Result<Self, Box<dyn std::error::Error>> {
        #[cfg(not(target_os="android"))]
        let project_dirs = directories::ProjectDirs::from("space", "tudsat", "sam").unwrap();
        #[cfg(not(target_os="android"))]
        let config_dir = project_dirs.config_dir();

        #[cfg(target_os="android")]
        let config_dir = std::path::Path::new("/data/user/0/space.tudsat.sam/files");

        if !config_dir.exists() {
            std::fs::create_dir_all(config_dir)?;
        }

        let f = File::open(config_dir.join("config.json"))?;
        let config = serde_json::from_reader(f)?;
        Ok(config)
    }

    pub fn save(&self) -> Result<(), Box<dyn std::error::Error>> {
        #[cfg(not(target_os="android"))]
        let project_dirs = directories::ProjectDirs::from("space", "tudsat", "sam").unwrap();
        #[cfg(not(target_os="android"))]
        let config_dir = project_dirs.config_dir();

        #[cfg(target_os="android")]
        let config_dir = std::path::Path::new("/data/user/0/space.tudsat.sam/files");

        if !config_dir.exists() {
            std::fs::create_dir_all(config_dir)?;
        }

        let f = File::create(config_dir.join("config.json"))?;
        serde_json::to_writer_pretty(f, self)?;
        Ok(())
    }
}
