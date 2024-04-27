use shared_types::{DownlinkMessage, VehicleState, Settings};

#[derive(Debug, Clone, PartialEq)]
pub struct LaunchSite {
    pub name: &'static str,
    pub latitude: f64,
    pub longitude: f64,
    pub azimuth: Option<f64>,
    pub elevation: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ArchivedLog {
    /// For selecting the log from the command line
    pub id: &'static str,
    /// Which flight computer was it?
    pub fc_serial: &'static str,
    /// Launch vehicle
    pub vehicle: &'static str,
    /// Launch site
    pub site: LaunchSite,
    /// Launch date
    pub date: chrono::NaiveDate,
    /// Additional launch name
    pub name: &'static str,
    /// Notes, known issues, etc.
    pub description: &'static str,
    pub telemetry_log_url: Option<&'static str>,
    pub flash_log_url: Option<&'static str>,
    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
    pub flash_log_bytes: Option<&'static [u8]>,
    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
    pub fc_settings_bytes: Option<&'static [u8]>,
    pub apogee_agl: f32,
}

const ZUELPICH: LaunchSite = LaunchSite {
    name: "LSC Zülpich",
    latitude: 0.0, // TODO
    longitude: 0.0,
    azimuth: None,
    elevation: None,
};

const DARE_ASK: LaunchSite = LaunchSite {
    name: "ASK t' Harde",
    latitude: 0.0, // TODO
    longitude: 0.0,
    azimuth: None,
    elevation: None,
};

const EUROC: LaunchSite = LaunchSite {
    name: "EuRoC",
    latitude: 0.0, // TODO
    longitude: 0.0,
    azimuth: None,
    elevation: None,
};

pub const ARCHIVED_LOGS: [ArchivedLog; 6] = [
    ArchivedLog {
        id: "zuelpich22-1",
        fc_serial: "1A",
        vehicle: "FRoDO-H",
        site: ZUELPICH,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2022, 12, 10),
        name: "Launch #1",
        description: "",
        telemetry_log_url: None, // TODO: migrate
        flash_log_url: None, // TODO: migrate
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        flash_log_bytes: None,
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        fc_settings_bytes: None,
        apogee_agl: 395.9,
    },
    ArchivedLog {
        id: "zuelpich22-2",
        fc_serial: "1A",
        vehicle: "FRoDO-H",
        site: ZUELPICH,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2022, 12, 10),
        name: "Launch #2",
        description: "",
        telemetry_log_url: None, // TODO: migrate
        flash_log_url: None, // TODO: migrate
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        flash_log_bytes: None,
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        fc_settings_bytes: None,
        apogee_agl: 361.8,
    },
    ArchivedLog {
        id: "dare23-a",
        fc_serial: "2A",
        vehicle: "FRoDO-J",
        site: DARE_ASK,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2023, 9, 22),
        name: "Primary FC",
        description: "",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/dare_launch_a_telem_filtered.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/dare_launch_a_flash_filtered.json"),
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        flash_log_bytes: Some(include_bytes!("../assets/dare_launch_a_flash_filtered.json").as_slice()),
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        fc_settings_bytes: Some(include_bytes!("../assets/dare_launch_a_settings.json").as_slice()),
        apogee_agl: 1130.4,
    },
    ArchivedLog {
        id: "dare23-b",
        fc_serial: "2D",
        vehicle: "FRoDO-J",
        site: DARE_ASK,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2023, 9, 22),
        name: "Secondary FC",
        description: "",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/dare_launch_b_telem_filtered.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/dare_launch_b_flash_filtered.json"),
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        flash_log_bytes: Some(include_bytes!("../assets/dare_launch_b_flash_filtered.json").as_slice()),
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        fc_settings_bytes: Some(include_bytes!("../assets/dare_launch_b_settings.json").as_slice()),
        apogee_agl: 1130.4,
    },
    ArchivedLog {
        id: "euroc23-aesir",
        fc_serial: "2B",
        vehicle: "ÆSIR Signý",
        site: EUROC,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2023, 10, 13),
        name: "Payload",
        description: "",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/euroc_2023_telem_filtered.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/euroc_2023_flash_filtered.json"),
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        flash_log_bytes: Some(include_bytes!("../assets/euroc_2023_flash_filtered.json").as_slice()),
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        fc_settings_bytes: Some(include_bytes!("../assets/euroc_2023_settings.json").as_slice()),
        apogee_agl: 3474.8,
    },
    ArchivedLog {
        id: "zuelpich24",
        fc_serial: "2C",
        vehicle: "FRoDO-H-GOSA",
        site: ZUELPICH,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2024, 4, 21),
        name: "",
        description: "significant barometer spikes",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/develop/archive/assets/zuelpich_gosa_telem.json"),
        flash_log_url: None,
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        flash_log_bytes: None,
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        fc_settings_bytes: Some(include_bytes!("../assets/zuelpich_gosa_settings.json").as_slice()),
        apogee_agl: 246.5,
    },
];

impl ArchivedLog {
    pub fn find(id: &str) -> Option<Self> {
        ARCHIVED_LOGS.into_iter().find(|log| log.id == id)
    }

    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
    pub fn flash_messages(&self) -> Option<Vec<DownlinkMessage>> {
        self.flash_log_bytes.map(|bytes| serde_json::from_slice::<Vec<DownlinkMessage>>(bytes).unwrap())
    }

    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
    pub fn flash_states(&self) -> Option<Vec<VehicleState>> {
        self.flash_messages().map(|vec| vec.into_iter().map(|msg| Into::<VehicleState>::into(msg)).collect())
    }

    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
    pub fn fc_settings(&self) -> Option<Settings> {
        self.fc_settings_bytes.map(|bytes| serde_json::from_slice::<Settings>(bytes).unwrap())
    }
}

impl ToString for ArchivedLog {
    fn to_string(&self) -> String {
        if self.name.is_empty() {
            format!("{}", self.vehicle)
        } else {
            format!("{} ({})", self.vehicle, self.name)
        }
    }
}
