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

pub const ARCHIVED_LOGS: [ArchivedLog; 8] = [
    ArchivedLog {
        id: "zuelpich22-1",
        fc_serial: "1A",
        vehicle: "FRoDO-H",
        site: ZUELPICH,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2022, 12, 10),
        name: "Launch #1",
        description: "",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/zuelpich_2022_launch1_telem.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/zuelpich_2022_launch1_flash.json"),
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
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/zuelpich_2022_launch2_telem.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/zuelpich_2022_launch2_flash.json"),
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
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/dare_2023_primary_telem.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/dare_2023_primary_flash.json"),
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
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/dare_2023_secondary_telem.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/dare_2023_secondary_flash.json"),
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
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/euroc_2023_telemetry.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/euroc_2023_flash.json"),
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
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/zuelpich_2024_telem.json"),
        flash_log_url: None,
        apogee_agl: 246.5,
    },
    ArchivedLog {
        id: "euroc24-primary",
        fc_serial: "2B",
        vehicle: "FRoDO-M",
        site: EUROC,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2024, 10, 12),
        name: "Primary",
        description: "",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/euroc_2024_primary_telem.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/euroc_2024_primary_flash.json"),
        apogee_agl: 3049.6,
    },
    ArchivedLog {
        id: "euroc24-secondary",
        fc_serial: "2D",
        vehicle: "FRoDO-M",
        site: EUROC,
        #[allow(deprecated)] // the replacement is not const yet
        date: chrono::NaiveDate::from_ymd(2024, 10, 12),
        name: "Secondary",
        description: "",
        telemetry_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/euroc_2024_secondary_telem.json"),
        flash_log_url: Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/flight_data/euroc_2024_secondary_flash.json"),
        apogee_agl: 3052.0,
    },
];

impl ArchivedLog {
    pub fn find(id: &str) -> Option<Self> {
        ARCHIVED_LOGS.into_iter().find(|log| log.id == id)
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
