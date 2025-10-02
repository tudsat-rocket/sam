//TODO Hans: to_string/to_float should be encapsulated in a DisplayableValue trait and interact for plotting
pub trait StorableValue {
    ///Convert the value to a 64bit representation. Must be the inverse of from_bits
    fn to_bits(&self) -> u64;
    ///Get the value from a 64bit representation. Must be the inverse of to_bits
    fn from_bits(bits: u64) -> Self;
    ///Get the current value as a string for displaying
    fn to_string(&self) -> String;
    ///Get the current value as a float for plotting
    fn to_float(&self) -> f64;
}

impl StorableValue for f64 {
    fn to_bits(&self) -> u64 {
        return f64::to_bits(*self);
    }

    fn from_bits(bits: u64) -> Self {
        return f64::from_bits(bits);
    }

    fn to_string(&self) -> String {
        return format!("{self}");
    }

    fn to_float(&self) -> f64 {
        return *self;
    }
}

impl StorableValue for u8 {
    fn to_bits(&self) -> u64 {
        return *self as u64;
    }

    fn from_bits(bits: u64) -> Self {
        return bits as Self;
    }

    fn to_string(&self) -> String {
        return format!("{self}");
    }

    fn to_float(&self) -> f64 {
        return *self as f64;
    }
}

#[derive(Clone, PartialEq, PartialOrd)]
pub enum ValveState {
    // Connected(Option<FluidType>),
    Open,
    Closed,
}

impl StorableValue for ValveState {
    fn to_bits(&self) -> u64 {
        match self {
            ValveState::Open => return 1,
            ValveState::Closed => return 0,
        }
    }
    fn from_bits(bits: u64) -> Self {
        if bits == 1 {
            return ValveState::Open;
        } else if bits == 0 {
            return ValveState::Closed;
        } else {
            panic!("Cannot convert {bits} to ValveState!")
        }
    }

    fn to_string(&self) -> String {
        match self {
            ValveState::Open => "Open".to_string(),
            ValveState::Closed => "Closed".to_string(),
        }
    }

    fn to_float(&self) -> f64 {
        match self {
            ValveState::Open => return 1f64,
            ValveState::Closed => return 0f64,
        }
    }
}

impl StorableValue for shared_types::telemetry::FlightMode {
    fn to_bits(&self) -> u64 {
        return (*self as u8).to_bits();
    }

    fn from_bits(bits: u64) -> Self {
        match bits {
            0 => Self::Idle,
            1 => Self::HardwareArmed,
            2 => Self::Armed,
            3 => Self::ArmedLaunchImminent,
            4 => Self::Burn,
            5 => Self::Coast,
            6 => Self::RecoveryDrogue,
            7 => Self::RecoveryMain,
            8 => Self::Landed,
            _ => panic!("Cannot convert {bits} to FlightMode!"),
        }
    }

    fn to_string(&self) -> String {
        return format!("{:?}", self);
    }

    fn to_float(&self) -> f64 {
        return (*self as u8) as f64;
    }
}

impl StorableValue for shared_types::telemetry::TransmitPower {
    fn to_bits(&self) -> u64 {
        return match self {
            shared_types::TransmitPower::P14dBm => 0x00,
            shared_types::TransmitPower::P17dBm => 0x01,
            shared_types::TransmitPower::P20dBm => 0x02,
            shared_types::TransmitPower::P22dBm => 0x03,
        };
    }

    fn from_bits(bits: u64) -> Self {
        match bits {
            0x00 => shared_types::TransmitPower::P14dBm,
            0x01 => shared_types::TransmitPower::P17dBm,
            0x02 => shared_types::TransmitPower::P20dBm,
            0x03 => shared_types::TransmitPower::P22dBm,
            _ => shared_types::TransmitPower::default(),
        }
    }

    fn to_string(&self) -> String {
        return format!("{:?}", self);
    }

    fn to_float(&self) -> f64 {
        return match self {
            shared_types::TransmitPower::P14dBm => 14f64,
            shared_types::TransmitPower::P17dBm => 17f64,
            shared_types::TransmitPower::P20dBm => 20f64,
            shared_types::TransmitPower::P22dBm => 22f64,
        };
    }
}

impl StorableValue for shared_types::telemetry::AcsMode {
    fn to_bits(&self) -> u64 {
        return match self {
            shared_types::AcsMode::Disabled => 0b00,
            shared_types::AcsMode::Auto => 0b01,
            shared_types::AcsMode::Manual => 0b10,
        };
    }

    fn from_bits(bits: u64) -> Self {
        match bits {
            0b00 => Self::Disabled,
            0b01 => Self::Auto,
            0b10 => Self::Manual,
            _ => panic!("Cannot convert {bits} to AcsMode!"),
        }
    }

    fn to_string(&self) -> String {
        return format!("{:?}", self);
    }

    fn to_float(&self) -> f64 {
        return (*self as u8) as f64;
    }
}

impl StorableValue for shared_types::telemetry::ThrusterValveState {
    fn to_bits(&self) -> u64 {
        return match self {
            shared_types::ThrusterValveState::Closed => 0b00,
            shared_types::ThrusterValveState::OpenAccel => 0b10,
            shared_types::ThrusterValveState::OpenDecel => 0b01,
            shared_types::ThrusterValveState::OpenBoth => 0b11,
        };
    }

    fn from_bits(bits: u64) -> Self {
        match bits {
            0b00 => Self::Closed,
            0b10 => Self::OpenAccel,
            0b01 => Self::OpenDecel,
            0b11 => Self::OpenBoth,
            _ => panic!("Cannot convert {bits} to ThrusterValveState!"),
        }
    }

    fn to_string(&self) -> String {
        return format!("{:?}", self);
    }

    fn to_float(&self) -> f64 {
        return match self {
            Self::OpenAccel => 1.0,
            Self::OpenDecel => -1.0,
            _ => 0.0,
        };
    }
}

impl StorableValue for shared_types::telemetry::BatteryChargerState {
    fn to_bits(&self) -> u64 {
        return match self {
            shared_types::BatteryChargerState::NotCharging => 0b00,
            shared_types::BatteryChargerState::Charging => 0b01,
            shared_types::BatteryChargerState::RecoverableFault => 0b10,
            shared_types::BatteryChargerState::NonRecoverableFault => 0b11,
        };
    }

    fn from_bits(bits: u64) -> Self {
        match bits {
            0b00 => Self::NotCharging,
            0b01 => Self::Charging,
            0b10 => Self::RecoverableFault,
            0b11 => Self::NonRecoverableFault,
            _ => panic!("Cannot convert {bits} to BatteryChargerState!"),
        }
    }

    fn to_string(&self) -> String {
        return format!("{:?}", self);
    }

    fn to_float(&self) -> f64 {
        return (*self as u8) as f64;
    }
}

impl StorableValue for shared_types::telemetry::GPSFixType {
    fn to_bits(&self) -> u64 {
        return *self as u64;
    }

    fn from_bits(bits: u64) -> Self {
        match bits {
            0 => Self::NoFix,
            1 => Self::AutonomousFix,
            2 => Self::DifferentialFix,
            3 => Self::RTKFix,
            4 => Self::RTKFloat,
            5 => Self::DeadReckoningFix,
            _ => Self::NoFix,
        }
    }

    fn to_string(&self) -> String {
        return format!("{:?}", self);
    }

    fn to_float(&self) -> f64 {
        return (*self as u8) as f64;
    }
}

impl TryFrom<u8> for ValveState {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            core::u8::MAX => Ok(Self::Open),
            core::u8::MIN => Ok(Self::Closed),
            _ => Err(()),
        }
    }
}
