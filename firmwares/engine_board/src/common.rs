#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub enum EngineMode {
    Disarmed = 0;
    PreIgnition = 1;
    Ignition = 2;
    Combustion = 3;
    EngineOut = 4;
}

impl TryFrom<u8> for EngineMode {
    type Error = ();
    
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Disarmed),
            1 => Ok(Self::PreIgnition),
            2 => Ok(Self::Ignition),
            3 => Ok(Self::Combustion),
            4 => Ok(Self::EngineOut),
            _ => Err(()),
        }
    }
}

