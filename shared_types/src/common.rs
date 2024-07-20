#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FlightMode {
    Idle = 0,
    HardwareArmed = 1,
    Armed = 2,
    Burn = 3,
    Coast = 4,
    RecoveryDrogue = 5,
    RecoveryMain = 6,
    Landed = 7,
}

impl TryFrom<u8> for FlightMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Idle),
            1 => Ok(Self::HardwareArmed),
            2 => Ok(Self::Armed),
            3 => Ok(Self::Burn),
            4 => Ok(Self::Coast),
            5 => Ok(Self::RecoveryDrogue),
            6 => Ok(Self::RecoveryMain),
            7 => Ok(Self::Landed),
            _ => Err(())
        }
    }
}

impl FlightMode {
    pub fn led_state(self, time: u32) -> (bool, bool, bool) {
        match self {
            FlightMode::Idle => (false, false, true),                       // ( ,  , G)
            FlightMode::HardwareArmed => (true, time % 500 < 250, false),   // (R, y,  )
            FlightMode::Armed => (true, true, false),                       // (R, Y,  )
            FlightMode::Burn => (false, time % 200 < 100, false),           // ( , y,  )
            FlightMode::Coast => (false, true, false),                      // ( , Y,  )
            FlightMode::RecoveryDrogue => (false, true, true),              // ( , Y, G)
            FlightMode::RecoveryMain => (true, false, true),                // (R,  , G)
            FlightMode::Landed => (false, false, time % 1000 < 500),        // ( ,  , g)
        }
    }
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Idle
    }
}
