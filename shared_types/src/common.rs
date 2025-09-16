#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FlightMode {
    Idle = 0,
    HardwareArmed = 1,
    Armed = 2,
    ArmedLaunchImminent = 3,
    Burn = 4,
    Coast = 5,
    RecoveryDrogue = 6,
    RecoveryMain = 7,
    Landed = 8,
}

impl TryFrom<u8> for FlightMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Idle),
            1 => Ok(Self::HardwareArmed),
            2 => Ok(Self::Armed),
            3 => Ok(Self::ArmedLaunchImminent),
            4 => Ok(Self::Burn),
            5 => Ok(Self::Coast),
            6 => Ok(Self::RecoveryDrogue),
            7 => Ok(Self::RecoveryMain),
            8 => Ok(Self::Landed),
            _ => Err(()),
        }
    }
}

impl FlightMode {
    pub fn led_state(self, time: u32) -> (bool, bool, bool) {
        match self {
            FlightMode::Idle => (false, false, true),                          // ( ,  , G)
            FlightMode::HardwareArmed => (true, time % 500 < 250, false),      // (R, y,  )
            FlightMode::Armed => (true, true, false),                          // (R, Y,  )
            FlightMode::ArmedLaunchImminent => (time % 100 < 50, true, false), // (r, Y,  )
            FlightMode::Burn => (false, time % 200 < 100, false),              // ( , y,  )
            FlightMode::Coast => (false, true, false),                         // ( , Y,  )
            FlightMode::RecoveryDrogue => (false, true, true),                 // ( , Y, G)
            FlightMode::RecoveryMain => (true, false, true),                   // (R,  , G)
            FlightMode::Landed => (false, false, time % 1000 < 500),           // ( ,  , g)
        }
    }
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Idle
    }
}
