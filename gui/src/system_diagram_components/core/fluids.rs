use egui::Color32;

macro_rules! define_fluids {
    (
        $(
            $key:ident => ($name:expr, $color:expr)
        ),* $(,)?
    ) => {
        #[derive(Clone)]
        pub enum FluidType {
            $( $key ),*
        }

        pub fn get_fluid_color(key: &FluidType) -> Color32 {
            match key {
                $( FluidType::$key => $color),*
            }
        }

        pub fn get_fluid_name(key: &FluidType) -> &'static str {
            match key {
                $( FluidType::$key => $name),*
            }
        }
    }
}

define_fluids! {
    N2 => ( "N\u{2082}", Color32::LIGHT_GREEN),
    N2O => ("N\u{2082}O", Color32::LIGHT_BLUE),
    ETHANOL => ("Ethanol", Color32::RED),
    CompressedAir => ("Compressed Air", Color32::from_rgb(0x45, 0x85, 0x88)),
    UNKNOWN => ("Unknown", Color32::LIGHT_GRAY),
}
