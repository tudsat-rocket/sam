pub mod configure;
pub mod plot;

pub use configure::*;
pub use plot::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GuiTab {
    Launch,
    Plot,
    Configure,
}
