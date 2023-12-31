mod configure;
mod launch;
mod plot;

pub use configure::*;
pub use launch::*;
pub use plot::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GuiTab {
    Launch,
    Plot,
    Configure,
}
