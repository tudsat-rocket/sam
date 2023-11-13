#[cfg(feature = "rev1")]
mod bmm150;
#[cfg(feature = "rev2")]
mod lis3mdl;

#[cfg(feature = "rev1")]
pub use bmm150::BMM150;
#[cfg(feature = "rev2")]
pub use lis3mdl::LIS3MDL;
