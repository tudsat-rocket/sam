#[cfg(feature = "rev1")]
mod adxl375;
#[cfg(feature = "rev2")]
mod h3lis331dl;

#[cfg(feature = "rev1")]
pub use adxl375::ADXL375;
#[cfg(feature = "rev2")]
pub use h3lis331dl::H3LIS331DL;
