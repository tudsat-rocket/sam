use core::ops::RangeInclusive;

use crate::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Representation {
    Enum { bits: usize },
    FixedPoint { bits: usize, min: f64, max: f64 },
    FloatingPoint { bits: usize },
    // TODO: largefixpoint?
}

/// Represents a partially written telemetry messages. Components implementing [MetricSource]
/// are able to write new data into it similar to the way implementors of [std::fmt::Format]
/// write strings to a [std::fmt::Formatter].
#[derive(Debug, Default)]
pub struct TelemetryMessageWriter<const N: usize> {
    bit_pointer: usize,
    buffer: heapless::Vec<u8, N>,
}

#[derive(Debug)]
pub struct TelemetryMessageReader<const N: usize> {
    bit_pointer: usize,
    buffer: heapless::Vec<u8, N>,
}

impl Representation {
    pub fn bits(&self) -> usize {
        match self {
            Self::Enum { bits } => *bits,
            Self::FixedPoint { bits, .. } => *bits,
            Self::FloatingPoint { bits } => *bits,
        }
    }

    pub const fn float(bits: usize) -> Self {
        Self::FloatingPoint { bits }
    }

    pub const fn fixed(bits: usize, range: RangeInclusive<f64>) -> Self {
        Self::FixedPoint {
            bits,
            min: *range.start(),
            max: *range.end(),
        }
    }
}

impl<const N: usize> TelemetryMessageWriter<N> {
    pub fn write_enum(&mut self, repr: Representation, value: u8) -> Result<(), Infallible> {
        let bits = repr.bits();
        assert_eq!(repr, Representation::Enum { bits });
        assert_eq!(self.bit_pointer / 8, self.buffer.len());

        self.buffer.push(value).unwrap();
        self.bit_pointer += bits;

        Ok(())
    }

    pub fn write_float<F: Into<f64>>(&mut self, repr: Representation, value: F) -> Result<(), Infallible> {
        assert_eq!(self.bit_pointer / 8, self.buffer.len());

        let bits = repr.bits();

        let f = value.into();
        let int = match repr {
            Representation::FixedPoint { bits, min, max } => {
                if f >= max {
                    u64::MAX
                } else if f <= min {
                    u64::MIN
                } else {
                    (((f - min) / (max - min)) * (((1u64 << bits) - 1) as f64)) as u64
                }
            }
            Representation::FloatingPoint { bits } => match bits {
                64 => f.to_bits(),
                32 => (f as f32).to_bits() as u64,
                16 => half::f16::from_f64(f).to_bits() as u64,
                _ => panic!("non-{{16,32,64}}-bit floating point numbers not supported"),
            },
            Representation::Enum { bits: _ } => unreachable!(),
        };

        let mut len = bits / 8;
        if bits % 8 != 0 {
            len += 1;
        }

        let bytes = int.to_be_bytes();
        for byte in &bytes[(8 - len)..] {
            self.buffer.push(*byte).unwrap(); // TODO
        }

        self.bit_pointer += bits;

        Ok(())
    }

    pub fn write_vector(
        &mut self,
        repr: Representation,
        dim: Dim,
        value: &nalgebra::Vector3<f32>,
    ) -> Result<(), Infallible> {
        let i = dim as usize;
        self.write_float(repr, value[i])
    }

    pub fn finish(self) -> heapless::Vec<u8, N> {
        self.buffer
    }
}

impl<const N: usize> TelemetryMessageReader<N> {
    pub fn new(message: heapless::Vec<u8, N>) -> Self {
        Self {
            bit_pointer: 0,
            buffer: message,
        }
    }

    pub fn read_enum<E: TryFrom<u8>>(&mut self, _repr: Representation) -> Result<E, ()> {
        let b = self.buffer[self.bit_pointer / 8];
        self.bit_pointer += 8;

        let e = b.try_into().ok().unwrap();
        Ok(e)
    }

    pub fn read_float(&mut self, repr: Representation) -> Result<f64, ()> {
        if self.bit_pointer % 8 != 0 {
            self.bit_pointer += 8 - self.bit_pointer % 8;
        }

        assert_eq!(repr.bits() % 8, 0);

        let bytes = &self.buffer[(self.bit_pointer / 8)..((self.bit_pointer + repr.bits()) / 8)];
        let value = match repr {
            Representation::FixedPoint { bits, min, max } => {
                let mut be: [u8; 8] = [0; 8];
                be[(8 - bytes.len())..].copy_from_slice(&bytes);
                min + ((u64::from_be_bytes(be) as f64) / (((1u64 << bits) - 1) as f64)) * (max - min)
            }
            Representation::FloatingPoint { bits } => match bits {
                64 => f64::from_be_bytes(bytes.try_into().unwrap()),
                32 => f32::from_be_bytes(bytes.try_into().unwrap()).into(),
                16 => half::f16::from_be_bytes(bytes.try_into().unwrap()).to_f64(),
                _ => todo!(),
            },
            Representation::Enum { bits: _ } => unreachable!(),
        };

        self.bit_pointer += repr.bits();

        Ok(value)
    }
}

#[cfg(test)]
mod tests {
    use crate::{Representation, TelemetryMessageReader, TelemetryMessageWriter};

    #[test]
    fn should_handle_byte_aligned_fixed_point_numbers() {
        let mut w = TelemetryMessageWriter::<16>::default();
        let r1 = Representation::FixedPoint {
            bits: 8,
            min: 0.0,
            max: 1000.0,
        };
        let r2 = Representation::FixedPoint {
            bits: 8,
            min: -1000.0,
            max: 1000.0,
        };
        let r3 = Representation::FixedPoint {
            bits: 16,
            min: -1000.0,
            max: 1000.0,
        };
        w.write_float(r1, 0.0).unwrap();
        w.write_float(r1, 999.0).unwrap();
        w.write_float(r1, 1000.0).unwrap();
        w.write_float(r2, -1000.0).unwrap();
        w.write_float(r2, 0.0).unwrap();
        w.write_float(r2, 1000.0).unwrap();
        w.write_float(r3, -1000.0).unwrap();
        w.write_float(r3, 0.0).unwrap();
        w.write_float(r3, 990.0).unwrap();

        let buffer = w.finish();
        assert_eq!(buffer, [0x00, 0xfe, 0xff, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0xb7]);

        let mut r = TelemetryMessageReader::new(buffer);
        assert_eq!(r.read_float(r1).unwrap(), 0.0);
        assert_eq!(r.read_float(r1).unwrap(), 996.0784313725491);
        assert_eq!(r.read_float(r1).unwrap(), 1000.0);
        assert_eq!(r.read_float(r2).unwrap(), -1000.0);
        assert_eq!(r.read_float(r2).unwrap(), -3.921568627450938);
        assert_eq!(r.read_float(r2).unwrap(), 1000.0);
        assert_eq!(r.read_float(r3).unwrap(), -1000.0);
        assert_eq!(r.read_float(r3).unwrap(), -0.015259021896667946);
        assert_eq!(r.read_float(r3).unwrap(), 989.9900816357672);
    }

    #[test]
    fn should_handle_non_byte_aligned_fixed_point_numbers() {
        // TODO
    }

    #[test]
    fn should_handle_offscale_fixed_point_numbers() {
        let mut w = TelemetryMessageWriter::<16>::default();
        let r1 = Representation::FixedPoint {
            bits: 8,
            min: 0.0,
            max: 1000.0,
        };
        let r2 = Representation::FixedPoint {
            bits: 16,
            min: -1000.0,
            max: 1000.0,
        };
        w.write_float(r1, -1000.0).unwrap();
        w.write_float(r1, 2000.0).unwrap();
        w.write_float(r2, -1002.0).unwrap();
        w.write_float(r2, 1002.0).unwrap();

        let buffer = w.finish();
        assert_eq!(buffer, [0x00, 0xff, 0x00, 0x00, 0xff, 0xff]);

        let mut r = TelemetryMessageReader::new(buffer);
        assert_eq!(r.read_float(r1).unwrap(), 0.0);
        assert_eq!(r.read_float(r1).unwrap(), 1000.0);
        assert_eq!(r.read_float(r2).unwrap(), -1000.0);
        assert_eq!(r.read_float(r2).unwrap(), 1000.0);
    }

    #[test]
    fn should_handle_byte_aligned_floating_point_numbers() {
        let mut w = TelemetryMessageWriter::<64>::default();
        let r1 = Representation::FloatingPoint { bits: 64 };
        let r2 = Representation::FloatingPoint { bits: 32 };
        let r3 = Representation::FloatingPoint { bits: 16 };
        w.write_float(r1, -1000.0).unwrap();
        w.write_float(r1, 1.0).unwrap();
        w.write_float(r1, 1000.0).unwrap();
        w.write_float(r2, -1000.0).unwrap();
        w.write_float(r2, 1.0).unwrap();
        w.write_float(r2, 1000.0).unwrap();
        w.write_float(r3, -1000.0).unwrap();
        w.write_float(r3, 1.0).unwrap();
        w.write_float(r3, 1000.0).unwrap();

        let buffer = w.finish();
        assert_eq!(buffer.len(), 3 * 2 + 3 * 4 + 3 * 8);

        let mut r = TelemetryMessageReader::new(buffer);
        assert_eq!(r.read_float(r1).unwrap(), -1000.0);
        assert_eq!(r.read_float(r1).unwrap(), 1.0);
        assert_eq!(r.read_float(r1).unwrap(), 1000.0);
        assert_eq!(r.read_float(r2).unwrap(), -1000.0);
        assert_eq!(r.read_float(r2).unwrap(), 1.0);
        assert_eq!(r.read_float(r2).unwrap(), 1000.0);
        assert_eq!(r.read_float(r3).unwrap(), -1000.0);
        assert_eq!(r.read_float(r3).unwrap(), 1.0);
        assert_eq!(r.read_float(r3).unwrap(), 1000.0);
    }

    #[test]
    fn should_handle_special_floating_point_values() {
        let mut w = TelemetryMessageWriter::<64>::default();
        let r1 = Representation::FloatingPoint { bits: 64 };
        let r2 = Representation::FloatingPoint { bits: 32 };
        let r3 = Representation::FloatingPoint { bits: 16 };
        w.write_float(r1, f64::INFINITY).unwrap();
        w.write_float(r1, f64::NEG_INFINITY).unwrap();
        w.write_float(r1, f64::NAN).unwrap();
        w.write_float(r2, f64::INFINITY).unwrap();
        w.write_float(r2, f64::NEG_INFINITY).unwrap();
        w.write_float(r2, f64::NAN).unwrap();
        w.write_float(r3, f64::INFINITY).unwrap();
        w.write_float(r3, f64::NEG_INFINITY).unwrap();
        w.write_float(r3, f64::NAN).unwrap();

        let buffer = w.finish();
        assert_eq!(buffer.len(), 3 * 2 + 3 * 4 + 3 * 8);

        let mut r = TelemetryMessageReader::new(buffer);
        assert_eq!(r.read_float(r1).unwrap(), f64::INFINITY);
        assert_eq!(r.read_float(r1).unwrap(), f64::NEG_INFINITY);
        assert!(r.read_float(r1).unwrap().is_nan());
        assert_eq!(r.read_float(r2).unwrap(), f64::INFINITY);
        assert_eq!(r.read_float(r2).unwrap(), f64::NEG_INFINITY);
        assert!(r.read_float(r2).unwrap().is_nan());
        assert_eq!(r.read_float(r3).unwrap(), f64::INFINITY);
        assert_eq!(r.read_float(r3).unwrap(), f64::NEG_INFINITY);
        assert!(r.read_float(r3).unwrap().is_nan());
    }

    #[test]
    fn should_handle_enums() {}

    #[test]
    fn should_handle_mixed_data() {}
}
