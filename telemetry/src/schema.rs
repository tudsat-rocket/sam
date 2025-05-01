use crate::*;

#[derive(Clone, Debug, PartialEq)]
pub struct MessageDefinition(pub &'static [(Metric, Representation)]);

#[derive(Clone, Debug, PartialEq)]
pub struct TelemetrySchema(&'static [(MessageDefinition, u32, u32)]);

impl MessageDefinition {
    pub fn collect<S: MetricSource, const N: usize>(
        &self,
        mut writer: TelemetryMessageWriter<N>,
        source: &mut S,
    ) -> heapless::Vec<u8, N> {
        for (metric, repr) in self.0 {
            source.write_metric(&mut writer, *metric, *repr).unwrap(); // TODO
        }

        writer.finish()
    }

    pub fn read<const N: usize, F: FnMut(Metric, Representation, &mut TelemetryMessageReader<N>)>(
        &self,
        mut reader: TelemetryMessageReader<N>,
        mut cb: F,
    ) {
        for (metric, repr) in self.0 {
            cb(*metric, *repr, &mut reader)
        }
    }
}

impl TelemetrySchema {
    pub const fn new(messages: &'static [(MessageDefinition, u32, u32)]) -> Self {
        Self(messages)
    }

    pub fn message<S: MetricSource, const N: usize>(&self, source: &mut S, time: u32) -> Option<heapless::Vec<u8, N>> {
        for (message, modulus, comp) in self.0 {
            if time % *modulus == *comp {
                let writer = TelemetryMessageWriter::default();
                return Some(message.collect(writer, source));
            }
        }

        None
    }

    pub fn receive<const N: usize, F: FnMut(Metric, Representation, &mut TelemetryMessageReader<N>)>(
        &self,
        time: u32,
        message: heapless::Vec<u8, N>,
        cb: F,
    ) -> Result<(), ()> {
        let reader = TelemetryMessageReader::new(message);
        for (message, modulus, comp) in self.0 {
            if time % *modulus == *comp {
                message.read(reader, cb);
                return Ok(());
            }
        }

        todo!()
    }
}
