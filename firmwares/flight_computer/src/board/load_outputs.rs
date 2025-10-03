use embassy_stm32::gpio::{AnyPin, Input, Level, Output, OutputType, Pin, Pull, Speed};
pub struct LoadOutputs {
    arm: Output<'static>,
    out1: Output<'static>,
    out2: Output<'static>,
    out3: Output<'static>,
    out4: Output<'static>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LoadOutput {
    Out1,
    Out2,
    Out3,
    Out4,
}

impl LoadOutputs {
    pub fn new(
        arm: Output<'static>,
        out1: Output<'static>,
        out2: Output<'static>,
        out3: Output<'static>,
        out4: Output<'static>,
    ) -> Self {
        Self {
            arm,
            out1,
            out2,
            out3,
            out4,
        }
    }
    pub fn arm(&mut self) {
        self.arm.set_high();
    }
    pub fn disarm(&mut self) {
        self.arm.set_high();
        self.out1.set_high();
        self.out2.set_high();
        self.out3.set_high();
        self.out4.set_high();
    }
    pub fn is_armed(&mut self) -> bool {
        self.arm.is_set_high()
    }
    pub fn activate(&mut self, output: LoadOutput) {
        if !self.is_armed() {
            return;
        }
        match output {
            LoadOutput::Out1 => self.out1.set_high(),
            LoadOutput::Out2 => self.out2.set_high(),
            LoadOutput::Out3 => self.out3.set_high(),
            LoadOutput::Out4 => self.out4.set_high(),
        }
    }
    pub fn deactivate(&mut self, output: LoadOutput) {
        match output {
            LoadOutput::Out1 => self.out1.set_low(),
            LoadOutput::Out2 => self.out2.set_low(),
            LoadOutput::Out3 => self.out3.set_low(),
            LoadOutput::Out4 => self.out4.set_low(),
        }
    }
    pub fn is_active(&mut self, output: LoadOutput) -> bool {
        match output {
            LoadOutput::Out1 => self.out1.is_set_high(),
            LoadOutput::Out2 => self.out2.is_set_high(),
            LoadOutput::Out3 => self.out3.is_set_high(),
            LoadOutput::Out4 => self.out4.is_set_high(),
        }
    }
}
