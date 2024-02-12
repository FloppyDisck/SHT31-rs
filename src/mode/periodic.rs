use crate::{
    error::Result,
    mode::{Sht31Measure, Sht31Reader},
    Accuracy, Reading, SHT31,
};
use embedded_hal::i2c::I2c;

/// Periodic reading where reading returns the last available data
#[derive(Default, Copy, Clone, Debug)]
pub struct Periodic {
    mps: MPS,
    art: bool,
}

/// Stands for measurements per second
#[allow(dead_code)]
#[derive(Default, Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum MPS {
    Half = 0x20,
    #[default]
    Normal = 0x21,
    Double = 0x22,
    X4 = 0x23,
    X10 = 0x27,
}

impl Periodic {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            mps: MPS::Normal,
            art: false,
        }
    }

    /// Sets the measurements per second for the periodic readings,
    /// NOTE: The higher the frequency the less accurate the readings will be
    pub fn set_mps(&mut self, mps: MPS) {
        self.mps = mps;
    }

    /// Sets the measurements per second for the periodic readings,
    /// NOTE: The higher the frequency the less accurate the readings will be
    pub fn with_mps(mut self, mps: MPS) -> Self {
        self.set_mps(mps);
        self
    }

    /// Enables accelerated response time, evaluates data at a frequency of 4 Hz
    pub fn with_art(mut self) -> Self {
        self.art = true;
        self
    }
}

impl<I2C> Sht31Reader for SHT31<Periodic, I2C>
where
    I2C: I2c,
{
    fn read(&mut self) -> Result<Reading> {
        let mut buffer = [0; 6];

        self.i2c_read(&[0xE0, 0x00], &mut buffer)?;
        self.process_data(buffer)
    }
}

impl<I2C> Sht31Measure for SHT31<Periodic, I2C>
where
    I2C: I2c,
{
    /// Initialized the periodic measuring mode,
    /// a break command must be run in order to change
    /// the measuring style
    fn measure(&mut self) -> Result<()> {
        let (msb, lsb) = if self.mode.art {
            (0x2B, 0x32)
        } else {
            let lsb = match self.mode.mps {
                MPS::Half => match self.accuracy {
                    Accuracy::High => 0x32,
                    Accuracy::Medium => 0x24,
                    Accuracy::Low => 0x2F,
                },
                MPS::Normal => match self.accuracy {
                    Accuracy::High => 0x30,
                    Accuracy::Medium => 0x26,
                    Accuracy::Low => 0x2D,
                },
                MPS::Double => match self.accuracy {
                    Accuracy::High => 0x36,
                    Accuracy::Medium => 0x20,
                    Accuracy::Low => 0x2B,
                },
                MPS::X4 => match self.accuracy {
                    Accuracy::High => 0x34,
                    Accuracy::Medium => 0x22,
                    Accuracy::Low => 0x29,
                },
                MPS::X10 => match self.accuracy {
                    Accuracy::High => 0x37,
                    Accuracy::Medium => 0x21,
                    Accuracy::Low => 0x2A,
                },
            };
            (self.mode.mps as u8, lsb)
        };

        self.i2c_write(&[msb, lsb])
    }
}
