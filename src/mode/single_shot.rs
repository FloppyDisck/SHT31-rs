use crate::error::Result;
use crate::mode::{Sht31Measure, Sht31Reader};
use crate::Reading;
use crate::{Accuracy, SHT31};
use embedded_hal::blocking::i2c;

/// Complex read that may require multiple attempts to read output until its ready
#[derive(Default, Copy, Clone, Debug)]
pub struct SingleShot {}

impl SingleShot {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {}
    }
}

pub(crate) fn single_shot_read<Mode, I2C: i2c::WriteRead + i2c::Write>(sensor: &mut SHT31<Mode, I2C>) -> Result<Reading> {
    // TODO: If error is a NACK then return another unique error to identify
    let mut buffer = [0; 6];

    sensor.i2c_read(&[], &mut buffer)?;
    sensor.process_data(buffer)
}

impl<I2C> Sht31Reader for SHT31<SingleShot, I2C>
    where
        I2C: i2c::WriteRead + i2c::Write,
{
    /// Try reading, if the reading is not available yet then it will return an error
    fn read(&mut self) -> Result<Reading> {
        single_shot_read(self)
    }
}

impl<I2C> Sht31Measure for SHT31<SingleShot, I2C>
    where
        I2C: i2c::WriteRead + i2c::Write,
{
    /// Commence measuring
    #[allow(dead_code)]
    fn measure(&mut self) -> Result<()> {
        let lsb = match self.accuracy {
            Accuracy::High => 0x00,
            Accuracy::Medium => 0x0B,
            Accuracy::Low => 0x16,
        };

        self.i2c_write(&[0x24, lsb])
    }
}
