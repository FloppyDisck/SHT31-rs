use crate::{
    error::{Result, SHTError::PlaceholderError},
    mode::{single_shot::single_shot_read, Sht31Reader},
    Accuracy, Reading, SHT31,
};
use embedded_hal::{delay::DelayNs, i2c::I2c};

/// A simple reading that blocks until the measurement is obtained
#[derive(Copy, Clone, Debug)]
pub struct SimpleSingleShot<D: DelayNs> {
    max_retries: u8,
    ms_delay: u32,
    delay: D,
}

impl<D> SimpleSingleShot<D>
where
    D: DelayNs,
{
    #[allow(dead_code)]
    pub fn new(delay: D) -> Self {
        Self {
            max_retries: 8,
            ms_delay: 100,
            delay,
        }
    }
    /// Sets the max number of retries to read a sensor before giving up
    pub fn set_max_retries(&mut self, max_retries: u8) {
        self.max_retries = max_retries
    }
    /// Sets the max number of retries to read a sensor before giving up
    pub fn with_max_retries(mut self, max_retries: u8) -> Self {
        self.set_max_retries(max_retries);
        self
    }
    /// Sets the millisecond delay between each try
    pub fn set_delay(&mut self, ms_delay: u32) {
        self.ms_delay = ms_delay
    }
    /// Sets the millisecond delay between each try
    pub fn with_delay(mut self, ms_delay: u32) -> Self {
        self.set_delay(ms_delay);
        self
    }
}

impl<I2C, D> Sht31Reader for SHT31<SimpleSingleShot<D>, I2C>
where
    I2C: I2c,
    D: DelayNs,
{
    /// It will initiate a read and wont stop until its either exhausted its retries or a reading is found
    fn read(&mut self) -> Result<Reading> {
        // Commence reading
        let lsb = match self.accuracy {
            Accuracy::High => 0x06,
            Accuracy::Medium => 0x0D,
            Accuracy::Low => 0x10,
        };

        self.i2c_write(&[0x2C, lsb])?;

        // TODO: figure out clock stretching
        let mut read_attempt = Err(PlaceholderError);

        for _ in 0..self.mode.max_retries {
            read_attempt = single_shot_read(self);

            if read_attempt.is_err() {
                self.mode.delay.delay_ms(self.mode.ms_delay);
            } else {
                return read_attempt;
            }
        }
        read_attempt
    }
}
