pub mod error;
pub mod mode;

use crc::{Algorithm, Crc};
use embedded_hal::blocking::i2c;
use error::{Result, SHTError};
use std::fmt::Debug;
use crate::mode::SimpleSingleShot;

pub mod prelude {
    pub use super::{
        mode::Periodic, mode::Sht31Reader, mode::SingleShot, mode::MPS, Accuracy, DeviceAddr,
        Reading, TemperatureUnit, SHT31,
    };
}

const CRC_ALGORITHM: Algorithm<u8> = Algorithm {
    width: 8,
    poly: 0x31,
    init: 0xFF,
    refin: false,
    refout: false,
    xorout: 0x00,
    check: 0x00,
    residue: 0x00,
};

// 2**16 - 1
const CONVERSION_DENOM: f32 = 65535f32;

// Constants used to convert values
const CELSIUS_PAIR: (f32, f32) = (45f32, 175f32);
const FAHRENHEIT_PAIR: (f32, f32) = (49f32, 315f32);

/// The temperature and humidity sensor
#[derive(Copy, Clone, Debug)]
pub struct SHT31<Mode, I2C> {
    mode: Mode,
    i2c: I2C,
    address: u8,
    accuracy: Accuracy,
    unit: TemperatureUnit,
}

/// Represents the reading gotten from the sensor
#[derive(Default, Clone, Copy, Debug)]
pub struct Reading {
    pub temperature: f32,
    pub humidity: f32,
}

/// The two supported I2C addresses
#[allow(dead_code)]
#[derive(Default, Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum DeviceAddr {
    #[default]
    AD0 = 0x44,
    AD1 = 0x45,
}

/// Influences what the reading temperature numbers are
#[allow(dead_code)]
#[derive(Default, Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum TemperatureUnit {
    Celsius,
    #[default]
    Fahrenheit,
}

/// Determines the accuracy of the sensor, the higher the repeatability
/// the longer it'll take and the more accurate it will be
#[allow(dead_code)]
#[derive(Default, Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum Accuracy {
    #[default]
    High,
    Medium,
    Low,
}

fn merge_bytes(a: u8, b: u8) -> u16 {
    ((a as u16) << 8) | b as u16
}

fn verify_data(buffer: [u8; 6]) -> Result<()> {
    let crc = Crc::<u8>::new(&CRC_ALGORITHM);

    let mut temp_digest = crc.digest();
    temp_digest.update(&[buffer[0], buffer[1]]);
    let temp_result = temp_digest.finalize();
    if temp_result != buffer[2] {
        return Err(SHTError::InvalidTemperatureChecksumError {
            bytes_start: buffer[0],
            bytes_end: buffer[1],
            expected_checksum: buffer[2],
            calculated_checksum: temp_result,
        });
    }

    let mut humidity_digest = crc.digest();
    humidity_digest.update(&[buffer[3], buffer[4]]);
    let humidity_result = humidity_digest.finalize();
    if humidity_result != buffer[5] {
        return Err(SHTError::InvalidHumidityChecksumError {
            bytes_start: buffer[3],
            bytes_end: buffer[4],
            expected_checksum: buffer[5],
            calculated_checksum: humidity_result,
        });
    }

    return Ok(());
}

impl<Mode, I2C> SHT31<Mode, I2C> {
    /// Merges two bytes so the result is both, ex merge_bytes(0x20, 0x33) = 0x2033
    fn merge_bytes(a: u8, b: u8) -> u16 {
        merge_bytes(a, b)
    }

    /// Verifies the two bytes against the returned checksum
    fn verify_data(buffer: [u8; 6]) -> Result<()> {
        verify_data(buffer)
    }
}

#[allow(dead_code)]
impl<I2C> SHT31<SimpleSingleShot, I2C>
    where
        I2C: i2c::WriteRead + i2c::Write,
{
    /// Create a new sensor
    /// I2C clock frequency must must be between 0 and 1000 kHz
    pub fn new(i2c: I2C) -> Self {
        Self {
            mode: SimpleSingleShot::new(),
            i2c,
            address: DeviceAddr::default() as u8,
            unit: TemperatureUnit::default(),
            accuracy: Accuracy::default(),
        }
    }
}

#[allow(dead_code)]
impl<Mode, I2C> SHT31<Mode, I2C>
    where
        I2C: i2c::WriteRead + i2c::Write,
{
    /// Changes the SHT31 mode
    pub fn with_mode<NewMode>(self, mode: NewMode) -> SHT31<NewMode, I2C> {
        SHT31 {
            mode,
            i2c: self.i2c,
            address: self.address,
            accuracy: self.accuracy,
            unit: self.unit,
        }
    }

    /// Change the sensor's temperature unit
    pub fn set_unit(&mut self, unit: TemperatureUnit) {
        self.unit = unit;
    }

    /// Change the sensor's temperature unit
    pub fn with_unit(mut self, unit: TemperatureUnit) -> Self {
        self.unit = unit;
        self
    }

    /// Change the sensor's accuracy which also influences how long it takes to read
    pub fn set_accuracy(&mut self, accuracy: Accuracy) {
        self.accuracy = accuracy;
    }

    /// Change the sensor's accuracy which also influences how long it takes to read
    pub fn with_accuracy(mut self, accuracy: Accuracy) -> Self {
        self.accuracy = accuracy;
        self
    }

    /// Change the sensor's I2C address
    pub fn with_address(mut self, address: DeviceAddr) -> Self {
        self.address = address as u8;
        self
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    fn i2c_write(&mut self, bytes: &[u8]) -> Result<()> {
        match self.i2c.write(self.address, bytes) {
            Ok(res) => Ok(res),
            Err(_) => Err(SHTError::WriteI2CError),
        }
    }

    fn i2c_read(&mut self, bytes: &[u8], buffer: &mut [u8]) -> Result<()> {
        match self.i2c.write_read(self.address, bytes, buffer) {
            Ok(res) => Ok(res),
            Err(_) => Err(SHTError::WriteReadI2CError),
        }
    }

    fn process_data(&self, buffer: [u8; 6]) -> Result<Reading> {
        Self::verify_data(buffer)?;

        let raw_temp = Self::merge_bytes(buffer[0], buffer[1]) as f32;

        let (sub, mul) = match self.unit {
            TemperatureUnit::Celsius => {
                // TODO: figure out why this is necessary
                #[cfg(feature = "esp32-fix")]
                println!();
                CELSIUS_PAIR
            }
            TemperatureUnit::Fahrenheit => {
                #[cfg(feature = "esp32-fix")]
                println!();
                FAHRENHEIT_PAIR
            }
        };

        let pre_sub = mul * (raw_temp / CONVERSION_DENOM);

        // This needs to be printed, if not temperature = - temperature
        // i swear to god im not making this up
        #[cfg(feature = "esp32-fix")]
        println!("{}", pre_sub);

        let temperature = pre_sub - sub;

        let raw_humidity = Self::merge_bytes(buffer[3], buffer[4]) as f32;
        let humidity = 100f32 * raw_humidity / CONVERSION_DENOM;

        Ok(Reading {
            temperature,
            humidity,
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn byte_merge() {
        let a = 0x20;
        let b = 0x33;
        assert_eq!(merge_bytes(a, b), 0x2033);
    }

    #[test]
    fn verify_checksum() {
        let buffer = [98, 153, 188, 98, 32, 139];

        assert!(verify_data(buffer).is_ok());

        let corrupt_temperature = [98, 153, 180, 98, 32, 139];

        assert_eq!(
            verify_data(corrupt_temperature).err().unwrap().to_string(),
            SHTError::InvalidTemperatureChecksumError {
                bytes_start: 98,
                bytes_end: 153,
                expected_checksum: 180,
                calculated_checksum: 188
            }
                .to_string()
        );

        let corrupt_humidity = [98, 153, 188, 98, 32, 180];
        assert_eq!(
            verify_data(corrupt_humidity).err().unwrap().to_string(),
            SHTError::InvalidHumidityChecksumError {
                bytes_start: 98,
                bytes_end: 32,
                expected_checksum: 180,
                calculated_checksum: 139
            }
                .to_string()
        );
    }
}
