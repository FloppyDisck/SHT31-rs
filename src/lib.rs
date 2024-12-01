#![no_std]

pub mod error;
pub mod mode;

use crate::mode::{Periodic, SimpleSingleShot, SingleShot};
use crc::{Algorithm, Crc};
use embedded_hal::{delay::DelayNs, i2c::I2c};

pub use crate::error::{Result, SHTError};
pub mod prelude {
    pub use super::{
        mode::{Periodic, Sht31Measure, Sht31Reader, SimpleSingleShot, SingleShot, MPS},
        Accuracy, DeviceAddr, Reading, Status, TemperatureUnit, SHT31,
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
    heater: bool,
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

#[derive(Copy, Clone, Debug)]
pub struct Status {
    /// Last checksum transfer failed
    pub checksum_failed: bool,
    /// Last received command has been processed
    pub last_command_processed: bool,
    /// System was reset recently
    pub system_reset: bool,
    /// T has an alert
    pub t_alert: bool,
    /// RH has an alert
    pub rh_alert: bool,
    /// Heater status
    pub heater_on: bool,
    /// At least one pending alert
    pub pending_alert: bool,
}

fn bit_flag(n: u16, pos: u8) -> bool {
    n & (1 << pos) != 0
}

impl Status {
    fn from_bytes(bytes: u16) -> Self {
        Self {
            checksum_failed: bit_flag(bytes, 0),
            last_command_processed: !bit_flag(bytes, 1),
            system_reset: bit_flag(bytes, 4),
            t_alert: bit_flag(bytes, 10),
            rh_alert: bit_flag(bytes, 11),
            heater_on: bit_flag(bytes, 13),
            pending_alert: bit_flag(bytes, 15),
        }
    }
}

fn calculate_checksum(crc: &Crc<u8>, msb: u8, lsb: u8) -> u8 {
    let mut digest = crc.digest();
    digest.update(&[msb, lsb]);
    digest.finalize()
}

fn verify_reading(buffer: [u8; 6]) -> Result<()> {
    let crc = Crc::<u8>::new(&CRC_ALGORITHM);

    let temp_result = calculate_checksum(&crc, buffer[0], buffer[1]);
    if temp_result != buffer[2] {
        return Err(SHTError::InvalidTemperatureChecksumError {
            bytes_start: buffer[0],
            bytes_end: buffer[1],
            expected_checksum: buffer[2],
            calculated_checksum: temp_result,
        });
    }

    let humidity_result = calculate_checksum(&crc, buffer[3], buffer[4]);
    if humidity_result != buffer[5] {
        return Err(SHTError::InvalidHumidityChecksumError {
            bytes_start: buffer[3],
            bytes_end: buffer[4],
            expected_checksum: buffer[5],
            calculated_checksum: humidity_result,
        });
    }

    Ok(())
}

impl<Mode, I2C> SHT31<Mode, I2C> {
    /// Verifies the two bytes against the returned checksum
    fn verify_data(buffer: [u8; 6]) -> Result<()> {
        verify_reading(buffer)
    }
}

impl<I2C, D> SHT31<SimpleSingleShot<D>, I2C>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Create a new sensor
    /// I2C clock frequency must must be between 0 and 1000 kHz
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self::simple_single_shot(i2c, SimpleSingleShot::new(delay))
    }

    pub fn simple_single_shot(i2c: I2C, mode: SimpleSingleShot<D>) -> Self {
        Self {
            mode,
            i2c,
            address: DeviceAddr::default() as u8,
            unit: TemperatureUnit::default(),
            accuracy: Accuracy::default(),
            heater: false,
        }
    }
}

impl<I2C> SHT31<Periodic, I2C>
where
    I2C: I2c,
{
    pub fn periodic(i2c: I2C, mode: Periodic) -> SHT31<Periodic, I2C> {
        Self {
            mode,
            i2c,
            address: DeviceAddr::default() as u8,
            unit: TemperatureUnit::default(),
            accuracy: Accuracy::default(),
            heater: false,
        }
    }
}

impl<I2C> SHT31<SingleShot, I2C>
where
    I2C: I2c,
{
    pub fn single_shot(i2c: I2C, mode: SingleShot) -> SHT31<SingleShot, I2C> {
        Self {
            mode,
            i2c,
            address: DeviceAddr::default() as u8,
            unit: TemperatureUnit::default(),
            accuracy: Accuracy::default(),
            heater: false,
        }
    }
}

#[allow(dead_code)]
impl<Mode, I2C> SHT31<Mode, I2C>
where
    I2C: I2c,
{
    /// Changes the SHT31 mode
    pub fn with_mode<NewMode>(self, mode: NewMode) -> SHT31<NewMode, I2C> {
        SHT31 {
            mode,
            i2c: self.i2c,
            address: self.address,
            accuracy: self.accuracy,
            unit: self.unit,
            heater: false,
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

    /// Set the heater's heating state
    pub fn set_heating(&mut self, heating: bool) -> Result<()> {
        self.heater = heating;
        self.switch_heater()
    }

    /// Enables the onboard heater
    pub fn with_heating(mut self) -> Result<Self> {
        self.heater = true;
        self.switch_heater()?;
        Ok(self)
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    /// Switch the heater on or off
    fn switch_heater(&mut self) -> Result<()> {
        let lsb = if self.heater { 0x6D } else { 0x66 };

        self.i2c_write(&[0x30, lsb])
    }

    /// Cancel the currently running command, this is necessary for when attempting to transition
    /// between single shot and periodic
    pub fn break_command(&mut self) -> Result<()> {
        self.i2c_write(&[0x30, 0x93])
    }

    /// Trigger a soft reset
    pub fn soft_reset(&mut self) -> Result<()> {
        self.i2c_write(&[0x30, 0xA2])
    }

    /// Triggers an I2C general reset, keep in mind that this will reset all
    /// I2C devices connected to this line
    pub fn reset(&mut self) -> Result<()> {
        self.i2c_write(&[0x00, 0x06])
    }

    /// Read the sensors status
    pub fn status(&mut self) -> Result<Status> {
        let mut buffer = [0; 3];

        self.i2c_write_read(&[0xF3, 0x2D], &mut buffer)?;

        // Verify data
        let calculated = calculate_checksum(&Crc::<u8>::new(&CRC_ALGORITHM), buffer[0], buffer[1]);
        if calculated != buffer[2] {
            return Err(SHTError::InvalidStatusChecksumError {
                bytes_start: buffer[0],
                bytes_end: buffer[1],
                expected_checksum: buffer[2],
                calculated_checksum: calculated,
            });
        }

        Ok(Status::from_bytes(u16::from_be_bytes([
            buffer[0], buffer[1],
        ])))
    }

    /// Clear all status registers
    pub fn clear_status(&mut self) -> Result<()> {
        self.i2c_write(&[0x30, 0x41])
    }

    /// Consumes the instance and returns the i2c
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    fn i2c_write(&mut self, bytes: &[u8]) -> Result<()> {
        match self.i2c.write(self.address, bytes) {
            Ok(res) => Ok(res),
            Err(_) => Err(SHTError::WriteI2CError),
        }
    }

    fn i2c_read(&mut self, buffer: &mut [u8]) -> Result<()> {
        match self.i2c.read(self.address, buffer) {
            Ok(res) => Ok(res),
            Err(_) => Err(SHTError::ReadI2CError),
        }
    }

    fn i2c_write_read(&mut self, bytes: &[u8], buffer: &mut [u8]) -> Result<()> {
        match self.i2c.write_read(self.address, bytes, buffer) {
            Ok(res) => Ok(res),
            Err(_) => Err(SHTError::WriteReadI2CError),
        }
    }

    fn process_data(&self, buffer: [u8; 6]) -> Result<Reading> {
        Self::verify_data(buffer)?;

        let raw_temp = i16::from_be_bytes([buffer[0], buffer[1]]) as f32;

        let (sub, mul) = match self.unit {
            TemperatureUnit::Celsius => CELSIUS_PAIR,
            TemperatureUnit::Fahrenheit => FAHRENHEIT_PAIR,
        };

        let pre_sub = mul * (raw_temp / CONVERSION_DENOM);

        let temperature = pre_sub - sub;

        let raw_humidity = i16::from_be_bytes([buffer[0], buffer[1]]) as f32;
        let humidity = 100f32 * raw_humidity / CONVERSION_DENOM;

        Ok(Reading {
            temperature,
            humidity,
        })
    }
}

#[cfg(test)]
mod test {
    extern crate alloc;
    use alloc::vec;
    use alloc::vec::Vec;
    use super::*;
    use crate::prelude::*;
    use embedded_hal_mock::common::Generic;
    use embedded_hal_mock::eh1::delay::CheckedDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use rstest::rstest;

    impl SHT31<SingleShot, Generic<Transaction>> {
        fn done(mut self) {
            self.i2c.done()
        }
    }

    impl SHT31<Periodic, Generic<Transaction>> {
        fn done(mut self) {
            self.i2c.done()
        }
    }

    impl SHT31<SimpleSingleShot<CheckedDelay>, Generic<Transaction>> {
        fn done(mut self) {
            self.i2c.done();
            self.mode.delay.done();
        }
    }

    #[test]
    fn reading() {
        let buffer = [98, 153, 188, 98, 32, 139];

        assert!(verify_reading(buffer).is_ok());

        let corrupt_temperature = [98, 153, 180, 98, 32, 139];

        assert_eq!(
            verify_reading(corrupt_temperature).err().unwrap(),
            SHTError::InvalidTemperatureChecksumError {
                bytes_start: 98,
                bytes_end: 153,
                expected_checksum: 180,
                calculated_checksum: 188
            }
        );

        let corrupt_humidity = [98, 153, 188, 98, 32, 180];
        assert_eq!(
            verify_reading(corrupt_humidity).err().unwrap(),
            SHTError::InvalidHumidityChecksumError {
                bytes_start: 98,
                bytes_end: 32,
                expected_checksum: 180,
                calculated_checksum: 139
            }
        );
    }

    #[test]
    fn status() {
        let status = Status::from_bytes(0x8010);
        assert!(status.pending_alert);
        assert!(!status.heater_on);
        assert!(!status.rh_alert);
        assert!(!status.t_alert);
        assert!(status.system_reset);
        assert!(status.last_command_processed);
        assert!(!status.checksum_failed);
    }

    fn single_shot_expectations(msb: u8, lsb: u8) -> [Transaction; 2] {
        [
            Transaction::write(DeviceAddr::AD0 as u8, Vec::from(&[msb, lsb])),
            Transaction::read(
                DeviceAddr::AD0 as u8,
                Vec::from(&[98, 153, 188, 98, 32, 139]),
            ),
        ]
    }

    #[rstest]
    #[case(0x10, Accuracy::Low)]
    #[case(0x0D, Accuracy::Medium)]
    #[case(0x06, Accuracy::High)]
    fn simple_single_shot(#[case] lsb: u8, #[case] accuracy: Accuracy) {
        let i2c = Mock::new(&single_shot_expectations(0x2C, lsb));

        let mut sht31 = SHT31::new(i2c, CheckedDelay::new([])).with_accuracy(accuracy);
        let reading = sht31.read().unwrap();
        assert_eq!(reading.humidity, 38.515297);
        assert_eq!(reading.temperature, 72.32318);

        sht31.done();
    }

    #[rstest]
    #[case(0x16, Accuracy::Low)]
    #[case(0x0B, Accuracy::Medium)]
    #[case(0x00, Accuracy::High)]
    fn single_shot(#[case] lsb: u8, #[case] accuracy: Accuracy) {
        let i2c = Mock::new(&single_shot_expectations(0x24, lsb));

        let mut sht31 = SHT31::single_shot(i2c, SingleShot::new()).with_accuracy(accuracy);
        sht31.measure().unwrap();
        let reading = sht31.read().unwrap();
        assert_eq!(reading.humidity, 38.515297);
        assert_eq!(reading.temperature, 72.32318);

        sht31.done()
    }

    #[rstest]
    #[case(0x20, 0x32, false, Accuracy::High, MPS::Half)]
    #[case(0x20, 0x24, false, Accuracy::Medium, MPS::Half)]
    #[case(0x20, 0x2F, false, Accuracy::Low, MPS::Half)]
    #[case(0x21, 0x30, false, Accuracy::High, MPS::Normal)]
    #[case(0x21, 0x26, false, Accuracy::Medium, MPS::Normal)]
    #[case(0x21, 0x2D, false, Accuracy::Low, MPS::Normal)]
    #[case(0x22, 0x36, false, Accuracy::High, MPS::Double)]
    #[case(0x22, 0x20, false, Accuracy::Medium, MPS::Double)]
    #[case(0x22, 0x2B, false, Accuracy::Low, MPS::Double)]
    #[case(0x23, 0x34, false, Accuracy::High, MPS::X4)]
    #[case(0x23, 0x22, false, Accuracy::Medium, MPS::X4)]
    #[case(0x23, 0x29, false, Accuracy::Low, MPS::X4)]
    #[case(0x27, 0x37, false, Accuracy::High, MPS::X10)]
    #[case(0x27, 0x21, false, Accuracy::Medium, MPS::X10)]
    #[case(0x27, 0x2A, false, Accuracy::Low, MPS::X10)]
    #[case(0x2B, 0x32, true, Accuracy::Low, MPS::Half)]
    fn periodic(
        #[case] msb: u8,
        #[case] lsb: u8,
        #[case] art: bool,
        #[case] accuracy: Accuracy,
        #[case] mps: MPS,
    ) {
        let expectations = [
            Transaction::write(DeviceAddr::AD0 as u8, vec![msb, lsb]),
            Transaction::write_read(
                DeviceAddr::AD0 as u8,
                vec![0xE0, 0x00],
                vec![98, 153, 188, 98, 32, 139],
            ),
        ];
        let i2c = Mock::new(&expectations);

        let mut periodic = Periodic::new().with_mps(mps);
        if art {
            periodic.set_art();
        }

        let mut sht31 = SHT31::periodic(i2c, periodic).with_accuracy(accuracy);
        sht31.measure().unwrap();
        let reading = sht31.read().unwrap();
        assert_eq!(reading.humidity, 38.515297);
        assert_eq!(reading.temperature, 72.32318);

        sht31.done();
    }

    #[test]
    fn common_interactions() {
        let expectations = [
            // Heater On
            Transaction::write(DeviceAddr::AD0 as u8, vec![0x30, 0x6D]),
            // Heater Off
            Transaction::write(DeviceAddr::AD0 as u8, vec![0x30, 0x66]),
            // Break
            Transaction::write(DeviceAddr::AD0 as u8, vec![0x30, 0x93]),
            // Soft reset
            Transaction::write(DeviceAddr::AD0 as u8, vec![0x30, 0xA2]),
            // Reset
            Transaction::write(DeviceAddr::AD0 as u8, vec![0x00, 0x06]),
            // Reset Status
            Transaction::write(DeviceAddr::AD0 as u8, vec![0x30, 0x41]),
        ];
        let i2c = Mock::new(&expectations);

        // Heating
        let mut sht31 = SHT31::new(i2c, CheckedDelay::new([]))
            .with_heating()
            .unwrap();
        sht31.set_heating(false).unwrap();

        // Break
        sht31.break_command().unwrap();

        // Resets
        sht31.soft_reset().unwrap();
        sht31.reset().unwrap();

        // Status
        sht31.clear_status().unwrap();

        sht31.done();
    }
}
