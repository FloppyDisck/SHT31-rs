use thiserror::Error;

pub type Result<T> = std::result::Result<T, SHTError>;

#[derive(Error, Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum SHTError {
    #[error("Write Read I2C Error")]
    WriteReadI2CError,
    #[error("Write I2C Error")]
    WriteI2CError,
    #[error("Humidity bytes [{bytes_start:#x}, {bytes_end:#x}] expected {expected_checksum:#x} but got the checksum {calculated_checksum:#x}")]
    InvalidHumidityChecksumError {
        bytes_start: u8,
        bytes_end: u8,
        expected_checksum: u8,
        calculated_checksum: u8,
    },
    #[error("Temperature bytes [{bytes_start:#x}, {bytes_end:#x}] expected {expected_checksum:#x} but got the checksum {calculated_checksum:#x}")]
    InvalidTemperatureChecksumError {
        bytes_start: u8,
        bytes_end: u8,
        expected_checksum: u8,
        calculated_checksum: u8,
    },
    #[error("Single shot reading timeout")]
    ReadingTimeoutError,
    #[error("This error should not happen")]
    PlaceholderError,
}
