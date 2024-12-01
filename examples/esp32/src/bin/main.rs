#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::prelude::*;
use sht31::prelude::*;
use log::info;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    info!("Initializing SHT31");
    let i2c = I2c::new(
        peripherals.I2C0,
        Config::default()
    );
    let delay = Delay::new();
    let mut sht = SHT31::new(i2c, delay);
    
    loop {
        let reading = sht.read().unwrap();
        info!("Temperature: {}\nHumidity: {}", reading.temperature, reading.humidity);
    }
}
