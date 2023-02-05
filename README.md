# SHT31-rs &emsp; [![Build Status]][actions] [![Latest Version]][crates.io]

[Build Status]: https://img.shields.io/github/actions/workflow/status/FloppyDisck/SHT31-rs/rust.yml?branch=main
[actions]: https://github.com/FloppyDisck/SHT31-rs/actions?query=branch%3Amain
[Latest Version]: https://img.shields.io/crates/v/sht31.svg
[crates.io]: https://crates.io/crates/sht31

A cargo library for the SHT31 temperature / humidity sensors

---

## Usage
The default sensor usage includes a blocking sensor read that blocks 
the current thread until the sensor does a reading.
```rust
use sht31::prelude::*;

fn main() -> Result<()> {
    // Requires an 12c connection only
    let sht = SHT31::new(i2c);
    
    loop {
        let reading = sht.read()?;
    }
}
```

---

## Advanced Single Shot Usage
Breaks down the simple usage into two commands, one where 
the sensor is notified to start reading and another where 
the reading is requested
```rust
use sht31::prelude::*;

fn main() -> Result<()> {
    // i2c setup
    
    // Use single shot, with high accuracy, an alternate I2C address 
    // and return temp data in Fahrenheit
    let sht = SHT31::new(i2c)
        .with_mode(SingleShot::new())
        .with_accuracy(Accuracy::High)
        .with_unit(TemperatureUnit::Fahrenheit)
        .with_address(DeviceAddr::AD1);

    loop {
        // Start measuring before you need the reading, 
        // this is more efficient than waiting for readings
        sht.measure()?;

        // Some other code here...

        let reading = sht.read()?;
    }
}
```

---

## Periodic Usage
Periodic mode continually reads data at a rate of the 
given measurements per second (MPS) and at a quality 
of the given Accuracy. This means that you dont have to 
worry about running `SHT31::measure()` every time you want 
to read data
```rust
use sht31::prelude::*;

fn main() -> Result<()> {
    // i2c setup
    
    // In periodic mode, the sensor keeps updating the reading
    // without needing to measure
    let mut sht = SHT31::new(sht_i2c)
        .with_mode(Periodic::new().with_mps(MPS::Normal))
        .with_accuracy(Accuracy::High);
    
    // Trigger the measure before running your loop to initialize the periodic mode
    sht.measure()?;
    
    loop {
        let reading = sht.read()?;
    }
}
```

---

## Periodic with Accelerated Response Time
Periodic mode has a unique mode that allows the sensor to read data at 4Hz
```rust
use sht31::prelude::*;

fn main() -> Result<()> {
    
    // Makes the sensor acquire the data at 4 Hz
    let mut sht = SHT31::new(sht_i2c)
        .with_mode(Periodic::new().with_art());
    sht.measure()?;
    
    loop {
        let reading = sht.read()?;
    }
}
```

---

## Mode switching
This crate also supports more complex case scenarios where you might want to switch 
between period to single shot for example
```rust
use sht31::prelude::*;

fn main() -> Result<()> {
    // i2c setup
    
    let mut sht = SHT31::new(sht_i2c)
        .with_mode(Periodic::new());
    
    // Trigger the measure before running your loop to initialize the periodic mode
    sht.measure()?;
    
    // Do a periodic read
    let reading = sht.read()?;
    
    // Cancel the currently running command (periodic)
    sht.break_command()?;
    
    sht = sht.with_mode(SingleShot::new());
    
    sht.measure()?;
    
    let new_reading = sht.read()?;
}
```