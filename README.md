# SHT31-rs
A cargo library for the SHT31 temperature / humidity sensors

## Usage
```rust
use sht31::prelude::*;

// Requires an 12c connection only
let sht31 = SHT31::new(i2c);

let reading = sht31.read().unwrap();
```

## Advanced Usage
```rust
use sht31::prelude::*;

// Use single shot, with high accuracy, an alternate I2C address 
// and return temp data in Fahrenheit
let sht31 = SHT31::new(sht_i2c)
        .with_mode(SingleShot::new())
        .with_accuracy(Accuracy::High)
        .with_unit(TemperatureUnit::Fahrenheit)
        .with_address(DeviceAddr::AD1);

// Start measuring before you need the reading, 
// this is more efficient than waiting for readings
sht31.measure().unwrap();

// Some other code here...

let reading = sht31.read().unwrap();

```