use crate::mode::Periodic;
use crate::prelude::{Sht31Measure, Sht31Reader};
use crate::SHT31;
use bevy_ecs::prelude::*;
use embedded_hal::blocking::i2c;
use robotrs::check_timer;
use robotrs::dev::*;

pub struct SHT31Component<I2C>
where
    I2C: i2c::WriteRead + i2c::Write + 'static + Send + Sync,
{
    sensors: Vec<(String, SHT31<Periodic, I2C>, Option<Duration>)>,
}

impl<I2C: i2c::WriteRead + i2c::Write + Send + Sync> SHT31Component<I2C> {
    pub fn new() -> Self {
        Self { sensors: vec![] }
    }

    pub fn add(&mut self, name: String, sensor: SHT31<Periodic, I2C>) {
        self.sensors.push((name, sensor, None));
    }

    pub fn add_with_timer(
        &mut self,
        name: String,
        sensor: SHT31<Periodic, I2C>,
        duration: Duration,
    ) {
        self.sensors.push((name, sensor, Some(duration)));
    }

    pub fn with(mut self, name: String, sensor: SHT31<Periodic, I2C>) -> Self {
        self.add(name, sensor);
        self
    }

    pub fn with_timer(
        mut self,
        name: String,
        sensor: SHT31<Periodic, I2C>,
        duration: Duration,
    ) -> Self {
        self.add_with_timer(name, sensor, duration);
        self
    }
}

pub struct InitializedSHT31Component {
    pub sensors: Vec<Entity>,
}

impl<I2C: i2c::WriteRead + i2c::Write + Send + Sync> Module<InitializedSHT31Component>
    for SHT31Component<I2C>
{
    fn init(self, robot: &mut Robot) -> InitializedSHT31Component {
        robot.add_system(sht31_reading::<I2C>);
        let mut initialized = vec![];

        for (name, mut sensor, duration) in self.sensors {
            sensor.measure().unwrap();

            let s = SensorBuilder::new(&format!("SHT31 - {}", name), robot)
                .with_type(&sensor)
                .with_output(OutputType::Temperature)
                .with_output(OutputType::Humidity)
                .with_timer(duration)
                .with_component(sensor)
                .build();

            initialized.push(s);
        }

        InitializedSHT31Component {
            sensors: initialized,
        }
    }
}

impl<I2C: Send + Sync> Descriptor for SHT31<Periodic, I2C> {
    fn id(&self) -> u8 {
        0
    }

    fn name(&self) -> String {
        "SHT31".to_string()
    }

    fn description(&self) -> String {
        "SHT31 temperature and humidity sensor".to_string()
    }
}

fn sht31_reading<I2C>(
    mut timer: Query<(Entity, Option<&mut Timer>), With<SHT31<Periodic, I2C>>>,
    mut sensor: Query<(&mut SHT31<Periodic, I2C>, &Features)>,
    mut output_query: Query<(&mut Reading, &Metadata)>,
) where
    I2C: i2c::WriteRead + i2c::Write + Send + Sync,
{
    for (entity, timer) in timer.iter_mut() {
        check_timer!(timer);

        let (mut sensor, features) = sensor.get_mut(entity).unwrap();

        let sensor_reading = sensor.read().unwrap();

        for feature in features.iter() {
            let (mut reading, metadata) = output_query.get_mut(*feature).unwrap();

            if OutputType::Temperature.id() == metadata.id {
                reading.set(sensor_reading.temperature as f64);
            } else {
                reading.set(sensor_reading.humidity as f64);
            }
        }
    }
}
