use crate::mode::Periodic;
use crate::SHT31;
use embedded_hal::blocking::i2c;
use robotrs::component::feature::FeatureType;
use robotrs::component::sensor::SensorBuilder;
use robotrs::component::{RobotComponent, TypeDescriptor};
use robotrs::{Entity, Robot};

pub struct SHT31Component<I2C>
where
    I2C: i2c::WriteRead + i2c::Write,
{
    sensors: Vec<(String, SHT31<Periodic, I2C>)>,
}

impl<I2C> SHT31Component<I2C> {
    pub fn new() -> Self {
        Self { sensors: vec![] }
    }

    pub fn add(&mut self, name: String, sensor: SHT31<Periodic, I2C>) {
        self.sensors.push((name, sensor));
    }

    pub fn with(mut self, name: String, sensor: SHT31<Periodic, I2C>) -> Self {
        self.add(name, sensor);
        self
    }
}

pub struct InitializedSHT31Component {
    pub sensors: Vec<Entity>,
}

impl<I2C> RobotComponent<InitializedSHT31Component> for SHT31Component<I2C> {
    fn init(self, robot: &mut Robot) -> InitializedSHT31Component {
        robot.add_system(ReadSensor, "SHT31_Sensor_Reading", &[]);
        self.partial_init_as_ref(robot)
    }

    fn partial_init_as_ref(&self, robot: &mut Robot) -> InitializedSHT31Component {
        let mut initialized = vec![];

        for (name, sensor) in self.sensors.iter() {
            sensor.measure().unwrap();

            let s = SensorBuilder::new(&format!("SHT31 - {}", name), robot)
                .with_type(&sensor)
                .with_feature(FeatureType::Temperature)
                .with_feature(FeatureType::Humidity)
                .build()
                .with_custom_feature(sensor);

            initialized.push(s.entity);
        }

        InitializedSHT31Component {
            sensors: initialized,
        }
    }
}

impl<I2C> TypeDescriptor for SHT31<Periodic, I2C> {
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
