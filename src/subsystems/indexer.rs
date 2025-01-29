use crate::constants::robotmap;
use frcrs::ctre::{ControlMode, Talon};
use frcrs::laser_can::LaserCan;

pub struct Indexer {
    motor: Talon,
    laser_can: LaserCan,
}

impl Default for Indexer {
    fn default() -> Self {
        Self::new()
    }
}

impl Indexer {
    pub fn new() -> Self {
        let motor = Talon::new(robotmap::indexer::MOTOR, None);

        Self { motor, laser_can }
    }

    pub fn set_speed(&self, speed: f64) {
        self.motor.set(ControlMode::Percent, speed);
    }

    pub async fn intake_coral(&self) {
        while self.laser_can.get_measurement() > robotmap::indexer::DISTANCE {
            self.motor.set(ControlMode::Percent, 1.0); //may be -1
        }
        self.motor.stop();
    }

    pub fn stop(&self) {
        self.motor.stop();
    }
}
