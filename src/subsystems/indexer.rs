use frcrs::ctre::{ControlMode, Talon};
use crate::constants::robotmap;

pub struct Indexer {
    motor: Talon,
}

impl Indexer {
    pub fn new() -> Self {
        let motor = Talon::new(robotmap::indexer::MOTOR, None);

        Self {
            motor,
        }
    }

    pub fn set_speed(&self, speed: f64) {
        self.motor.set(ControlMode::Percent, speed);
    }

    pub fn stop(&self) {
        self.motor.stop();
    }
}