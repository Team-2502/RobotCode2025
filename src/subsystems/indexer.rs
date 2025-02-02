use crate::constants::robotmap;
use frcrs::ctre::{ControlMode, Talon};
use frcrs::laser_can::LaserCan;
use std::time::Duration;
use std::time::Instant;
use tokio::time::sleep;

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
        let mut last_loop = Instant::now();

        while self.laser_can.get_measurement() > robotmap::indexer::DISTANCE {
            self.motor.set(ControlMode::Percent, 1.0); //may be -1 need to test

            // Cap at 250 hz
            let elapsed = last_loop.elapsed().as_secs_f64();
            let left = (1. / 250. - elapsed).max(0.);
            sleep(Duration::from_secs_f64(left)).await;
            last_loop = Instant::now();
        }
        self.motor.stop();
    }

    pub fn stop(&self) {
        self.motor.stop();
    }
}
