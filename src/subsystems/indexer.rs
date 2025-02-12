use crate::constants::robotmap;
use crate::{constants, Ferris};
use frcrs::ctre::{ControlMode, Talon};
use frcrs::laser_can::LaserCan;
use std::cell::RefCell;
use std::rc::Rc;
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
        let laser_can = LaserCan::new(robotmap::indexer::LASER_CAN);

        Self { motor, laser_can }
    }

    pub fn set_speed(&self, speed: f64) {
        self.motor.set(ControlMode::Percent, speed);
    }

    pub async fn intake_coral(indexer: Rc<RefCell<Indexer>>) {
        if let Ok(indexer) = indexer.try_borrow_mut() {
            while indexer.get_laser_dist() > constants::indexer::LASER_TRIP_DISTANCE_MM
                || indexer.laser_can.get_measurement() == -1
            {
                println!("Dist: {}", indexer.get_laser_dist());
                indexer.set_speed(-0.3);
            }

            indexer.motor.stop();
        }
    }

    pub fn get_laser_dist(&self) -> i32 {
        self.laser_can.get_measurement()
    }

    pub fn stop(&self) {
        self.motor.stop();
    }
}
