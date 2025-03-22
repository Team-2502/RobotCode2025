use crate::constants::robotmap;
use crate::{constants, Ferris};
use frcrs::ctre::{ControlMode, Talon};
use frcrs::laser_can::LaserCan;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;
use std::time::Instant;
use tokio::time::sleep;
use crate::constants::indexer::{INDEXER_LASER_DEBOUNCE_TIME_SECONDS, INTAKE_SPEED, LASER_TRIP_DISTANCE_MM};
use crate::subsystems::{DebounceType, Debouncer};

pub struct Indexer {
    motor: Talon,
    laser_can: LaserCan,
    debouncer: Debouncer,
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
        let debouncer = Debouncer::new(Duration::from_secs_f64(INDEXER_LASER_DEBOUNCE_TIME_SECONDS), DebounceType::RISING);

        Self { motor, laser_can, debouncer }
    }

    pub fn set_speed(&self, speed: f64) {
        self.motor.set(ControlMode::Percent, speed);
    }

    pub async fn intake_coral(indexer: Rc<RefCell<Indexer>>) {
        if let Ok(mut indexer) = indexer.try_borrow_mut() {
            while !indexer.is_laser_tripped()
            {
                println!("Dist: {}", indexer.get_laser_dist());
                indexer.set_speed(INTAKE_SPEED);
            }

            indexer.motor.stop();
        }
    }

    pub fn get_laser_dist(&self) -> i32 {
        self.laser_can.get_measurement()
    }

    pub fn is_laser_tripped(&mut self) -> bool {
        self.debouncer.calculate(self.get_laser_dist() < LASER_TRIP_DISTANCE_MM && self.get_laser_dist() != -1)
    }

    pub fn stop(&self) {
        self.motor.stop();
    }
}
