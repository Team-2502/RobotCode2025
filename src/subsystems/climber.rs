use crate::constants::robotmap::climber::*;
use crate::Ferris;
use frcrs::solenoid::{ModuleType, Solenoid};
use std::time::Duration;
use frcrs::ctre::{ControlMode, Talon};
use tokio::time::sleep;
use crate::constants::climber::{CLIMBER_CURRENT_THRESHOLD, CLIMB_SPEED};
use crate::constants::robotmap;

pub struct Climber {
    motor: Talon
}

impl Default for Climber {
    fn default() -> Self {
        Self::new()
    }
}

impl Climber {
    pub fn new() -> Self {
        Self {
            motor: Talon::new(CLIMBER_MOTOR_ID, Some("can0".to_string()))
        }
    }

    pub fn set(&self, speed: f64){
        self.motor.set(ControlMode::Percent, speed);
    }

    pub fn climb(&self) {
        if self.motor.get_position() < 380. {
            self.set(CLIMB_SPEED);
        } else {
            self.set(0.);
        }
    }

    pub fn climb_safe(&self) {
        while self.motor.get_current() < CLIMBER_CURRENT_THRESHOLD {
            self.set(CLIMB_SPEED);
        }
        self.set(0.);
    }
}
