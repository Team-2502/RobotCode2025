use crate::constants::{elevator, robotmap};
use frcrs::ctre::{ControlMode, Talon};

pub struct Elevator {
    left: Talon,
    right: Talon,
}

pub enum ElevatorPosition {
    Bottom,
    L2,
    L3,
    L4,
}

impl Default for Elevator {
    fn default() -> Self {
        Self::new()
    }
}

impl Elevator {
    pub fn new() -> Self {
        let left = Talon::new(robotmap::elevator::LEFT, Some("can0".to_string()));
        let right = Talon::new(robotmap::elevator::RIGHT, Some("can0".to_string()));

        right.follow(&left, true);

        Self { left, right }
    }

    pub fn set_position(&self, position: ElevatorPosition) {
        match position {
            ElevatorPosition::Bottom => self.left.set(ControlMode::Position, elevator::BOTTOM),
            ElevatorPosition::L2 => self.left.set(ControlMode::Position, elevator::L2),
            ElevatorPosition::L3 => self.left.set(ControlMode::Position, elevator::L3),
            ElevatorPosition::L4 => self.left.set(ControlMode::Position, elevator::L4),
        }
    }

    pub fn set_speed(&self, speed: f64) {
        self.left.set(ControlMode::Percent, speed);
    }

    pub fn stop(&self) {
        self.left.stop();
        self.right.stop();
    }
}
