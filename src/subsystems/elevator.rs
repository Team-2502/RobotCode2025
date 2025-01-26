use frcrs::ctre::{ControlMode, Talon};
use crate::constants::elevator;

struct Elevator {
    left: Talon,
    right: Talon,
}
enum ElevatorPosition {
    Bottom,
    L2,
    L3,
    L4,
}

impl Elevator {
    pub fn new() -> Self {
        Self{
            left: Talon::new(1, None),
            right: Talon::new(2, None),
        }
    }
    pub fn set_position(&self, position: ElevatorPosition){
        match position {
            ElevatorPosition::Bottom => {self.left.set(ControlMode::Position, elevator::BOTTOM)}
            ElevatorPosition::L2 => {self.left.set(ControlMode::Position, elevator::L2)}
            ElevatorPosition::L3 => {self.left.set(ControlMode::Position, elevator::L3)}
            ElevatorPosition::L4 => {self.left.set(ControlMode::Position, elevator::L4)}
        }
    }
    pub fn stop(&self) {
        self.left.stop();
        self.right.stop();
    }
}