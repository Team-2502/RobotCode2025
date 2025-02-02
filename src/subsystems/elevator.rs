use std::fmt::Display;
use std::time::Duration;
use crate::constants::{elevator, robotmap};
use frcrs::ctre::{ControlMode, Talon};
use tokio::time::sleep;

pub struct Elevator {
    left: Talon,
    right: Talon,
    target: ElevatorPosition
}

#[derive(Copy, Clone, Debug)]
pub enum ElevatorPosition {
    Bottom,
    L2,
    L3,
    L4,
}
impl PartialEq for ElevatorPosition {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (ElevatorPosition::Bottom, ElevatorPosition::Bottom) => true,
            (ElevatorPosition::L2, ElevatorPosition::L2) => true,
            (ElevatorPosition::L3, ElevatorPosition::L3) => true,
            (ElevatorPosition::L4, ElevatorPosition::L4) => true,
            _ => false,
        }
    }
}


impl Default for Elevator {
    fn default() -> Self {
        Self::new()
    }
}

impl Elevator {
    pub fn new() -> Self {
        let left = Talon::new(robotmap::elevator::LEFT, Some("can0".to_string()));
        let right = Talon::new(robotmap::elevator::RIGHT, Some("can0".to_string())); //should be inverted (clockwise positive) in config

        //right.follow(&left, true);

        Self { left, right, target: ElevatorPosition::Bottom }
    }
    /// Set the stored goal position.
    /// TODO: update the below lines of this comment once controls scheme is figured out.
    /// That position is referenced by run_to_target_trapezoid.
    /// Not much benefit over just passing a target to that function yet but I have plans.
    pub fn set_target(&mut self, target: ElevatorPosition) {
        self.target = target;
    }
    /// Get the stored goal position.
    /// For stuff like LEDs.
    /// Indexer used to reference this to get scoring speed, might need to go back to that once we have real bumpers
    pub fn get_target(&self) -> ElevatorPosition {self.target}

    /// Runs a trapezoidal (motion magic) profile on the elevator krakens to move the elevator to its stored target position.
    /// Most of the interesting stuff for this is in the elevator krakens' configurations (set in pheonix tuner).
    /// Those configuration files have been saved to Documents on the driver station laptop.
    pub fn run_to_target_trapezoid(&mut self) {
        //load position to run to in rotations from constants.rs
        let target_position = match self.get_target(){
            ElevatorPosition::Bottom => {elevator::BOTTOM},
            ElevatorPosition::L2 => {elevator::L2},
            ElevatorPosition::L3 => {elevator::L3},
            ElevatorPosition::L4 => {elevator::L4},
        };

        // current implementation is to just set the control mode and call this function every frame
        self.right.set(ControlMode::MotionMagic, target_position);
        self.left.set(ControlMode::MotionMagic, target_position);

        // below is for when we eventually make this function async
        /*
        while (self.right.get_position() - target_position).abs() < elevator::POSITION_TOLERANCE {
            self.right.set(ControlMode::MotionMagic, target_position);
            self.left.set(ControlMode::MotionMagic, target_position);
        }
         */
    }

    pub fn set_speed(&self, speed: f64) {
        self.left.set(ControlMode::Percent, speed);
        self.right.set(ControlMode::Percent, speed); //is inverted in config
    }

    pub fn stop(&self) {
        self.left.stop();
        self.right.stop();
    }
}
