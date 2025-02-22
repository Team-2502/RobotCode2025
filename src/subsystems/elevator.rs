use crate::constants;
use crate::constants::{elevator, robotmap};
use frcrs::ctre::{ControlMode, Talon};
use std::fmt::Display;
use std::time::Duration;
use tokio::time::sleep;

pub struct Elevator {
    left: Talon,
    right: Talon,
    target: ElevatorPosition,
}

#[derive(Copy, Clone, Debug, PartialEq)]
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
        let right = Talon::new(robotmap::elevator::RIGHT, Some("can0".to_string())); //should be inverted (clockwise positive) in config
                                                                                     // println!("left pos: {} right pos: {} diff: {}",right.get_position(),left.get_position(),right.get_position() - left.get_position());
        left.zero();
        right.zero();

        //right.follow(&left, true);

        Self {
            left,
            right,
            target: ElevatorPosition::Bottom,
        }
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
    pub fn get_target(&self) -> ElevatorPosition {
        self.target
    }
    /// in rotations, from left motor
    pub fn get_position(&self) -> f64 {
        self.left.get_position()
    }

    /// Runs a trapezoidal (motion magic) profile on the elevator krakens to move the elevator to its stored target position.
    /// Most of the interesting stuff for this is in the elevator krakens' configurations (set in pheonix tuner).
    /// Those configuration files have been saved to Documents on the driver station laptop.
    pub fn run_to_target_trapezoid(&mut self) -> bool {
        //load position to run to in rotations from constants.rs
        let target_position = match self.get_target() {
            ElevatorPosition::Bottom => elevator::BOTTOM,
            ElevatorPosition::L2 => elevator::L2,
            ElevatorPosition::L3 => elevator::L3,
            ElevatorPosition::L4 => elevator::L4,
        };

        // current implementation is to just set the control mode and call this function every frame
        self.right.set(ControlMode::MotionMagic, target_position);
        self.left.follow(&self.right, true);

        // println!("left pos: {} right pos: {} diff: {}",self.right.get_position(),self.left.get_position(),self.right.get_position() - self.left.get_position());
        if (target_position - self.right.get_position()).abs() < 0.5 {
            // println!("Elevator at target");
            true
        } else {
            // println!("Elevator not at target {}", (target_position - self.right.get_position()).abs());
            false
        }

        // below is for when we eventually make this function async
        /*
        while (self.right.get_position() - target_position).abs() < elevator::POSITION_TOLERANCE {
            self.right.set(ControlMode::MotionMagic, target_position);
            self.left.set(ControlMode::MotionMagic, target_position);
        }
         */
    }

    pub async fn run_to_target_trapezoid_async(&mut self) {
        let target_position = match self.get_target() {
            ElevatorPosition::Bottom => elevator::BOTTOM,
            ElevatorPosition::L2 => elevator::L2,
            ElevatorPosition::L3 => elevator::L3,
            ElevatorPosition::L4 => elevator::L4,
        };

        while (self.right.get_position() - target_position).abs() > elevator::POSITION_TOLERANCE {
            self.right.set(ControlMode::MotionMagic, target_position);
            self.left.follow(&self.right, true);

            sleep(Duration::from_millis(20)).await;
        }
        println!("elevator at target");
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
