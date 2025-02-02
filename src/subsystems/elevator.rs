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
    pub fn set_target(&mut self, target: ElevatorPosition) {
        self.target = target;
    }
    pub fn get_target(&self) -> ElevatorPosition {self.target}

    pub fn run_to_target_trapezoid(&mut self) {
        let target_position = match self.get_target(){
            ElevatorPosition::Bottom => {elevator::BOTTOM},
            ElevatorPosition::L2 => {elevator::L2},
            ElevatorPosition::L3 => {elevator::L3},
            ElevatorPosition::L4 => {elevator::L4},
        };
        self.right.set(ControlMode::MotionMagic, target_position);
        self.left.set(ControlMode::MotionMagic, target_position);
        /*
        while (self.right.get_position() - target_position).abs() < elevator::POSITION_TOLERANCE {
            self.right.set(ControlMode::MotionMagic, target_position);
            self.left.set(ControlMode::MotionMagic, target_position);
        }
         */
    }
    pub fn set_position_rotations(&mut self, position: f64){
        self.left.set(ControlMode::Position, position);
        self.right.set(ControlMode::Position, position);
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
