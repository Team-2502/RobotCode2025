pub mod subsystems;
pub mod swerve;
pub mod constants;
pub mod container;

use frcrs::input::Joystick;

pub struct Controllers {
    pub left_drive: Joystick,
    pub right_drive: Joystick,
    pub operator: Joystick
}

pub struct Ferris;

impl Ferris {
    pub fn new() -> Self {
        Ferris
    }
}