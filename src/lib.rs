pub mod subsystems;
pub mod swerve;
pub mod constants;
pub mod container;

use std::cell::RefCell;
use std::rc::Rc;
use frcrs::input::Joystick;
use crate::subsystems::{Drivetrain, DrivetrainControlState};

pub struct Controllers {
    pub left_drive: Joystick,
    pub right_drive: Joystick,
    pub operator: Joystick
}

#[derive(Default)]
struct TeleopState {
    drivetrain_state: DrivetrainControlState,
}

pub struct Ferris {
    pub drivetrain: Rc<RefCell<Drivetrain>>,

    teleop_state: Rc<RefCell<TeleopState>>,
}

impl Ferris {
    pub fn new() -> Self {
        Ferris {
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            teleop_state: Default::default(),
        }
    }
}