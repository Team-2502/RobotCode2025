pub mod subsystems;
pub mod swerve;
pub mod constants;
pub mod container;

use std::cell::RefCell;
use std::future::Future;
use std::ops::Deref;
use std::pin::Pin;
use std::rc::Rc;
use std::sync::Arc;
use tokio::sync::Mutex;
use frcrs::input::Joystick;
use frcrs::{deadzone, Robot, TaskManager};
use frcrs::networktables::NetworkTable;
use frcrs::telemetry::Telemetry;
use tokio::task::spawn_local;
use uom::si::angle::{degree, radian};
use crate::constants::drivetrain::SWERVE_TURN_KP;
use crate::container::control_drivetrain;
use crate::subsystems::{Drivetrain, DrivetrainControlState, Elevator, LineupSide};

#[derive(Clone)]
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
    pub task_manager: TaskManager,
    pub controllers: Controllers,

    pub drivetrain: Rc<RefCell<Drivetrain>>,
    pub elevator: Rc<RefCell<Elevator>>,

    teleop_state: Rc<RefCell<TeleopState>>,
}

impl Ferris {
    pub fn new() -> Self {
        Ferris {
            task_manager: TaskManager::new(),
            controllers: Controllers {
                left_drive: Joystick::new(1),
                right_drive: Joystick::new(0),
                operator: Joystick::new(2),
            },
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            elevator: Rc::new(RefCell::new(Elevator::new())),
            teleop_state: Default::default(),
        }
    }
}


impl Robot for Ferris {
    fn robot_init(&mut self) {
        Telemetry::init(5807);

        NetworkTable::init();
    }

    fn disabled_init(&mut self) {
        println!("Disabled init");
    }

    fn autonomous_init(&mut self) {
        println!("Autonomous init");
    }

    fn teleop_init(&mut self) {
        println!("Teleop init");
    }

    fn test_init(&mut self) {
        println!("Test init");
    }

    async fn disabled_periodic(&mut self) {
        if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
            drivetrain.update_limelight().await;
            drivetrain.post_odo().await;
        }
    }

    async fn autonomous_periodic(&mut self) {
        // println!("Autonomous periodic");
    }

    async fn teleop_periodic(&mut self) {
        let TeleopState {
            ref mut drivetrain_state,
        } = *self.teleop_state.deref().borrow_mut();

        if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
            drivetrain.update_limelight().await;
            drivetrain.post_odo().await;

            if self.controllers.right_drive.get(3) {
                drivetrain.lineup(LineupSide::Left).await;
            } else if self.controllers.right_drive.get(4) {
                drivetrain.lineup(LineupSide::Right).await;
            } else {
                control_drivetrain(&mut drivetrain, &mut self.controllers, drivetrain_state).await;
            }
        }

        if let Ok(mut elevator) = self.elevator.try_borrow_mut() {
            if self.controllers.operator.get(3) {
                elevator.set_speed(0.5);
            } else if self.controllers.operator.get(4) {
                elevator.set_speed(-0.5);
            } else {
                elevator.set_speed(0.0);
            }
        }
    }

    async fn test_periodic(&mut self) {
        // println!("Test periodic");
    }
}
