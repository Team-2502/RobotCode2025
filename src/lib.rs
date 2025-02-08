pub mod auto;
pub mod constants;
pub mod container;
pub mod subsystems;
pub mod swerve;

use crate::auto::Auto;
use crate::constants::elevator;
use crate::container::control_drivetrain;
use crate::subsystems::{
    Climber, Drivetrain, DrivetrainControlState, Elevator, ElevatorPosition, Indexer, LineupSide,
    Vision,
};
use constants::joystick_map::*;
use frcrs::ctre::ControlMode;
use frcrs::input::Joystick;
use frcrs::networktables::NetworkTable;
use frcrs::telemetry::Telemetry;
use frcrs::{Robot, TaskManager};
use std::cell::RefCell;
use std::cmp::PartialEq;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::ops::Deref;
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::task::{spawn_local, AbortHandle};
use tokio::time::sleep;

#[derive(Clone)]
pub struct Controllers {
    pub left_drive: Joystick,
    pub right_drive: Joystick,
    pub operator: Joystick,
}

#[derive(Default)]
struct TeleopState {
    drivetrain_state: DrivetrainControlState,
}

#[derive(Clone)]
pub struct Ferris {
    pub task_manager: TaskManager,
    pub controllers: Controllers,

    pub drivetrain: Rc<RefCell<Drivetrain>>,
    pub elevator: Rc<RefCell<Elevator>>,
    pub indexer: Rc<RefCell<Indexer>>,
    pub climber: Rc<RefCell<Climber>>,

    teleop_state: Rc<RefCell<TeleopState>>,

    auto_handle: Option<tokio::task::AbortHandle>,
    elevator_trapezoid_handle: Option<tokio::task::AbortHandle>,
    indexer_intake_handle: Option<AbortHandle>,
    climb_handle: Option<AbortHandle>,
}

impl Default for Ferris {
    fn default() -> Self {
        Self::new()
    }
}

impl Ferris {
    pub fn new() -> Self {
        Ferris {
            task_manager: TaskManager::new(),
            controllers: Controllers {
                left_drive: Joystick::new(constants::joystick_map::LEFT_DRIVE),
                right_drive: Joystick::new(constants::joystick_map::RIGHT_DRIVE),
                operator: Joystick::new(constants::joystick_map::OPERATOR),
            },
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            elevator: Rc::new(RefCell::new(Elevator::new())),
            indexer: Rc::new(RefCell::new(Indexer::new())),
            climber: Rc::new(RefCell::new(Climber::new())),

            teleop_state: Default::default(),

            auto_handle: None,
            elevator_trapezoid_handle: None,
            indexer_intake_handle: None,
            climb_handle: None,
        }
    }

    pub fn stop(&self) {
        if let Ok(drivetrain) = self.drivetrain.try_borrow() {
            drivetrain.stop();
        }

        if let Ok(elevator) = self.elevator.try_borrow() {
            elevator.stop();
        }

        if let Ok(indexer) = self.indexer.try_borrow() {
            indexer.stop();
        }
    }
}

impl Robot for Ferris {
    async fn robot_init(&mut self) {
        Telemetry::init(5807);

        NetworkTable::init();

        Telemetry::put_string(
            "auto chooser",
            serde_json::to_string(&Auto::names()).unwrap(),
        )
        .await;
        Telemetry::put_string("selected auto", Auto::BlueTriangle.name().to_string()).await;
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
        &self.stop();

        if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
            //drivetrain.update_limelight().await;
            drivetrain.post_odo().await;
        }

        if let Some(handle) = self.auto_handle.take() {
            handle.abort();
        }
    }

    async fn autonomous_periodic(&mut self) {
        if self.auto_handle.is_none() {
            let f = self.clone();

            if let Some(selected_auto) = Telemetry::get("selected auto").await {
                let chosen = Auto::from_dashboard(selected_auto.as_str());

                let auto_task = Auto::run_auto(f, chosen);
                let handle = spawn_local(auto_task).abort_handle();
                self.auto_handle = Some(handle);
            } else {
                eprintln!("Failed to get selected auto from telemetry, running default");

                let auto_task = Auto::run_auto(f, Auto::Nothing);
                let handle = spawn_local(auto_task).abort_handle();
                self.auto_handle = Some(handle);
            }
        }
    }

    async fn teleop_periodic(&mut self) {
        let TeleopState {
            ref mut drivetrain_state,
        } = *self.teleop_state.deref().borrow_mut();

        if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
            if let Ok(mut elevator) = self.elevator.try_borrow_mut() {
                if let Ok(mut indexer) = self.indexer.try_borrow_mut() {
                    drivetrain.update_limelight().await;
                    drivetrain.post_odo().await;

                    let side = if self.controllers.right_drive.get(LINEUP_LEFT) {
                        LineupSide::Left
                    } else if self.controllers.right_drive.get(LINEUP_RIGHT) {
                        LineupSide::Right
                    } else {
                        LineupSide::Left
                    };

                    if self.controllers.left_drive.get(SCORE_L2) {
                        score(
                            &mut drivetrain,
                            &mut elevator,
                            &mut indexer,
                            ElevatorPosition::L2,
                            side,
                        )
                    } else if self.controllers.left_drive.get(SCORE_L3) {
                        score(
                            &mut drivetrain,
                            &mut elevator,
                            &mut indexer,
                            ElevatorPosition::L3,
                            side,
                        )
                    } else if self.controllers.left_drive.get(SCORE_L4) {
                        score(
                            &mut drivetrain,
                            &mut elevator,
                            &mut indexer,
                            ElevatorPosition::L4,
                            side,
                        )
                    } else if self.controllers.right_drive.get(INTAKE) {
                        elevator.set_target(ElevatorPosition::L2);
                        elevator.run_to_target_trapezoid();

                        if indexer.get_laser_dist() > constants::indexer::LASER_TRIP_DISTANCE_MM
                            || indexer.get_laser_dist() == -1
                        {
                            indexer.set_speed(-0.25);
                        } else {
                            indexer.stop();
                        }
                    } else {
                        control_drivetrain(
                            &mut drivetrain,
                            &mut self.controllers,
                            drivetrain_state,
                        )
                        .await;

                        elevator.stop();
                        indexer.stop();
                    }
                }
            }
        }

        if self.controllers.right_drive.get(CLIMB) {
            if self.climb_handle.is_none() {
                let f = self.clone();
                let climb_task = Climber::climb(f);
                let handle = spawn_local(climb_task).abort_handle();
                self.climb_handle = Some(handle);
            }
        } else {
            if let Some(handle) = self.climb_handle.take() {
                handle.abort();
            }
        }
    }

    async fn test_periodic(&mut self) {
        // println!("Test periodic");
    }
}
pub async fn elevator_move_to_target_async(robot: Ferris) {
    println!("Called elevator_move_to_target_async");
    if let Ok(mut elevator) = robot.elevator.try_borrow_mut() {
        //println!("Borrowed elevator");
        let target_position = match elevator.get_target() {
            ElevatorPosition::Bottom => elevator::BOTTOM,
            ElevatorPosition::L2 => elevator::L2,
            ElevatorPosition::L3 => elevator::L3,
            ElevatorPosition::L4 => elevator::L4,
        };
        //println!("Error: {}", (elevator.get_position() - target_position).abs());
        //println!("{}", (elevator.get_position() - target_position).abs() > elevator::POSITION_TOLERANCE);
        while (elevator.get_position() - target_position).abs() > elevator::POSITION_TOLERANCE {
            elevator.run_to_target_trapezoid();
        }
        println!("End of elevator_move_to_target_async");
    }
}

pub fn score(
    drivetrain: &mut Drivetrain,
    elevator: &mut Elevator,
    indexer: &mut Indexer,
    elevator_position: ElevatorPosition,
    lineup_side: LineupSide,
) {
    let drivetrain_at_position = drivetrain.lineup_2d(lineup_side);
    elevator.set_target(elevator_position);
    let elevator_at_target = elevator.run_to_target_trapezoid();

    if elevator_at_target && drivetrain_at_position {
        if indexer.get_laser_dist() < constants::indexer::LASER_TRIP_DISTANCE_MM {
            indexer.set_speed(-0.25);
        } else {
            indexer.stop();

            elevator.set_target(ElevatorPosition::Bottom);
            elevator.run_to_target_trapezoid();
        }
    }
}

pub fn after_score(robot: &Ferris) {
    if let Ok(mut elevator) = robot.elevator.try_borrow_mut() {
        if let Ok(mut indexer) = robot.indexer.try_borrow_mut() {
            elevator.set_target(ElevatorPosition::Bottom);
            elevator.run_to_target_trapezoid();

            indexer.stop();
        }
    }
}
