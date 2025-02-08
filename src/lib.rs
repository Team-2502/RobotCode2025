pub mod auto;
pub mod constants;
pub mod container;
pub mod subsystems;
pub mod swerve;

use crate::auto::Auto;
use crate::container::control_drivetrain;
use crate::subsystems::{Climber, Drivetrain, DrivetrainControlState, Elevator, ElevatorPosition, Indexer, LineupSide, Vision};
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
use frcrs::ctre::ControlMode;
use tokio::task::{AbortHandle, spawn_local};
use tokio::time::sleep;
use crate::constants::elevator;

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
    pub climber: Arc<Rc<RefCell<Climber>>>,

    teleop_state: Rc<RefCell<TeleopState>>,

    auto_handle: Option<tokio::task::AbortHandle>,
    elevator_trapezoid_handle: Option<tokio::task::AbortHandle>,
    indexer_intake_handle: Option<AbortHandle>,
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
            climber: Arc::new(Rc::new(RefCell::new(Climber::new()))),

            teleop_state: Default::default(),

            auto_handle: None,
            elevator_trapezoid_handle: None,
            indexer_intake_handle: None,
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
            drivetrain.update_limelight().await;
            drivetrain.post_odo().await;

            if self
                .controllers
                .right_drive
                .get(constants::joystick_map::LINEUP_LEFT)
            {
                drivetrain.lineup_2d(LineupSide::Left);
            } else if self
                .controllers
                .right_drive
                .get(constants::joystick_map::LINEUP_RIGHT)
            {
                drivetrain.lineup_2d(LineupSide::Right);
            } else {
                control_drivetrain(&mut drivetrain, &mut self.controllers, drivetrain_state).await;
            }
        }

        if let Ok(mut elevator) = self.elevator.try_borrow_mut() {
            // Setting the target position
            if self
                .controllers
                .operator
                .get(constants::joystick_map::SET_TARGET_L2)
            {
                elevator.set_target(ElevatorPosition::L2);
            } else if self
                .controllers
                .operator
                .get(constants::joystick_map::SET_TARGET_L3)
            {
                elevator.set_target(ElevatorPosition::L3);
            } else if self
                .controllers
                .operator
                .get(constants::joystick_map::SET_TARGET_L4)
            {
                elevator.set_target(ElevatorPosition::L4);
            }

            // Setting the actual elevator operation
            if self
                .controllers
                .operator
                .get(constants::joystick_map::ELEVATOR_UP_MANUAL)
            {
                // Manual up
                elevator.set_speed(0.1);
            } else if self
                .controllers
                .operator
                .get(constants::joystick_map::ELEVATOR_DOWN_MANUAL)
            {
                // Manual down
                elevator.set_speed(-0.1);
            } else if self
                .controllers
                .operator
                .get(constants::joystick_map::ELEVATOR_TRAPEZOID_TO_STORED_TARGET)
            {
                // Trapezoidal to stored target
                elevator.run_to_target_trapezoid();
            } else {
                elevator.set_speed(0.0);
            }
        }

        if let Ok(indexer) = self.indexer.try_borrow_mut() {
            //println!("laser dist: {}", indexer.get_laser_dist());
            if self
                .controllers
                .left_drive
                .get(constants::joystick_map::INDEXER_OUT)
            {
                // Out the front
                indexer.set_speed(0.3);
            } else if self
                .controllers
                .left_drive
                .get(constants::joystick_map::INDEXER_IN)
            {
                    if self.indexer_intake_handle.is_none() {
                        let f = self.clone();
                        let handle = spawn_local(Indexer::intake_coral(f.indexer)).abort_handle();
                        self.elevator_trapezoid_handle = Some(handle);
                }
            } else if let Some(handle) = self.elevator_trapezoid_handle.take() {
                handle.abort();
            } else {
                indexer.stop();
            }
        }

        if self.controllers.operator.get(constants::joystick_map::ELEVATOR_TRAPEZOID_TO_STORED_TARGET_ASYNC) {
            // If no abort handle is stored, this function isn't already running
            // (prevents duplicate tasks)
            if self.elevator_trapezoid_handle.is_none() {
                // Clone Ferris
                // Functions called this way need to:
                // be defined in the free space (not any impl block of lib)
                // and take a robot struct to do their work with
                // This prevents issues with values passing out of scope & such
                let f = self.clone();
                // Start a task from the async function's returned future and set handle to the abort handle for that task
                let handle = spawn_local(elevator_move_to_target_async(f)).abort_handle();
                // Store the abort handle in the Ferris struct
                self.elevator_trapezoid_handle = Some(handle);
            }
        } else if let Some(handle) = self.elevator_trapezoid_handle.take() {
            // If the button is no longer held, use the stored abort handle to abort the task & regain full robot control
            handle.abort();
        }

        // TODO: make more ergonomic, maybe move away from frcrs task manager in favor for abort handle in ferris struct
        // Untested
        let climber = Arc::clone(&self.climber);
        let climb = {
            let climber = Arc::clone(&climber);
            move || {
                let climber = Arc::clone(&climber);
                async move {
                    if let Ok(climber) = climber.try_borrow_mut() {
                        climber.climb().await;
                    };
                }
            }
        };

        let climber = Arc::clone(&self.climber);
        let cancel_climb = {
            let climber = Arc::clone(&climber);
            move || {
                let climber = Arc::clone(&climber);
                async move {
                    if let Ok(climber) = climber.try_borrow_mut() {
                        climber.set_raise(false);
                        climber.set_grab(false);
                    };
                }
            }
        };

        if self
            .controllers
            .operator
            .get(constants::joystick_map::CLIMB_FULL)
        {
            self.task_manager.run_task(climb);
        } else {
            self.task_manager.run_task(cancel_climb);
            self.task_manager.abort_task(climb);
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
            elevator.run_to_target_trapezoid();;
        }
        println!("End of elevator_move_to_target_async");
    }
}