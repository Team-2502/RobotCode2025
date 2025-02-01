pub mod auto;
pub mod constants;
pub mod container;
pub mod subsystems;
pub mod swerve;

use std::cell::RefCell;

use crate::auto::Auto;
use crate::container::control_drivetrain;
use crate::subsystems::{
    Climber, Drivetrain, DrivetrainControlState, Elevator, Indexer, LineupSide,
};
use frcrs::input::Joystick;
use frcrs::networktables::NetworkTable;
use frcrs::telemetry::Telemetry;
use frcrs::{Robot, TaskManager};
use std::ops::Deref;
use std::rc::Rc;
use std::sync::Arc;
use tokio::task::spawn_local;

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
                left_drive: Joystick::new(1),
                right_drive: Joystick::new(0),
                operator: Joystick::new(2),
            },
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            elevator: Rc::new(RefCell::new(Elevator::new())),
            indexer: Rc::new(RefCell::new(Indexer::new())),
            climber: Arc::new(Rc::new(RefCell::new(Climber::new()))),

            teleop_state: Default::default(),

            auto_handle: None,
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
        Telemetry::put_string("selected auto", Auto::Nothing.name().to_string()).await;
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
                eprintln!("Failed to get selected auto from telemetry.");
            }
        }
    }

    async fn teleop_periodic(&mut self) {
        let TeleopState {
            ref mut drivetrain_state,
        } = *self.teleop_state.deref().borrow_mut();

        if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
            //drivetrain.update_limelight().await;
            drivetrain.post_odo().await;

            if self.controllers.right_drive.get(3) {
                drivetrain.lineup(LineupSide::Left).await;
            } else if self.controllers.right_drive.get(4) {
                drivetrain.lineup(LineupSide::Right).await;
            } else {
                control_drivetrain(&mut drivetrain, &mut self.controllers, drivetrain_state).await;
            }
        }

        if let Ok(elevator) = self.elevator.try_borrow_mut() {
            if self.controllers.operator.get(3) {
                elevator.set_speed(0.5);
            } else if self.controllers.operator.get(4) {
                elevator.set_speed(-0.5);
            } else {
                elevator.set_speed(0.0);
            }
        }

        if let Ok(indexer) = self.indexer.try_borrow_mut() {
            if self.controllers.operator.get(1) {
                indexer.set_speed(0.5);
            } else if self.controllers.operator.get(2) {
                indexer.set_speed(-0.5);
            } else {
                indexer.set_speed(0.0);
            }
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

        if self.controllers.operator.get(8) {
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
