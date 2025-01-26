pub mod subsystems;
pub mod swerve;
pub mod constants;
pub mod container;

use std::cell::RefCell;
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;
use std::sync::Arc;
use tokio::sync::Mutex;
use frcrs::input::Joystick;
use frcrs::{Robot, TaskManager};
use frcrs::networktables::NetworkTable;
use frcrs::telemetry::Telemetry;
use crate::subsystems::{Drivetrain, DrivetrainControlState, LineupSide};

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

    pub drivetrain: Arc<Mutex<Drivetrain>>,

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
            drivetrain: Arc::new(Mutex::new(Drivetrain::new())),
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
        // println!("Disabled periodic");
    }

    async fn autonomous_periodic(&mut self) {
        // println!("Autonomous periodic");
    }

    async fn teleop_periodic(&mut self) {
        let drivetrain = Arc::clone(&self.drivetrain);
        let dt_update = {
            let drivetrain = Arc::clone(&drivetrain);
            move || {
                let drivetrain = Arc::clone(&drivetrain);
                async move {
                    let mut drivetrain = drivetrain.lock().await;
                    drivetrain.update_limelight().await;
                    drivetrain.post_odo().await;
                }
            }
        };

        self.task_manager.run_task(dt_update);

        let drivetrain = Arc::clone(&self.drivetrain);
        if self.controllers.right_drive.get(3) {
            self.task_manager.run_task(lineup_task(drivetrain, LineupSide::Left));
        } else {
            self.task_manager.abort_task(lineup_task(drivetrain, LineupSide::Left));
        }

        let drivetrain = Arc::clone(&self.drivetrain);
        if self.controllers.right_drive.get(4) {
            self.task_manager.run_task(lineup_task(drivetrain, LineupSide::Right));
        } else {
            self.task_manager.abort_task(lineup_task(drivetrain, LineupSide::Right));
        }
    }

    async fn test_periodic(&mut self) {
        println!("Test periodic");
    }
}

fn lineup_task(drivetrain: Arc<Mutex<Drivetrain>>, side: LineupSide) -> impl FnMut() -> Pin<Box<dyn Future<Output = ()> + Send>> {
    move || {
        let drivetrain = Arc::clone(&drivetrain);
        Box::pin(async move {
            drivetrain.lock().await.lineup(side).await;
        })
    }
}