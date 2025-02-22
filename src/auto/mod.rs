mod path;

use std::cell::RefMut;
use crate::auto::path::drive;
use nalgebra::Vector2;
use uom::si::{
    f64::{Length},
    length::meter,
};
use serde::{Deserialize, Serialize};
use std::ops::Deref;
use std::time::Duration;
use tokio::join;
use tokio::time::{sleep, Instant};

use crate::{constants, Ferris, score};
use crate::subsystems::{Drivetrain, Elevator, ElevatorPosition, Indexer, LineupSide};
use crate::subsystems::ElevatorPosition::{L2, L4};

#[derive(Serialize, Deserialize)]
pub enum Auto {
    Nothing,
    BlueTriangle,
    Blue180,
    BlueLong,
    Blue2,
}

impl Auto {
    pub fn from_dashboard(s: &str) -> Self {
        match s {
            "Nothing" => Auto::Nothing,
            "BlueTriangle" => Auto::BlueTriangle,
            "Blue180" => Auto::Blue180,
            "BlueLong" => Auto::BlueLong,
            "Blue2" => Auto::Blue2,
            _ => Auto::Nothing,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Auto::Nothing => "Nothing",
            Auto::BlueTriangle => "BlueTriangle",
            Auto::Blue180 => "Blue180",
            Auto::BlueLong => "BlueLong",
            Auto::Blue2 => "Blue2",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![Auto::Nothing, Auto::BlueTriangle, Auto::Blue180, Auto::BlueLong, Auto::Blue2]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

    pub async fn run_auto<'a>(ferris: Ferris, chosen: Auto) {
        match chosen {
            Auto::Nothing => {}
            Auto::BlueTriangle => {
                blue_triangle(ferris).await.expect("Failed running auto");
            }
            Auto::Blue180 => {
                blue_180(ferris).await.expect("Failed running auto");
            }
            Auto::BlueLong => {
                blue_long(ferris).await.expect("Failed running auto")
            }
            Auto::Blue2 => {
                blue_2(ferris).await.expect("Failed running auto")
            }
        }
    }
}

pub async fn async_score(
    drivetrain: &mut Drivetrain,
    lineup_side: LineupSide,
    elevator: &mut Elevator,
    indexer: &mut Indexer,
    elevator_position: ElevatorPosition,
) -> bool {
    elevator.set_target(elevator_position);
    println!("running async_score");

    join! (
        async {
            loop {
                drivetrain.update_limelight().await;
                drivetrain.post_odo().await;

                if drivetrain.lineup(lineup_side, elevator_position).await {
                    break
                }

                sleep(Duration::from_millis(20)).await;
            }
            println!("lineup done")
        },
        elevator.run_to_target_trapezoid_async()
    );

    println!("ready to score");
    while indexer.get_laser_dist() < constants::indexer::LASER_TRIP_DISTANCE_MM {
        let indexer_speed = match elevator_position {
            ElevatorPosition::Bottom => -0.5,
            ElevatorPosition::L2 => -0.5,
            ElevatorPosition::L3 => -0.5,
            ElevatorPosition::L4 => -0.25
        };
        indexer.set_speed(indexer_speed);
    }
    println!("game piece past laser");

    sleep(Duration::from_secs_f64(0.5)).await;
    indexer.stop();

    true
}

pub async fn blue_triangle(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain
        .odometry
        .set_abs(Vector2::new(Length::new::<meter>(8.075126647949219), Length::new::<meter>(2.0993127822875977)));

    drive("BlueTriangle", &mut drivetrain, 1).await?;
    println!("BlueTriangle.1 done");

    sleep(Duration::from_secs_f64(1.)).await;

    drive("BlueTriangle", &mut drivetrain, 2).await?;
    println!("BlueTriangle.2 done");

    sleep(Duration::from_secs_f64(1.)).await;

    drive("BlueTriangle", &mut drivetrain, 3).await?;
    println!("BlueTriangle.3 done");

    Ok(())
}


pub async fn blue_180(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain
        .odometry
        .set_abs(Vector2::new(Length::new::<meter>(7.8775811195373535), Length::new::<meter>(6.840074062347412)));

    drive("Blue180", &mut drivetrain, 1).await?;
    println!("Blue180.1 done");

    Ok(())
}

pub async fn blue_long(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain.reset_heading();

    drivetrain
        .odometry
        .set_abs(Vector2::new(Length::new::<meter>(7.5), Length::new::<meter>(7.)));

    drive("BlueLong", &mut drivetrain, 1).await?;
    println!("BlueLong.1 done");

    Ok(())
}

pub async fn blue_2(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut elevator = robot.elevator.deref().borrow_mut();
    let mut indexer = robot.indexer.deref().borrow_mut();

    drivetrain.reset_heading();

    drivetrain
        .odometry
        .set_abs(Vector2::new(Length::new::<meter>(8.020708084106445), Length::new::<meter>(7.632927417755127)));

    println!("driving path 1");
    join!(
        drive("Blue2", &mut drivetrain, 1),
        async {
            sleep(Duration::from_secs_f64(2.5)).await;
            elevator.set_target(L4);
            elevator.run_to_target_trapezoid();
        }
    );

    println!("drove path 1");
    let start = Instant::now();
    while Instant::now().duration_since(start) < Duration::from_secs_f64(1.5) {
        drivetrain.update_limelight().await;
        sleep(Duration::from_millis(20)).await;
    }
    println!("wait finished");

    async_score(&mut drivetrain, LineupSide::Left, &mut elevator, &mut indexer, L4).await;

    Ok(())
}

async fn wait_indexer(indexer: &mut Indexer) {
    while indexer.get_laser_dist() < constants::indexer::LASER_TRIP_DISTANCE_MM {
        indexer.set_speed(-0.25);
    }

    indexer.stop();
}