mod path;

use crate::auto::path::drive;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use std::cell::RefMut;
use std::ops::Deref;
use std::time::Duration;
use tokio::join;
use tokio::time::{sleep, Instant, timeout};
use uom::si::{f64::Length, length::meter};
use uom::si::angle::degree;
use uom::si::f64::Angle;

use crate::subsystems::{Drivetrain, Elevator, ElevatorPosition, Indexer, LineupSide};
use crate::{constants, score, Ferris};
use crate::constants::indexer::{BOTTOM_SPEED, INTAKE_SPEED, L2_SPEED, L3_SPEED, L4_SPEED, LASER_TRIP_DISTANCE_MM};

#[derive(Serialize, Deserialize)]
pub enum Auto {
    Nothing,
    BlueTriangle,
    Blue180,
    BlueLong,
    Blue2,
    RotationTest,
    BlueMidLeft2,
    Center1,
}

impl Auto {
    pub fn from_dashboard(s: &str) -> Self {
        match s {
            "Nothing" => Auto::Nothing,
            "BlueTriangle" => Auto::BlueTriangle,
            "Blue180" => Auto::Blue180,
            "BlueLong" => Auto::BlueLong,
            "Blue2" => Auto::Blue2,
            "RotationTest" => Auto::RotationTest,
            "BlueMidLeft2" => Auto::BlueMidLeft2,
            "Center1" => Auto::Center1,
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
            Auto::RotationTest => "RotationTest",
            Auto::BlueMidLeft2 => "BlueMidLeft2",
            Auto::Center1 => "Center1",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![
            Auto::Nothing,
            // Auto::BlueTriangle,
            // Auto::Blue180,
            // Auto::BlueLong,
            Auto::Blue2,
            // Auto::RotationTest,
            Auto::BlueMidLeft2,
            Auto::Center1,
        ]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

    pub async fn run_auto<'a>(ferris: Ferris, chosen: Auto) {
        match chosen {
            Auto::Nothing => {
                println!("No auto was selected!");
            }
            Auto::BlueTriangle => {
                blue_triangle(ferris).await.expect("Failed running auto");
            }
            Auto::Blue180 => {
                blue_180(ferris).await.expect("Failed running auto");
            }
            Auto::BlueLong => blue_long(ferris).await.expect("Failed running auto"),
            Auto::Blue2 => blue_2(ferris).await.expect("Failed running auto"),
            Auto::RotationTest => rotation_test(ferris).await.expect("Failed running auto"),
            Auto::BlueMidLeft2 => blue_mid_left_2(ferris).await.expect("Failed running auto"),
            Auto::Center1 => center_1(ferris).await.expect("Failed running auto"),
        }
    }
}

pub async fn wait<F>(mut condition: F)
where F: FnMut() -> bool {
    loop {
        if condition() { return };
        sleep(Duration::from_millis(20)).await;
    }
}

pub async fn async_score(
    drivetrain: &mut Drivetrain,
    lineup_side: LineupSide,
    elevator: &mut Elevator,
    indexer: &mut Indexer,
    elevator_position: ElevatorPosition,
    dt: Duration,
) -> bool {
    elevator.set_target(elevator_position);

    join!(
        async {
            loop {
                drivetrain.update_limelight().await;
                drivetrain.post_odo().await;

                if drivetrain.lineup(lineup_side, elevator_position, dt).await {
                    break;
                }

                sleep(Duration::from_millis(20)).await;
            }
        },
        elevator.run_to_target_trapezoid_async()
    );

    drivetrain.stop();

    while indexer.get_laser_dist() < constants::indexer::LASER_TRIP_DISTANCE_MM {
        let indexer_speed = match elevator_position {
            ElevatorPosition::Bottom => BOTTOM_SPEED,
            ElevatorPosition::L2 => L2_SPEED,
            ElevatorPosition::L3 => L3_SPEED,
            ElevatorPosition::L4 => L4_SPEED,
        };
        indexer.set_speed(indexer_speed);
    }
    sleep(Duration::from_secs_f64(0.5)).await;
    indexer.stop();

    true
}

pub async fn blue_triangle(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(8.075126647949219),
        Length::new::<meter>(2.0993127822875977),
    ));

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

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(7.8775811195373535),
        Length::new::<meter>(6.840074062347412),
    ));

    drive("Blue180", &mut drivetrain, 1).await?;
    println!("Blue180.1 done");

    Ok(())
}

pub async fn blue_long(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain.reset_heading();

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(7.5),
        Length::new::<meter>(7.),
    ));

    drive("BlueLong", &mut drivetrain, 1).await?;
    println!("BlueLong.1 done");

    Ok(())
}

pub async fn blue_2(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut elevator = robot.elevator.deref().borrow_mut();
    let mut indexer = robot.indexer.deref().borrow_mut();

    drivetrain.reset_heading_offset(Angle::new::<degree>(180.));

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(8.020708084106445),
        Length::new::<meter>(7.632927417755127),
    ));

    join!(drive("Blue2", &mut drivetrain, 1), async {
        elevator.set_target(ElevatorPosition::L2);
        elevator.run_to_target_trapezoid();

        indexer.set_speed(INTAKE_SPEED);
        wait(|| indexer.get_laser_dist() < LASER_TRIP_DISTANCE_MM && indexer.get_laser_dist() != -1).await;
        indexer.stop();

        elevator.set_target(ElevatorPosition::L4);
        elevator.run_to_target_trapezoid();
    });

    let _ = timeout(Duration::from_secs_f64(0.75), async {
        loop {
            drivetrain.update_limelight().await;
            sleep(Duration::from_millis(20)).await;
        }
    }).await;

    async_score(
        &mut drivetrain,
        LineupSide::Left,
        &mut elevator,
        &mut indexer,
        ElevatorPosition::L4,
        robot.dt,
    )
    .await;

    join!(drive("Blue2", &mut drivetrain, 3), async {
        elevator.set_target(ElevatorPosition::Bottom);
        elevator.run_to_target_trapezoid_async().await;

        indexer.set_speed(INTAKE_SPEED);

        wait(|| indexer.get_laser_dist() < LASER_TRIP_DISTANCE_MM && indexer.get_laser_dist() != -1).await;

        indexer.stop();
    });

    join!(drive("Blue2", &mut drivetrain, 4), async {
        sleep(Duration::from_secs_f64(1.25)).await;
        elevator.set_target(ElevatorPosition::L4);
        elevator.run_to_target_trapezoid();
    });

    let _ = timeout(Duration::from_secs_f64(0.5), async {
        loop {
            drivetrain.update_limelight().await;
            sleep(Duration::from_millis(20)).await;
        }
    }).await;

    async_score(
        &mut drivetrain,
        LineupSide::Right,
        &mut elevator,
        &mut indexer,
        ElevatorPosition::L4,
        robot.dt,
    )
    .await;

    Ok(())
}

async fn rotation_test(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain.reset_heading();

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(7.84511661529541),
        Length::new::<meter>(6.556429862976074),
    ));

    drive("RotationTest", &mut drivetrain, 1).await?;

    Ok(())
}

async fn blue_mid_left_2(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut elevator = robot.elevator.deref().borrow_mut();
    let mut indexer = robot.indexer.deref().borrow_mut();

    drivetrain.reset_heading_offset(Angle::new::<degree>(180.));

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(7.2230658531188965),
        Length::new::<meter>(5.444962978363037)
    ));

    join!(
        drive("BlueHighMid2", &mut drivetrain, 1),
        async {
            sleep(Duration::from_secs_f64(0.5)).await;
            elevator.set_target(ElevatorPosition::L4);
            elevator.run_to_target_trapezoid();
        }
    );

    let _ = timeout(Duration::from_secs_f64(0.5), async {
        loop {
            drivetrain.update_limelight().await;
            sleep(Duration::from_millis(20)).await;
        }
    }).await;

    async_score(
        &mut drivetrain,
        LineupSide::Right,
        &mut elevator,
        &mut indexer,
        ElevatorPosition::L4,
        robot.dt,
    ).await;

    join!(drive("BlueHighMid2", &mut drivetrain, 3), async {
        elevator.set_target(ElevatorPosition::Bottom);
        elevator.run_to_target_trapezoid_async().await;

        indexer.set_speed(INTAKE_SPEED);

        wait(|| indexer.get_laser_dist() < LASER_TRIP_DISTANCE_MM && indexer.get_laser_dist() != -1).await;

        indexer.stop();
    });

    join!(drive("BlueHighMid2", &mut drivetrain, 4), async {
        sleep(Duration::from_secs_f64(1.25)).await;
        elevator.set_target(ElevatorPosition::L4);
        elevator.run_to_target_trapezoid();
    });

    let _ = timeout(Duration::from_secs_f64(0.5), async {
        loop {
            drivetrain.update_limelight().await;
            sleep(Duration::from_millis(20)).await;
        }
    }).await;

    async_score(
        &mut drivetrain,
        LineupSide::Left,
        &mut elevator,
        &mut indexer,
        ElevatorPosition::L4,
        robot.dt,
    )
        .await;

    Ok(())
}

async fn center_1(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {



    Ok(())
}