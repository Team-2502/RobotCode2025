mod path;

use crate::auto::path::drive;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use std::cell::{RefCell, RefMut};
use std::ops::Deref;
use std::rc::Rc;
use std::time::Duration;
use frcrs::alliance_station;
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
    TushPush1,
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
            "TushPush1" => Auto::TushPush1,
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
            Auto::TushPush1 => "TushPush1",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![
            Auto::Nothing,
            Auto::BlueTriangle,
            // Auto::Blue180,
            Auto::BlueLong,
            Auto::Blue2,
            // Auto::RotationTest,
            Auto::BlueMidLeft2,
            Auto::TushPush1,
        ]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

    pub async fn run_auto<'a>(ferris: Rc<RefCell<Ferris>>, chosen: Auto) {
        match chosen {
            Auto::Nothing => {
                println!("No auto was selected!");
            }
            Auto::BlueTriangle => {
                blue_triangle(Rc::clone(&ferris)).await.expect("Failed running auto");
            }
            Auto::Blue180 => {
                blue_180(Rc::clone(&ferris)).await.expect("Failed running auto");
            }
            Auto::BlueLong => blue_long(Rc::clone(&ferris)).await.expect("Failed running auto"),
            Auto::Blue2 => blue_2(Rc::clone(&ferris)).await.expect("Failed running auto"),
            Auto::RotationTest => rotation_test(Rc::clone(&ferris)).await.expect("Failed running auto"),
            Auto::BlueMidLeft2 => blue_mid_left_2(Rc::clone(&ferris)).await.expect("Failed running auto"),
            Auto::TushPush1 => tush_push_1(Rc::clone(&ferris)).await.expect("Failed running auto"),
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
    use_tag: Option<i32>,
) -> bool {
    elevator.set_target(elevator_position);

    join!(
        timeout(Duration::from_secs_f64(2.), async {
            loop {
                drivetrain.update_limelight().await;
                drivetrain.post_odo().await;

                if drivetrain.lineup(lineup_side, elevator_position, dt, use_tag).await {
                    break;
                }

                sleep(Duration::from_millis(20)).await;
            }
        }),
        elevator.run_to_target_trapezoid_async()
    );

    drivetrain.stop();

    let indexer_speed = match elevator_position {
        ElevatorPosition::Bottom => BOTTOM_SPEED,
        ElevatorPosition::L2 => L2_SPEED,
        ElevatorPosition::L3 => L3_SPEED,
        ElevatorPosition::L4 => L4_SPEED,
    };
    indexer.set_speed(indexer_speed);

    wait(|| !indexer.is_laser_tripped()).await;

    sleep(Duration::from_secs_f64(0.2)).await;
    indexer.stop();

    true
}

pub async fn blue_triangle(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot_ref = robot.borrow_mut();
    let mut drivetrain = robot_ref.drivetrain.deref().borrow_mut();

    drivetrain.reset_heading();

    drivetrain.odometry.set(Vector2::new(
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

pub async fn blue_180(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot_ref = robot.borrow_mut();
    let mut drivetrain = robot_ref.drivetrain.deref().borrow_mut();
    let mut elevator = robot_ref.elevator.deref().borrow_mut();
    let mut indexer = robot_ref.indexer.deref().borrow_mut();

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(7.8775811195373535),
        Length::new::<meter>(6.840074062347412),
    ));

    drive("Blue180", &mut drivetrain, 1).await?;
    println!("Blue180.1 done");

    Ok(())
}

pub async fn blue_long(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot_ref = robot.borrow_mut();
    let mut drivetrain = robot_ref.drivetrain.deref().borrow_mut();
    let mut elevator = robot_ref.elevator.deref().borrow_mut();
    let mut indexer = robot_ref.indexer.deref().borrow_mut();

    drivetrain.reset_heading_offset(
        if alliance_station().red() {
            Angle::new::<degree>(180.)
        } else {
            Angle::new::<degree>(0.)
        });

    drivetrain.odometry.set(Vector2::new(
        Length::new::<meter>(7.5),
        Length::new::<meter>(7.),
    ));

    drive("BlueLong", &mut drivetrain, 1).await?;
    println!("BlueLong.1 done");

    Ok(())
}

pub async fn blue_2(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot_ref = robot.borrow_mut();
    let mut drivetrain = robot_ref.drivetrain.deref().borrow_mut();
    let mut elevator = robot_ref.elevator.deref().borrow_mut();
    let mut indexer = robot_ref.indexer.deref().borrow_mut();

    drivetrain.reset_heading_offset(
        if alliance_station().red() {
            Angle::new::<degree>(0.)
        } else {
            Angle::new::<degree>(180.)
        });
    drivetrain.odometry.set(Vector2::new(
        Length::new::<meter>(7.215517520904541),
        Length::new::<meter>(5.439107418060303),
    ));

    join!(drive("Blue2", &mut drivetrain, 1), async {
        elevator.set_target(ElevatorPosition::L2);
        elevator.run_to_target_trapezoid();

        indexer.set_speed(INTAKE_SPEED);
        wait(|| indexer.is_laser_tripped()).await;
        indexer.stop();

        elevator.set_target(ElevatorPosition::L4);
        elevator.run_to_target_trapezoid();
    });

    let _ = timeout(Duration::from_secs_f64(0.5), async {
        loop {
            drivetrain.update_limelight().await;
            drivetrain.post_odo().await;

            sleep(Duration::from_millis(20)).await;
        }
    }).await;

    async_score(
        &mut drivetrain,
        LineupSide::Left,
        &mut elevator,
        &mut indexer,
        ElevatorPosition::L4,
        robot_ref.dt,
        if alliance_station().red() {Some(6)} else { Some(19)}
    )
    .await;

    join!(drive("Blue2", &mut drivetrain, 3), async {
        elevator.set_target(ElevatorPosition::Bottom);
        elevator.run_to_target_trapezoid_async().await;

        indexer.set_speed(INTAKE_SPEED);

        wait(|| indexer.is_laser_tripped()).await;

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
        robot_ref.dt,
        if alliance_station().red() {Some(6)} else { Some(19)}
    )
    .await;

    Ok(())
}

async fn rotation_test(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot_ref = robot.borrow_mut();
    let mut drivetrain = robot_ref.drivetrain.deref().borrow_mut();
    let mut elevator = robot_ref.elevator.deref().borrow_mut();
    let mut indexer = robot_ref.indexer.deref().borrow_mut();

    drivetrain.reset_heading();

    drivetrain.odometry.set_abs(Vector2::new(
        Length::new::<meter>(7.84511661529541),
        Length::new::<meter>(6.556429862976074),
    ));

    drive("RotationTest", &mut drivetrain, 1).await?;

    Ok(())
}

async fn blue_mid_left_2(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot_ref = robot.borrow_mut();
    let mut drivetrain = robot_ref.drivetrain.deref().borrow_mut();
    let mut elevator = robot_ref.elevator.deref().borrow_mut();
    let mut indexer = robot_ref.indexer.deref().borrow_mut();

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
        robot_ref.dt,
        None,
    ).await;

    join!(drive("BlueHighMid2", &mut drivetrain, 3), async {
        elevator.set_target(ElevatorPosition::Bottom);
        elevator.run_to_target_trapezoid_async().await;

        indexer.set_speed(INTAKE_SPEED);

        wait(|| indexer.is_laser_tripped()).await;

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
        robot_ref.dt,
        None,
    )
        .await;

    Ok(())
}

async fn tush_push_1(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let mut robot = robot.borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let mut elevator = robot.elevator.deref().borrow_mut();
    let mut indexer = robot.indexer.deref().borrow_mut();

    drivetrain.reset_heading_offset(
        if alliance_station().red() {
            Angle::new::<degree>(-90.)
        } else {
            Angle::new::<degree>(90.)
        });

    drivetrain.odometry.set(Vector2::new(
        Length::new::<meter>(7.16530704498291),
        Length::new::<meter>(4.919252395629883)
    ));

    drive("TushPush1", &mut drivetrain, 1).await?;

    join!(drive("TushPush1", &mut drivetrain, 2), async {
        elevator.set_target(ElevatorPosition::L2);
        elevator.run_to_target_trapezoid();

        indexer.set_speed(INTAKE_SPEED);
        wait(|| indexer.is_laser_tripped()).await;
        indexer.stop();

        elevator.set_target(ElevatorPosition::L4);
        elevator.run_to_target_trapezoid();
    });

    let _ = timeout(Duration::from_secs_f64(1.25), async {
        loop {
            drivetrain.update_limelight().await;
            drivetrain.post_odo().await;

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
        if alliance_station().red() {Some(10)} else { Some(21)}
    )
        .await;

    Ok(())
}