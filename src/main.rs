#![warn(non_snake_case)]

use std::cell::RefCell;
use std::ops::Deref;
use std::process::exit;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;
use tokio::time::{Duration, Instant};
use frcrs::{init_hal, observe_user_program_starting, refresh_data, Robot};
use frcrs::input::{RobotMode, RobotState};
use frcrs::networktables::{NetworkTable, SmartDashboard};
use frcrs::telemetry::Telemetry;
use tokio::task;
use tokio::task::{AbortHandle, spawn_local};
use tokio::time::sleep;
use RobotCode2025::constants::joystick_map::{CLIMB, CLIMB_FALL, INTAKE, LINEUP_LEFT, LINEUP_RIGHT, SCORE_L2, SCORE_L3, SCORE_L4, WHEELS_ZERO};
use RobotCode2025::container::control_drivetrain;
use RobotCode2025::{constants, Ferris, score, TeleopState};
use RobotCode2025::auto::Auto;
use RobotCode2025::constants::climber::{CLIMB_SPEED, FALL_SPEED};
use RobotCode2025::constants::indexer::{INTAKE_SPEED, L3_SPEED};
use RobotCode2025::subsystems::{Climber, ElevatorPosition, LineupSide};

fn main() {
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let mut ferris = Rc::new(RefCell::new(Ferris::new()));
    // ferris.start_competition(runtime, local);

    runtime.block_on(local.run_until(async {
        if !init_hal() {
            panic!("Failed to initialize HAL");
        }

        observe_user_program_starting();

        Telemetry::init(5807);

        NetworkTable::init();

        Telemetry::put_selector("auto chooser", Auto::names()).await;

        // SmartDashboard::start_camera_server();

        let mut last_loop = Instant::now();

        let mut auto: Option<AbortHandle> = None;

        // Watchdog setup
        let last_loop_time = Arc::new(AtomicU64::new(0));
        let watchdog_last_loop = Arc::clone(&last_loop_time);
        let watchdog_ferris = ferris.clone();

        // Spawn watchdog task
        spawn_local(async move {
            loop {
                sleep(Duration::from_millis(20)).await;
                let last = watchdog_last_loop.load(Ordering::Relaxed);
                let now = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                if last != 0 && now - last > 150 {
                    println!("Loop Overrun: {}ms", now - last);
                    if let Ok(ferris) = watchdog_ferris.try_borrow_mut() {
                        ferris.stop();
                    } else {
                        println!("FAILED TO GET FERRIS TO STOP");
                        // exit(1);
                    }
                    println!("Watchdog triggered: Motors stopped");
                }
            }
        });

        loop {
            refresh_data();

            let state = RobotState::get();
            let dt = last_loop.elapsed();

            if !state.enabled() {
                // if let Some(handle) = auto.take() {
                //     println!("Aborted");
                //     handle.abort();
                // }

                if let Ok(f) = ferris.try_borrow() {
                    f.stop();
                } else {
                    println!("Didnt borrow ferris");
                }
            }

            if state.enabled() && state.teleop() {
                if let Ok(mut robot) = ferris.try_borrow_mut() {
                    robot.dt = dt;
                    teleop(&mut robot).await;
                }
            }

            if state.enabled() && state.auto() {
                // Update dt before using it in auto
                if let Ok(mut ferris_mut) = ferris.try_borrow_mut() {
                    ferris_mut.dt = dt;

                    // Now access drivetrain
                    if let Ok(mut drivetrain) = ferris_mut.drivetrain.try_borrow_mut() {
                        drivetrain.update_limelight().await;
                        drivetrain.post_odo().await;
                    }
                }

                if auto.is_none() {
                    let ferris_clone = Rc::clone(&ferris);

                    if let Some(selected_auto) = Telemetry::get_selection("auto chooser").await {
                        let chosen = Auto::from_dashboard(selected_auto.as_str());

                        let run = Auto::run_auto(ferris_clone, chosen);
                        auto = Some(local.spawn_local(run).abort_handle());
                    } else {
                        eprintln!("Failed to get selected auto from telemetry, running default");

                        let run = Auto::run_auto(ferris_clone, Auto::Nothing);
                        auto = Some(local.spawn_local(run).abort_handle());
                    }
                }
            } else if let Some(auto) = auto.take() {
                println!("Aborted");
                auto.abort();
            }

            Telemetry::put_number("Loop Rate", 1. / dt.as_secs_f64()).await;

            let now_millis = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;
            last_loop_time.store(now_millis, Ordering::Relaxed);

            let elapsed = dt.as_secs_f64();
            let left = (1. / 250. - elapsed).max(0.);
            sleep(Duration::from_secs_f64(left)).await;
            last_loop = Instant::now();
        }
    }));
}

async fn teleop(robot: &mut Ferris) {
    let TeleopState {
        ref mut drivetrain_state,
    } = *robot.teleop_state.deref().borrow_mut();

    if let Ok(mut drivetrain) = robot.drivetrain.try_borrow_mut() {
        if let Ok(mut elevator) = robot.elevator.try_borrow_mut() {
            if let Ok(mut indexer) = robot.indexer.try_borrow_mut() {
                drivetrain.update_limelight().await;
                drivetrain.post_odo().await;

                let drivetrain_aligned = if robot.controllers.right_drive.get(LINEUP_LEFT) {
                    drivetrain
                        .lineup(LineupSide::Left, elevator.get_target(), robot.dt, None)
                        .await
                } else if robot.controllers.right_drive.get(LINEUP_RIGHT) {
                    drivetrain
                        .lineup(LineupSide::Right, elevator.get_target(), robot.dt, None)
                        .await
                } else if robot.controllers.operator.get(WHEELS_ZERO) {
                    drivetrain.set_wheels_zero();
                    false
                } else {
                    control_drivetrain(
                        &mut drivetrain,
                        &mut robot.controllers,
                        drivetrain_state,
                    )
                        .await;

                    false
                };

                if robot.controllers.left_drive.get_pov() != -1 {
                    elevator.set_target(ElevatorPosition::L3Algae);
                    elevator.run_to_target_trapezoid();
                    
                    indexer.set_speed(L3_SPEED);
                } else if robot.controllers.left_drive.get(SCORE_L2) {
                    score(
                        drivetrain_aligned,
                        &mut elevator,
                        &mut indexer,
                        ElevatorPosition::L2,
                    )
                } else if robot.controllers.left_drive.get(SCORE_L3) {
                    score(
                        drivetrain_aligned,
                        &mut elevator,
                        &mut indexer,
                        ElevatorPosition::L3,
                    )
                } else if robot.controllers.left_drive.get(SCORE_L4) {
                    score(
                        drivetrain_aligned,
                        &mut elevator,
                        &mut indexer,
                        ElevatorPosition::L4,
                    )
                } else if robot.controllers.right_drive.get(INTAKE) {
                    elevator.set_target(ElevatorPosition::Bottom);
                    elevator.run_to_target_trapezoid();

                    if !indexer.is_laser_tripped()
                    {
                        indexer.set_speed(INTAKE_SPEED);
                    } else {
                        indexer.stop();
                    }
                } else if robot.controllers.left_drive.get(14) {
                    elevator.set_speed(1.);
                } else if robot.controllers.left_drive.get(15) {
                    elevator.set_speed(-1.)
                } else {
                    elevator.stop();
                    indexer.stop();
                }
            }
        }
    }

    if let Ok(climber) = robot.climber.try_borrow_mut() {
        if robot.controllers.right_drive.get(CLIMB) {
            climber.climb();
        } else if robot.controllers.right_drive.get(CLIMB_FALL) {
            climber.set(FALL_SPEED);
        } else {
            climber.set(0.);
        }
    }
}