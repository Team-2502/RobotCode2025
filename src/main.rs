#![warn(non_snake_case)]

use std::ops::Deref;
use tokio::time::{Duration, Instant};
use frcrs::{init_hal, observe_user_program_starting, refresh_data, Robot};
use frcrs::input::{RobotMode, RobotState};
use frcrs::networktables::NetworkTable;
use frcrs::telemetry::Telemetry;
use tokio::task;
use tokio::task::{AbortHandle, spawn_local};
use tokio::time::sleep;
use RobotCode2025::constants::joystick_map::{CLIMB, CLIMB_FALL, CLIMBER_GRAB, CLIMBER_RAISE, INTAKE, LINEUP_LEFT, LINEUP_RIGHT, SCORE_L2, SCORE_L3, SCORE_L4, WHEELS_ZERO};
use RobotCode2025::container::control_drivetrain;
use RobotCode2025::{constants, Ferris, score, TeleopState};
use RobotCode2025::auto::Auto;
use RobotCode2025::constants::indexer::INTAKE_SPEED;
use RobotCode2025::subsystems::{Climber, ElevatorPosition, LineupSide};

fn main() {
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let mut ferris = Ferris::new();
    // ferris.start_competition(runtime, local);

    runtime.block_on(local.run_until(async {
        if !init_hal() {
            panic!("Failed to initialize HAL");
        }

        observe_user_program_starting();

        Telemetry::init(5807);

        NetworkTable::init();

        Telemetry::put_selector("auto chooser", Auto::names()).await;

        let mut last_loop = Instant::now();

        let mut auto: Option<AbortHandle> = None;

        loop {
            refresh_data();

            let state = RobotState::get();

            if !state.enabled() {
                if let Some(handle) = auto.take() {
                    println!("Aborted");
                    handle.abort();
                }
            }

            if state.enabled() && state.teleop() {
                teleop(&mut ferris).await;
            }

            if state.enabled() && state.auto() {
                if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
                    drivetrain.update_limelight().await;
                    drivetrain.post_odo().await;
                }

                if let None = auto {
                    let f = ferris.clone();

                    if let Some(selected_auto) = Telemetry::get_selection("auto chooser").await {
                        let chosen = Auto::from_dashboard(selected_auto.as_str());

                        let run = Auto::run_auto(f, chosen);
                        auto = Some(local.spawn_local(run).abort_handle());
                    } else {
                        eprintln!("Failed to get selected auto from telemetry, running default");

                        let run = Auto::run_auto(f, Auto::Nothing);
                        auto = Some(local.spawn_local(run).abort_handle());
                    }
                }
            } else if let Some(auto) = auto.take() {
                println!("Aborted");
                auto.abort();
            }

            Telemetry::put_number("Loop Rate", 1. / last_loop.elapsed().as_secs_f64()).await;

            ferris.dt = last_loop.elapsed();
            let elapsed = ferris.dt.as_secs_f64();
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
                        .lineup(LineupSide::Left, elevator.get_target(), robot.dt)
                        .await
                } else if robot.controllers.right_drive.get(LINEUP_RIGHT) {
                    drivetrain
                        .lineup(LineupSide::Right, elevator.get_target(), robot.dt)
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

                if robot.controllers.left_drive.get(SCORE_L2) {
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

                    if indexer.get_laser_dist() > constants::indexer::LASER_TRIP_DISTANCE_MM
                        || indexer.get_laser_dist() == -1
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

    if robot.controllers.right_drive.get(CLIMB) {
        if robot.climb_handle.is_none() {
            let f = robot.clone();
            let climb_task = Climber::climb(f);
            let handle = spawn_local(climb_task).abort_handle();
            robot.climb_handle = Some(handle);
        }
    } else if robot.controllers.right_drive.get(CLIMB_FALL) {
        if let Some(handle) = robot.climb_handle.take() {
            handle.abort();
        }

        if let Ok(mut climber) = robot.climber.try_borrow_mut() {
            climber.fall()
        }
    } else {
        if let Some(handle) = robot.climb_handle.take() {
            handle.abort();
        }

        if let Ok(mut climber) = robot.climber.try_borrow_mut() {
            if robot.controllers.right_drive.get(CLIMBER_RAISE) {
                climber.set_raise(true);
            } else {
                climber.set_raise(false);
            }

            if robot.controllers.right_drive.get(CLIMBER_GRAB) {
                climber.set_grab(true);
            } else {
                climber.set_grab(false);
            }
        }
    }
}