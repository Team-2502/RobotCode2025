mod constants;
mod container;

use std::time::{Duration, Instant};
use frcrs::{hal_report, init_hal, observe_user_program_starting, refresh_data, telemetry};
use frcrs::input::{Joystick, RobotState};
use frcrs::telemetry::Telemetry;
use tokio::task;
use tokio::time::sleep;
use RobotCode2025::{Controllers, Ferris};
use crate::constants::FPS_LIMIT;
use crate::container::teleop;

fn main() {
    let executor = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let controller = local.run_until(async {
        if !init_hal() {
            panic!("Failed to init HAL")
        }

        hal_report(2, 7, 0, "2025-FRCRS".to_string());

        let left_drive = Joystick::new(1);
        let right_drive = Joystick::new(0);
        let operator = Joystick::new(2);
        let mut controllers = Controllers {
            left_drive,
            right_drive,
            operator,
        };

        let mut robot = Ferris::new();
        observe_user_program_starting();

        let telemetry = Telemetry::new(5807);

        let mut last_loop = Instant::now();
        let mut dt = Duration::from_millis(0);
        loop {
            refresh_data();

            let state = RobotState::get();

            if state.enabled() && state.teleop() && !state.test() {
                teleop(&mut controllers, &mut robot, &local, dt.clone()).await;
            }

            dt = last_loop.elapsed();
            let elapsed = dt.as_secs_f64();
            let left = (1. / FPS_LIMIT - elapsed).max(0.);

            // TODO: Change to f64 in frcrs
            telemetry.add_number("rio load", (last_loop.elapsed().as_secs_f64() / (1./FPS_LIMIT)) as i32).await;
            sleep(Duration::from_secs_f64(left)).await;
            telemetry.add_number("loop rate (hz)", (1. / last_loop.elapsed().as_secs_f64()) as i32).await;
            last_loop = Instant::now();
        }
    });

    executor.block_on(controller);
}
