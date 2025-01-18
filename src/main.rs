use std::time::{Duration, Instant};
use frcrs::{hal_report, init_hal, observe_user_program_starting, refresh_data, telemetry};
use frcrs::input::{Joystick, RobotState};
use frcrs::networktables::{NetworkTable, SmartDashboard};
use frcrs::telemetry::Telemetry;
use tokio::task;
use tokio::time::sleep;
use RobotCode2025::{Controllers, Ferris};
use RobotCode2025::constants::FPS_LIMIT;
use RobotCode2025::container::teleop;

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

        Telemetry::init(5807);

        NetworkTable::init();

        SmartDashboard::put_field();

        observe_user_program_starting();

        let mut last_loop = Instant::now();
        let mut dt = Duration::from_millis(0);
        loop {
            refresh_data();

            let state = RobotState::get();

            if state.enabled() && state.teleop() {
                teleop(&mut controllers, &mut robot, &local, dt.clone()).await;
            }

            dt = last_loop.elapsed();
            let elapsed = dt.as_secs_f64();
            let left = (1. / FPS_LIMIT - elapsed).max(0.);

            Telemetry::put_number("rio load", last_loop.elapsed().as_secs_f64() / (1./FPS_LIMIT)).await;
            sleep(Duration::from_secs_f64(left)).await;
            Telemetry::put_number("loop rate (hz)", 1. / last_loop.elapsed().as_secs_f64()).await;
            last_loop = Instant::now();
        }
    });

    executor.block_on(controller);
}
