#![warn(non_snake_case)]

use frcrs::Robot;
use tokio::task;
use RobotCode2025::Ferris;

fn main() {
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    let mut ferris = Ferris::new();
    ferris.start_competition(runtime, local);
}
