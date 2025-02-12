mod path;

use crate::auto::path::drive;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use std::ops::Deref;
use std::time::Duration;
use tokio::time::sleep;

use crate::Ferris;

#[derive(Serialize, Deserialize)]
pub enum Auto {
    Nothing,
    BlueTriangle,
}

impl Auto {
    pub fn from_dashboard(s: &str) -> Self {
        match s {
            "Nothing" => Auto::Nothing,
            "BlueTriangle" => Auto::BlueTriangle,
            _ => Auto::Nothing,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Auto::Nothing => "Nothing",
            Auto::BlueTriangle => "BlueTriangle",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![Auto::Nothing, Auto::BlueTriangle]
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
        }
    }
}

pub async fn blue_triangle(robot: Ferris) -> Result<(), Box<dyn std::error::Error>> {
    println!("RUNNING");

    let mut drivetrain = robot.drivetrain.deref().borrow_mut();

    drivetrain
        .odometry
        .set_abs(Vector2::new(8.075126647949219, 2.0993127822875977));

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
