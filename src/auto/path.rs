use frcrs::telemetry::Telemetry;
use std::f64::consts::PI;
use std::ops::{Add, Neg};
use std::time::Duration;
use frcrs::alliance_station;
use frcrs::input::RobotState;
use tokio::fs::File;

use nalgebra::Vector2;
use tokio::io::AsyncReadExt;
use tokio::time::{sleep, Instant};
use uom::si::angle::{Angle, degree};
use uom::si::{
    angle::radian,
    f64::{Length, Time},
    length::{foot, meter},
    time::{millisecond, second},
    velocity::meter_per_second,
};
use wpi_trajectory::Path;

use crate::subsystems::{calculate_relative_target, SwerveControlStyle};
use crate::{
    constants::drivetrain::{
        SWERVE_DRIVE_IE, SWERVE_DRIVE_KD, SWERVE_DRIVE_KF, SWERVE_DRIVE_KFA, SWERVE_DRIVE_KI,
        SWERVE_DRIVE_KP, SWERVE_DRIVE_MAX_ERR, SWERVE_TURN_KP,
    },
    subsystems::Drivetrain,
};
use crate::constants::{HALF_FIELD_LENGTH_METERS, HALF_FIELD_WIDTH_METERS};

// TODO: Test
pub async fn drive(
    name: &str,
    drivetrain: &mut Drivetrain,
    waypoint_index: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut path_content = String::new();
    File::open(format!("/home/lvuser/deploy/choreo/{}.traj", name))
        .await?
        .read_to_string(&mut path_content)
        .await?;

    let path = Path::from_trajectory(&path_content)?;
    let waypoints = path.waypoints();

    if waypoint_index >= waypoints.len() {
        return Err("Waypoint index out of bounds".into());
    }

    // Get start and end times for the waypoint
    let start_time = if waypoint_index == 0 {
        0.0
    } else {
        waypoints[waypoint_index - 1]
    };
    let end_time = waypoints[waypoint_index];

    // Follow path for this segment
    follow_path_segment(drivetrain, path, start_time, end_time).await;
    drivetrain.set_speeds(0., 0., 0., SwerveControlStyle::FieldOriented);
    Ok(())
}

pub async fn follow_path_segment(
    drivetrain: &mut Drivetrain,
    path: Path,
    start_time: f64,
    end_time: f64,
) {
    let start = Instant::now();
    let mut last_error = Vector2::zeros();
    let mut last_loop = Instant::now();
    let mut i = Vector2::zeros();
    let red = alliance_station().red();

    loop {
        let state = RobotState::get();

        if !state.enabled() {
            break
        }

        drivetrain.post_odo().await;
        drivetrain.update_limelight().await;

        let now = Instant::now();
        let dt = now - last_loop;
        last_loop = now;

        //println!("x: {}, y: {}", drivetrain.odometry.position.x, drivetrain.odometry.position.y);

        let elapsed = start.elapsed().as_secs_f64() + start_time;

        // Exit if we've reached the end time for this segment
        if elapsed > end_time {
            break;
        }

        let setpoint = if red {
            path.get(Time::new::<second>(elapsed)).mirror(HALF_FIELD_LENGTH_METERS, HALF_FIELD_WIDTH_METERS)
        } else {
            path.get(Time::new::<second>(elapsed))
        };


        let mut angle = -setpoint.heading;
        // let mut angle_radians: f64 = setpoint.heading.get::<radian>();
        let position = Vector2::new(setpoint.x.get::<meter>(), setpoint.y.get::<meter>());

        // if drivetrain.get_offset().get::<radian>() > 0. && angle.get::<radian>() < 0. {
        //     angle += Angle::new::<degree>(360.);
        // } else if drivetrain.get_offset().get::<radian>() < 0. && angle.get::<radian>() > 0. {
        //     angle -= Angle::new::<degree>(360.);
        // }

        // if drivetrain.get_offset().get::<radian>() > 0. && angle_radians < 0. {
        //     angle_radians += (PI * 2.);
        // } else if drivetrain.get_offset().get::<radian>() < 0. && angle_radians > 0. {
        //     angle_radians -= (PI * 2.);
        // }

        angle = Angle::new::<degree>(calculate_relative_target(drivetrain.get_offset().get::<degree>(), angle.get::<degree>()));

        let mut error_position = position
            - drivetrain
                .odometry
                .robot_pose_estimate
                .get_position_meters();
        let mut error_angle = (angle - drivetrain.get_offset()).get::<radian>();

        if error_position.abs().max() < SWERVE_DRIVE_IE {
            i += error_position;
        }

        if elapsed > path.length().get::<second>()
            && error_position.abs().max() < SWERVE_DRIVE_MAX_ERR
            && error_angle.abs() < 0.075
        {
            break;
        }

        error_angle *= SWERVE_TURN_KP;
        error_position *= -SWERVE_DRIVE_KP;

        let mut speed = error_position;

        let velocity = Vector2::new(setpoint.velocity_x, setpoint.velocity_y);
        let velocity = velocity.map(|x| x.get::<meter_per_second>());

        let velocity_next = Vector2::new(setpoint.velocity_x, setpoint.velocity_y)
            .map(|x| x.get::<meter_per_second>());

        let acceleration = (velocity_next - velocity) * 1000. / 20.;

        speed += velocity * -SWERVE_DRIVE_KF;
        speed += acceleration * -SWERVE_DRIVE_KFA;
        speed += i * -SWERVE_DRIVE_KI * dt.as_secs_f64();

        let speed_s = speed;
        speed += (speed - last_error) * SWERVE_DRIVE_KD * dt.as_secs_f64();
        last_error = speed_s;

        drivetrain.set_speeds(
            speed.x,
            speed.y,
            error_angle,
            SwerveControlStyle::FieldOriented,
        );

        Telemetry::put_number("path_turn_err", error_angle).await;
        Telemetry::put_number("path_target_angle", angle.get::<radian>()).await;

        sleep(Duration::from_millis(20)).await;
    }
}
