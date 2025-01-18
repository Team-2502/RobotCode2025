use std::fs::File;
use std::io::{Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::ops::Add;
use std::time::{Duration, Instant};
use frcrs::{alliance_station, AllianceStation, telemetry};

use frcrs::ctre::{talon_encoder_tick, CanCoder, ControlMode, Talon};

use crate::constants::drivetrain::{SWERVE_DRIVE_IE, SWERVE_DRIVE_KD, SWERVE_DRIVE_KF, SWERVE_DRIVE_KFA, SWERVE_DRIVE_KI, SWERVE_DRIVE_KP, SWERVE_ROTATIONS_TO_INCHES, SWERVE_TURN_KP};
use crate::constants::*;
use crate::swerve::kinematics::{ModuleState, Swerve};
use crate::swerve::odometry::{ModuleReturn, Odometry};
use frcrs::navx::NavX;
use frcrs::telemetry::Telemetry;
use nalgebra::{Quaternion, Rotation2, Vector2};
use serde::Deserialize;
use serde::Serialize;
use tokio::time::sleep;
use uom::num_traits::FloatConst;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{inch, meter};
use uom::si::time::Time;
use uom::si::velocity::meter_per_second;
use wpi_trajectory::Path;
use crate::constants::vision::ROBOT_CENTER_TO_LIMELIGHT_INCHES;
use crate::subsystems::Vision;

#[derive(Default)]
pub struct DrivetrainControlState {
    pub saved_angle: Option<Angle>,
}

pub enum LineupSide {
    Left,
    Right
}

#[derive(Debug)]
pub struct LineupTarget {
    pub position: Vector2<f64>,
    pub angle: Angle,
}

pub struct Drivetrain {
    navx: NavX,

    fr_drive: Talon,
    fr_turn: Talon,
    fr_encoder: CanCoder,

    fl_drive: Talon,
    fl_turn: Talon,
    fl_encoder: CanCoder,

    bl_drive: Talon,
    bl_turn: Talon,
    bl_encoder: CanCoder,

    br_drive: Talon,
    br_turn: Talon,
    br_encoder: CanCoder,

    kinematics: Swerve,
    pub odometry: Odometry,

    pub offset: Angle,

    absolute_offsets: Offsets,

    pub vision: Vision,
}

#[derive(Serialize, Deserialize)]
struct Offsets {
    offsets: [f64; 4],
}

#[derive(Debug, PartialEq)]
struct Point {
    x: f64,
    y: f64,
    distance: f64,
    angle: f64
}

impl Offsets {
    const PATH: &'static str = "/home/lvuser/absolut_homosezual.json";
    fn load() -> Self {
        let mut file = File::open(Self::PATH).unwrap();
        let mut buf = String::new();
        file.read_to_string(&mut buf).unwrap();
        serde_json::from_str(&buf).unwrap_or(Self { offsets: [0.; 4] })
    }
    fn store(&self) {
        let mut file = File::create(Self::PATH).unwrap();
        let buf = serde_json::to_string(&self).unwrap();
        file.write_all(buf.as_bytes()).unwrap();
    }
}

impl Drivetrain {
    pub fn new() -> Self {
        let mut absolute_offsets = Offsets::load();
        let fr_encoder = CanCoder::new(FR_ENCODER, Some("can0".to_owned()));
        let fl_encoder = CanCoder::new(FL_ENCODER, Some("can0".to_owned()));
        let bl_encoder = CanCoder::new(BL_ENCODER, Some("can0".to_owned()));
        let br_encoder = CanCoder::new(BR_ENCODER, Some("can0".to_owned()));

        let fr_turn = Talon::new(FR_TURN, Some("can0".to_owned()));
        let fl_turn = Talon::new(FL_TURN, Some("can0".to_owned()));
        let bl_turn = Talon::new(BL_TURN, Some("can0".to_owned()));
        let br_turn = Talon::new(BR_TURN, Some("can0".to_owned()));

        let vision = Vision::new(SocketAddr::new(IpAddr::V4(Ipv4Addr::new(10, 25, 2, 11)), 5807));

        for (encoder, offset) in [&fr_encoder, &fl_encoder, &bl_encoder, &br_encoder]
            .iter()
            .zip(absolute_offsets.offsets.iter_mut())
        {
            *offset -= encoder.get_absolute();
        }

        for (turn, offset) in [&fr_turn, &fl_turn, &bl_turn, &br_turn]
            .iter()
            .zip(absolute_offsets.offsets.iter_mut())
        {
            *offset -= turn.get_position();
            *offset = 0.;
            dbg!(offset);
        }

        let dt = Self {
            navx: NavX::new(),

            fr_drive: Talon::new(FR_DRIVE, Some("can0".to_owned())),
            fr_turn,
            fr_encoder,

            fl_drive: Talon::new(FL_DRIVE, Some("can0".to_owned())),
            fl_turn,
            fl_encoder,

            bl_drive: Talon::new(BL_DRIVE, Some("can0".to_owned())),
            bl_turn,
            bl_encoder,

            br_drive: Talon::new(BR_DRIVE, Some("can0".to_owned())),
            br_turn,
            br_encoder,

            kinematics: Swerve::rectangle(Length::new::<inch>(22.5), Length::new::<inch>(23.5)),
            odometry: Odometry::new(),

            offset: Angle::new::<degree>(0.),

            absolute_offsets,

            vision,
        };

        dt
    }

    pub async fn update_limelight(&mut self) {
        self.vision.update(self.get_offset().get::<degree>() + 180.).await;

        let pose = self.vision.get_botpose();

        // Calculate offset from robot center to limelight
        let robot_center_to_limelight_unrotated: Vector2<Length> = Vector2::new(
            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.x),
            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.y)
        );

        // Rotate the limelight offset by drivetrain angle
        let robot_to_limelight: Vector2<Length> = Vector2::new(
            Length::new::<meter>(
                robot_center_to_limelight_unrotated.x.get::<meter>() * f64::cos(self.get_offset().get::<radian>()) -
                    robot_center_to_limelight_unrotated.y.get::<meter>() * f64::sin(self.get_offset().get::<radian>())
            ),
            Length::new::<meter>(
                robot_center_to_limelight_unrotated.x.get::<meter>() * f64::sin(self.get_offset().get::<radian>()) +
                    robot_center_to_limelight_unrotated.y.get::<meter>() * f64::cos(self.get_offset().get::<radian>())
            )
        );

        // Will return 0, 0 if no tag found
        if pose.x.get::<meter>() != 0.0 {
            self.update_odo(pose + robot_to_limelight);
        }
    }

    pub async fn post_odo(&self) {
        Telemetry::put_number("odo_x", self.odometry.position.x).await;
        Telemetry::put_number("odo_y", self.odometry.position.y).await;
        Telemetry::put_number("angle", self.get_offset().get::<radian>()).await;
    }

    pub fn update_odo(&mut self, pose: Vector2<Length>) {
        self.odometry.set_abs(Vector2::new(pose.x.value, pose.y.value));
    }

    pub fn write_absolute(&mut self) {
        let mut offsets = Offsets::load();
        for (encoder, offset) in [
            &self.fr_encoder,
            &self.fl_encoder,
            &self.bl_encoder,
            &self.br_encoder,
        ]
            .iter()
            .zip(offsets.offsets.iter_mut())
        {
            *offset = encoder.get_absolute();
        }

        offsets.store();
    }

    pub fn stop(&self) {
        self.fr_drive.stop();
        self.fr_turn.stop();

        self.fl_drive.stop();
        self.fl_turn.stop();

        self.bl_drive.stop();
        self.bl_turn.stop();

        self.br_drive.stop();
        self.br_turn.stop();
    }

    fn get_positions(&self, angles: &Vec<ModuleState>) -> Vec<ModuleReturn> {
        let mut speeds = Vec::new();

        for (module, offset) in [
            &self.fr_drive,
            &self.fl_drive,
            &self.bl_drive,
            &self.br_drive,
        ]
            .iter()
            .zip(angles.iter())
        {
            let distance = module.get_position() * SWERVE_ROTATIONS_TO_INCHES;
            speeds.push(ModuleReturn {
                angle: offset.angle.clone(),
                distance: Length::new::<inch>(distance),
            });
        }

        speeds
    }

    fn get_speeds(&self) -> Vec<ModuleState> {
        let mut speeds = Vec::new();

        for (module, offset) in [&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn]
            .iter()
            .zip(self.absolute_offsets.offsets.iter())
        {
            speeds.push(ModuleState {
                speed: 0.,
                angle: Angle::new::<talon_encoder_tick>(-module.get_position())
                    + Angle::new::<degree>(*offset),
            });
        }

        speeds
    }
    pub fn set_speeds(&mut self, fwd: f64, str: f64, rot: f64) {
        //println!("ODO X: {}", self.odometry.position.x);
        let mut transform = Vector2::new(-str, fwd);
        transform = Rotation2::new(self.get_offset().get::<radian>()) * transform;
        let wheel_speeds = self.kinematics.calculate(transform, rot);

        //self.fr_turn.set(control_mode, amount)

        //self.fr_turn.set(ControlMode::Position, (0.).talon_encoder_ticks());

        let measured = self.get_speeds();

        let positions = self.get_positions(&measured);

        let angle = self.get_offset();

        self.odometry.calculate(positions, angle);

        //println!("angle fr {}", measured[0].angle.get::<revolution>());

        let wheel_speeds: Vec<ModuleState> = wheel_speeds
            .into_iter()
            .zip(measured.iter())
            .map(|(calculated, measured)| calculated.optimize(measured))
            .zip(self.absolute_offsets.offsets.iter())
            .map(|(mut state, offset)| {
                state.angle -= Angle::new::<degree>(*offset);
                state
            })
            .collect();

        self.fr_drive
            .set(ControlMode::Percent, wheel_speeds[0].speed);
        self.fl_drive
            .set(ControlMode::Percent, wheel_speeds[1].speed);
        self.bl_drive
            .set(ControlMode::Percent, wheel_speeds[2].speed);
        self.br_drive
            .set(ControlMode::Percent, wheel_speeds[3].speed);

        self.fr_turn.set(
            ControlMode::Position,
            -wheel_speeds[0].angle.get::<talon_encoder_tick>(),
        );
        self.fl_turn.set(
            ControlMode::Position,
            -wheel_speeds[1].angle.get::<talon_encoder_tick>(),
        );
        self.bl_turn.set(
            ControlMode::Position,
            -wheel_speeds[2].angle.get::<talon_encoder_tick>(),
        );
        self.br_turn.set(
            ControlMode::Position,
            -wheel_speeds[3].angle.get::<talon_encoder_tick>(),
        );
    }

    pub fn dbg_set(&self, angle: f64) {
        println!(
            "front right {}",
            (Angle::new::<talon_encoder_tick>(-self.fr_turn.get_position())).get::<revolution>()
        );
        println!("front right setting {}", angle);
        let angle = Angle::new::<revolution>(angle);
        self.fr_turn
            .set(ControlMode::Position, angle.get::<talon_encoder_tick>());
        //self.fl_turn.set(ControlMode::Position, angle.get::<talon_encoder_tick>());
        //self.bl_turn.set(ControlMode::Position, angle.get::<talon_encoder_tick>());
        //self.br_turn.set(ControlMode::Position, angle.get::<talon_encoder_tick>());
    }

    pub fn zero_wheels(&self) {
        let measured = self.get_speeds();

        for (module, motor) in measured
            .into_iter()
            .zip([&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn].into_iter())
        {
            let remainder = module.angle % Angle::new::<degree>(360.);
            let angle = module.angle - remainder;
            motor.set(ControlMode::Position, -angle.get::<talon_encoder_tick>());
        }
    }

    pub fn get_angle(&self) -> Angle {
        Angle::new::<degree>(-self.navx.get_angle())
    }

    pub fn get_offset(&self) -> Angle {
        let mut difference = (self.get_angle() - self.offset).get::<degree>();

        difference = (difference + 180.) % 360. - 180.;
        if difference < -180. {
            difference += 360.
        };

        Angle::new::<degree>(difference)
    }

    pub fn reset_angle(&self) {
        self.navx.reset_angle()
    }

    pub fn reset_heading(&mut self) {
        self.offset = self.get_angle();
    }

    pub async fn lineup(&mut self, side: LineupSide, dt: Duration) {
        let mut last_error = Vector2::zeros();
        let mut i = Vector2::zeros();

        if let Some(target) = self.calculate_target_lineup_position(side) {
            let mut error_position = target.position - self.odometry.position;
            let mut error_angle = (target.angle - self.get_offset()).get::<radian>();

            if error_position.abs().max() < SWERVE_DRIVE_IE {
                i += error_position;
            }

            error_angle *= SWERVE_TURN_KP;
            error_position *= -SWERVE_DRIVE_KP;

            let mut speed = error_position;
            speed += i * -SWERVE_DRIVE_KI * dt.as_secs_f64() * 9.;

            let speed_s = speed;
            speed += (speed - last_error) * -SWERVE_DRIVE_KD * dt.as_secs_f64() * 9.;
            last_error = speed_s;

            if (alliance_station().red()) { speed.x *= -1. }

            self.set_speeds(speed.x, -speed.y, error_angle);

            // sleep(Duration::from_millis(20)).await;

            Telemetry::put_number("error_position_x", error_position.x).await;
            Telemetry::put_number("error_position_y", error_position.y).await;
            Telemetry::put_number("error_angle", error_angle).await;

            Telemetry::put_number("speed_x", speed.x).await;
            Telemetry::put_number("speed_y", -speed.y).await;

            Telemetry::put_number("target_x", target.position.x).await;
            Telemetry::put_number("target_y", target.position.y).await;
            Telemetry::put_number("target_angle", target.angle.get::<radian>()).await;
        }
    }

    // Find the position to line up to based on which scoring side we want.
    // It will return a vector with the x and y coordinates of the target position.
    // The target position will be to the side of the apriltag, half a robot length away from the edge
    // Will account for the robot's orientation with the hexagon lineup
    pub fn calculate_target_lineup_position(&mut self, side: LineupSide) -> Option<LineupTarget> {
        let tag_id = self.vision.get_saved_id();
        if tag_id == -1 {
            return None;
        }

        let tag_position = self.vision.get_tag_position(tag_id)?;
        let tag_coords = tag_position.coordinate?;
        let tag_rotation = tag_position.quaternion?;

        let yaw = quaternion_to_yaw(tag_rotation);

        let side_distance = Length::new::<meter>(0.5);
        let forward_distance = Length::new::<meter>(0.5);

        let side_multiplier = match side {
            LineupSide::Left => -1.0,
            LineupSide::Right => 1.0,
        };

        let perpendicular_yaw = yaw + std::f64::consts::PI / 2.0;

        let offset_x = side_distance.get::<meter>() * f64::cos(perpendicular_yaw) * side_multiplier;
        let offset_y = side_distance.get::<meter>() * f64::sin(perpendicular_yaw) * side_multiplier;

        let forward_x = forward_distance.get::<meter>() * f64::cos(yaw);
        let forward_y = forward_distance.get::<meter>() * f64::sin(yaw);

        let target_pos = Vector2::new(
            (tag_coords.x + Length::new::<meter>(offset_x) + Length::new::<meter>(forward_x)).get::<meter>(),
            (tag_coords.y + Length::new::<meter>(offset_y) + Length::new::<meter>(forward_y)).get::<meter>(),
        );

        let robot_angle = Angle::new::<radian>(yaw).add(Angle::new::<radian>(std::f64::consts::PI));

        Some(LineupTarget {
            position: target_pos,
            angle: robot_angle,
        })
    }
}

fn quaternion_to_yaw(quaternion: Quaternion<f64>) -> f64 {
    let w: f64 = quaternion.w;
    let i: f64 = quaternion.i;
    let j: f64 = quaternion.j;
    let k: f64 = quaternion.k;

    let yaw = f64::atan2(2.0 * (w * k + i * j), 1.0 - 2.0 * (j * j + k * k));

    yaw.rem_euclid(2.0 * std::f64::consts::PI)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, Vector2, Vector3};
    use uom::ConversionFactor;
    use uom::si::angle::radian;
    use uom::si::f32::Angle;
    use uom::si::f64::Length;
    use uom::si::length::{inch, meter};
    use crate::subsystems::{FieldPosition, LineupSide, LineupTarget};
    use crate::subsystems::drivetrain::quaternion_to_yaw;

    #[test]
    fn calculate_target_lineup_position() {
        let side = LineupSide::Right;

        let tag_position = FieldPosition {
            coordinate: Some(Vector3::new(
                Length::new::<meter>(13.474446),
                Length::new::<meter>(3.3063179999999996),
                Length::new::<meter>(0.308102),
            )),
            quaternion: Some(Quaternion::new(
                -0.8660254037844387,
                -0.0,
                0.0,
                0.49999999999999994,
            )),
        };

        let tag_coords = tag_position.coordinate.unwrap();
        let tag_rotation = tag_position.quaternion.unwrap();

        let yaw = quaternion_to_yaw(tag_rotation);

        let side_distance = Length::new::<meter>(0.5);
        let forward_distance = Length::new::<meter>(0.5);

        let side_multiplier = match side {
            LineupSide::Left => -1.0,
            LineupSide::Right => 1.0,
        };

        let perpendicular_yaw = yaw + std::f64::consts::PI / 2.0;

        let offset_x = side_distance.get::<meter>() * f64::cos(perpendicular_yaw) * side_multiplier;
        let offset_y = side_distance.get::<meter>() * f64::sin(perpendicular_yaw) * side_multiplier;

        let forward_x = forward_distance.get::<meter>() * f64::cos(yaw);
        let forward_y = forward_distance.get::<meter>() * f64::sin(yaw);

        let target_pos = Vector2::new(
            (tag_coords.x + Length::new::<meter>(offset_x) + Length::new::<meter>(forward_x)).get::<meter>(),
            (tag_coords.y + Length::new::<meter>(offset_y) + Length::new::<meter>(forward_y)).get::<meter>(),
        );

        let robot_angle = Angle::new::<radian>(yaw as f32);

        println!("{:?}", target_pos);
        println!("{:?}", robot_angle);

        assert!(false);
    }
}