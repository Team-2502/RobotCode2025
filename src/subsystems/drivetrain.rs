use frcrs::alliance_station;

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::ops::Add;

use frcrs::ctre::{talon_encoder_tick, ControlMode, Pigeon, Talon};

use crate::constants::drivetrain::{
    SWERVE_DRIVE_IE, SWERVE_DRIVE_KP, SWERVE_ROTATIONS_TO_INCHES, SWERVE_TURN_KP,
};
use crate::constants::robotmap::swerve::*;
use crate::swerve::kinematics::{ModuleState, Swerve};
use crate::swerve::odometry::{ModuleReturn, Odometry};

use frcrs::telemetry::Telemetry;
use nalgebra::{Quaternion, Rotation2, Vector2};
use serde::Deserialize;
use serde::Serialize;

use crate::constants::vision::ROBOT_CENTER_TO_LIMELIGHT_INCHES;
use crate::subsystems::Vision;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{inch, meter};
use crate::constants;

#[derive(Default)]
pub struct DrivetrainControlState {
    pub saved_angle: Option<Angle>,
}

#[derive(Clone, Copy)]
pub enum LineupSide {
    Left,
    Right,
}
#[derive(Clone, Copy)]
pub enum SwerveControlStyle {
    RobotOriented,
    FieldOriented,
}

#[derive(Debug)]
pub struct LineupTarget {
    pub position: Vector2<f64>,
    pub angle: Angle,
}

pub struct Drivetrain {
    pigeon: Pigeon,

    fr_drive: Talon,
    fr_turn: Talon,

    fl_drive: Talon,
    fl_turn: Talon,

    bl_drive: Talon,
    bl_turn: Talon,

    br_drive: Talon,
    br_turn: Talon,

    kinematics: Swerve,
    pub odometry: Odometry,

    pub offset: Angle,

    pub limelight_lower: Vision,
    pub limelight_upper: Vision,
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
    angle: f64,
}

impl Default for Drivetrain {
    fn default() -> Self {
        Self::new()
    }
}

impl Drivetrain {
    pub fn new() -> Self {
        let fr_turn = Talon::new(FR_TURN, Some("can0".to_owned()));
        let fl_turn = Talon::new(FL_TURN, Some("can0".to_owned()));
        let bl_turn = Talon::new(BL_TURN, Some("can0".to_owned()));
        let br_turn = Talon::new(BR_TURN, Some("can0".to_owned()));

        let limelight_lower = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 11)),
            5807,
        ));
        let limelight_upper = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        let offset = if alliance_station().red() {
            Angle::new::<degree>(180.)
        } else {
            Angle::new::<degree>(0.)
        };

        Self {
            pigeon: Pigeon::new(PIGEON, Some("can0".to_owned())),

            fr_drive: Talon::new(FR_DRIVE, Some("can0".to_owned())),
            fr_turn,

            fl_drive: Talon::new(FL_DRIVE, Some("can0".to_owned())),
            fl_turn,

            bl_drive: Talon::new(BL_DRIVE, Some("can0".to_owned())),
            bl_turn,

            br_drive: Talon::new(BR_DRIVE, Some("can0".to_owned())),
            br_turn,

            kinematics: Swerve::rectangle(Length::new::<inch>(22.5), Length::new::<inch>(23.5)),
            odometry: Odometry::new(),

            offset,

            limelight_lower: limelight_lower,
            limelight_upper: limelight_upper,
        }
    }

    pub async fn update_limelight(&mut self) {
        self.limelight_lower
            .update(self.get_offset().get::<degree>() + 180.)
            .await;
        self.limelight_upper
            .update(self.get_offset().get::<degree>() + 180.)
            .await;

        let pose = self.limelight_lower.get_botpose();

        // Calculate offset from robot center to limelight
        let robot_center_to_limelight_unrotated: Vector2<Length> = Vector2::new(
            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.x),
            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.y),
        );

        // Rotate the limelight offset by drivetrain angle
        let robot_to_limelight: Vector2<Length> = Vector2::new(
            Length::new::<meter>(
                robot_center_to_limelight_unrotated.x.get::<meter>()
                    * f64::cos(self.get_offset().get::<radian>())
                    - robot_center_to_limelight_unrotated.y.get::<meter>()
                        * f64::sin(self.get_offset().get::<radian>()),
            ),
            Length::new::<meter>(
                robot_center_to_limelight_unrotated.x.get::<meter>()
                    * f64::sin(self.get_offset().get::<radian>())
                    + robot_center_to_limelight_unrotated.y.get::<meter>()
                        * f64::cos(self.get_offset().get::<radian>()),
            ),
        );

        // Will return 0, 0 if no tag found
        if pose.x.get::<meter>() != 0.0 {
            self.update_odo(pose + robot_to_limelight);
        }
        //println!("lower ll tx: {}", self.limelight_lower.get_tx().get::<degree>());
        //println!("upper ll tx: {}", self.limelight_upper.get_tx().get::<degree>());
    }

    pub async fn post_odo(&self) {
        Telemetry::put_number("odo_x", self.odometry.position.x).await;
        Telemetry::put_number("odo_y", self.odometry.position.y).await;
        Telemetry::put_number("angle", self.get_offset().get::<radian>()).await;
    }

    pub fn update_odo(&mut self, pose: Vector2<Length>) {
        self.odometry
            .set_abs(Vector2::new(pose.x.value, pose.y.value));
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
                angle: offset.angle,
                distance: Length::new::<inch>(distance),
            });
        }

        speeds
    }

    fn get_speeds(&self) -> Vec<ModuleState> {
        let mut speeds = Vec::new();

        for module in [&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn].iter() {
            speeds.push(ModuleState {
                speed: 0.,
                angle: Angle::new::<talon_encoder_tick>(-module.get_position()),
            });
        }

        speeds
    }

    fn normalize_angle(angle: f64) -> f64 {
        ((angle % 360.0) + 360.0) % 360.0
    }

    pub fn set_speeds(&mut self, fwd: f64, str: f64, rot: f64, style: SwerveControlStyle) {
        /*println!(
            "ODO XY: {}, {}",
            self.odometry.position.x, self.odometry.position.y
        );*/
        let mut transform = Vector2::new(-str, fwd);
        match style {
            SwerveControlStyle::FieldOriented => {transform = Rotation2::new(self.get_offset().get::<radian>()) * transform;;},
            SwerveControlStyle::RobotOriented => {},
        }

        let wheel_speeds = self.kinematics.calculate(transform, -rot);

        let measured = self.get_speeds();

        let positions = self.get_positions(&measured);

        let angle = self.get_offset();

        self.odometry.calculate(positions, angle);

        let wheel_speeds: Vec<ModuleState> = wheel_speeds
            .into_iter()
            .zip(measured.iter())
            .map(|(calculated, measured)| calculated.optimize(measured))
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
        if alliance_station().red() {
            Angle::new::<radian>(-self.pigeon.get_rotation().z + 180.)
        } else {
            Angle::new::<radian>(-self.pigeon.get_rotation().z)
        }
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
        self.pigeon.reset();
    }

    pub fn reset_heading(&mut self) {
        self.offset = self.get_angle();
    }

    pub async fn lineup(&mut self, side: LineupSide) {
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

            let speed_s = speed;
            last_error = speed_s;

            if alliance_station().red() {
                speed.x *= -1.
            }

            self.set_speeds(speed.x, -speed.y, error_angle, SwerveControlStyle::FieldOriented);

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
        let tag_id = self.limelight_lower.get_saved_id();
        if tag_id == -1 {
            return None;
        }

        let tag_position = self.limelight_lower.get_tag_position(tag_id)?;
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
            (tag_coords.x + Length::new::<meter>(offset_x) + Length::new::<meter>(forward_x))
                .get::<meter>(),
            (tag_coords.y + Length::new::<meter>(offset_y) + Length::new::<meter>(forward_y))
                .get::<meter>(),
        );

        let mut robot_angle =
            Angle::new::<radian>(yaw).add(Angle::new::<radian>(std::f64::consts::PI));

        robot_angle = Angle::new::<degree>(calculate_relative_target(
            self.get_offset().get::<degree>(),
            robot_angle.get::<degree>(),
        ));

        Some(LineupTarget {
            position: target_pos,
            angle: robot_angle,
        })
    }

    /// Set drivetrain speeds using tx and ty from the lower limelight.
    /// Cameras are positioned on the robot such that the tag on the base of the reef is in the exact center of the limelight's fov when the robot is fully lined up.
    /// Uses a basic PID: tx from the limelight (horizontal position of the tag on the screen) feeds into drivetrain strafe, while ty feeds into drivetrain forward.
    pub fn lineup_2d(&mut self, side: LineupSide) {
        // The lower limelight points at the tag when lined up on the right, the upper when lined up on the left
        let mut limelight = self.limelight_lower.clone();
        match side {
            LineupSide::Left => { limelight = self.limelight_upper.clone();}
            LineupSide::Right => {limelight = self.limelight_lower.clone();}
        }

        // Only try if a tag is detected
        if limelight.get_id() != -1 {
            // Figure out target angle from the tagmap
            let tag_position = limelight.get_tag_position(limelight.get_id()).unwrap();
            let tag_rotation = tag_position.quaternion.unwrap();
            // None of us actually know how the quaternions provided by said map work, this is copied code
            // Flip the tag normal to be out the back of the tag and wrap to the [0, 360] range
            let tag_yaw = (quaternion_to_yaw(tag_rotation) + std::f64::consts::PI) % (std::f64::consts::PI * 2.);
            // We score out the left, so forward-to-the-tag isn't very helpful
            let perpendicular_yaw = tag_yaw + std::f64::consts::PI / 2.0;

            // Calculate errors (difference between where you are (tx, ty, or drivetrain angle) and where you want to be (0 deg, 0 deg, or perpendicular_yaw))
            // Center of the limelight screen is (0,0) so we don't have to subtract anything for ty and tx
            let error_ty = limelight.get_ty().get::<degree>() - 2.5;
            let error_tx = limelight.get_tx().get::<degree>() - 8.;
            let error_yaw = perpendicular_yaw - self.get_offset().get::<radian>();
            println!("hi3");

            // Figure out correct direction for ks constants to apply in
            let tx_dir = if constants::drivetrain::LINEUP_2D_TX_KP * error_tx > 0. {1.} else {-1.};
            let ty_dir = if constants::drivetrain::LINEUP_2D_TY_KP * error_ty > 0. {1.} else {-1.};

            // Calculate PID stuff
            // KP - proportional: multiply the error by a tuned constant (KP)
            // KD - derivative: subtract the error from last frame by the error from this frame (this difference is roughly proportional to the rate at which the error is changing), then multiply by a tuned constant (KD)
            // KS - static: add a value to overcome static friction.
            let str =
                constants::drivetrain::LINEUP_2D_TY_KP * error_ty
                + constants::drivetrain::LINEUP_2D_TY_KS * ty_dir
                + constants::drivetrain::LINEUP_2D_TY_KD * (error_ty - limelight.get_last_results().ty);
            let fwd =
                constants::drivetrain::LINEUP_2D_TX_KP * error_tx
                + constants::drivetrain::LINEUP_2D_TX_KS * tx_dir
                + constants::drivetrain::LINEUP_2D_TX_KD * (error_tx - limelight.get_last_results().tx)
                - constants::drivetrain::LINEUP_2D_TY_KP * error_ty;
            let mut transform = Vector2::new(fwd, str);

            self.set_speeds(
                transform.x,
                transform.y,
                error_yaw * SWERVE_TURN_KP,
                SwerveControlStyle::RobotOriented
            );
        } else {println!("can't lineup - no tag seen");}
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

/// Normalize an angle to the range 0 to 360 degrees.
fn normalize_angle_360(angle: f64) -> f64 {
    let mut normalized = angle % 360.0;
    if normalized < 0.0 {
        normalized += 360.0;
    }
    normalized
}

/// Get the target angle in relation to the robot's current angle.
/// Ensures the target angle is mapped to the same rotational "circle" as the robot's angle.
fn calculate_relative_target(current: f64, target: f64) -> f64 {
    let target_relative = target + (current / 360.0).floor() * 360.0;

    // Ensure the relative target is the closest possible to the current angle
    if target_relative - current > 180.0 {
        target_relative - 360.0
    } else if target_relative - current < -180.0 {
        target_relative + 360.0
    } else {
        target_relative
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, Vector2, Vector3};

    use crate::subsystems::drivetrain::{calculate_relative_target, quaternion_to_yaw};
    use crate::subsystems::{FieldPosition, LineupSide};
    use uom::si::angle::radian;
    use uom::si::f32::Angle;
    use uom::si::f64::Length;
    use uom::si::length::meter;

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
            (tag_coords.x + Length::new::<meter>(offset_x) + Length::new::<meter>(forward_x))
                .get::<meter>(),
            (tag_coords.y + Length::new::<meter>(offset_y) + Length::new::<meter>(forward_y))
                .get::<meter>(),
        );

        let robot_angle = Angle::new::<radian>(yaw as f32);

        println!("{:?}", target_pos);
        println!("{:?}", robot_angle);

        assert!(false);
    }

    #[test]
    fn angle_in_scope() {
        let current_angle = 800.0;
        let target_angle = 250.0;

        let relative_angle = calculate_relative_target(current_angle, target_angle);
        assert_eq!(relative_angle, 970.0);
    }
}
