use frcrs::alliance_station;
use std::f64::consts::PI;

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::ops::{Add, Sub};

use frcrs::ctre::{talon_encoder_tick, CanCoder, ControlMode, Pigeon, Talon};

use crate::constants::drivetrain::{
    BL_OFFSET_DEGREES, BR_OFFSET_DEGREES, FL_OFFSET_DEGREES, FR_OFFSET_DEGREES,
    LINEUP_2D_TX_FWD_KP, LINEUP_2D_TX_STR_KP, LINEUP_2D_TY_FWD_KP, PIGEON_OFFSET, SWERVE_DRIVE_IE,
    SWERVE_DRIVE_KP, SWERVE_ROTATIONS_TO_INCHES, SWERVE_TURN_KP, SWERVE_TURN_RATIO, TARGET_TX_LEFT,
    TARGET_TX_RIGHT, TARGET_TY_LEFT, TARGET_TY_RIGHT, TX_ACCEPTABLE_ERROR, TY_ACCEPTABLE_ERROR,
    YAW_ACCEPTABLE_ERROR,
};
use crate::constants::robotmap::swerve::*;
use crate::swerve::kinematics::{ModuleState, Swerve};
use crate::swerve::odometry::{ModuleReturn, Odometry};

use frcrs::telemetry::Telemetry;
use nalgebra::{Quaternion, Rotation2, Vector2};
use serde::Deserialize;
use serde::Serialize;
use tokio::time::Instant;

use crate::constants;
use crate::constants::vision::ROBOT_CENTER_TO_LIMELIGHT_UPPER_INCHES;
use crate::subsystems::{ElevatorPosition, Vision};
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{inch, meter};

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

    pub limelight: Vision,

    abs_offsets: [Angle; 4],
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

        fr_turn.zero();
        fl_turn.zero();
        bl_turn.zero();
        br_turn.zero();
        println!(
            "These should be 0: {} {} {} {}",
            fr_turn.get_position(),
            fl_turn.get_position(),
            br_turn.get_position(),
            bl_turn.get_position()
        );

        let fr_drive = Talon::new(FR_DRIVE, Some("can0".to_owned()));
        let fl_drive = Talon::new(FL_DRIVE, Some("can0".to_owned()));
        let bl_drive = Talon::new(BL_DRIVE, Some("can0".to_owned()));
        let br_drive = Talon::new(BR_DRIVE, Some("can0".to_owned()));

        let fr_encoder = CanCoder::new(FR_ENCODER, Some("can0".to_owned()));
        let fl_encoder = CanCoder::new(FL_ENCODER, Some("can0".to_owned()));
        let bl_encoder = CanCoder::new(BL_ENCODER, Some("can0".to_owned()));
        let br_encoder = CanCoder::new(BR_ENCODER, Some("can0".to_owned()));

        let limelight = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        let offset = if alliance_station().red() {
            Angle::new::<degree>(180.)
        } else {
            Angle::new::<degree>(0.)
        };

        let abs_offsets = [
            Angle::new::<degree>((-fr_encoder.get_absolute() * 360.) - FR_OFFSET_DEGREES),
            Angle::new::<degree>((-fl_encoder.get_absolute() * 360.) - FL_OFFSET_DEGREES),
            Angle::new::<degree>((-bl_encoder.get_absolute() * 360.) - BL_OFFSET_DEGREES),
            Angle::new::<degree>((-br_encoder.get_absolute() * 360.) - BR_OFFSET_DEGREES),
        ];

        Self {
            pigeon: Pigeon::new(PIGEON, Some("can0".to_owned())),

            fr_drive,
            fr_turn,
            fr_encoder,

            fl_drive,
            fl_turn,
            fl_encoder,

            bl_drive,
            bl_turn,
            bl_encoder,

            br_drive,
            br_turn,
            br_encoder,

            kinematics: Swerve::rectangle(Length::new::<inch>(21.5), Length::new::<inch>(21.5)),
            odometry: Odometry::new(),

            offset,

            limelight: limelight,

            abs_offsets,
        }
    }

    pub async fn update_limelight(&mut self) {
        self.limelight
            .update(
                self.get_offset_wrapped(),
                self.odometry.robot_pose_estimate.get_position(),
            )
            .await;
        Telemetry::put_number(
            "limelight upper fom",
            self.limelight.get_figure_of_merit().get::<meter>(),
        )
        .await;
        /*
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
            self.update_odo_absolute(pose + robot_to_limelight);
        }

         */
        //println!("lower ll tx: {}", self.limelight_lower.get_tx().get::<degree>());
        //println!("upper ll tx: {}", self.limelight_upper.get_tx().get::<degree>());
    }

    /*pub fn reset_turn_angles(&self) {
        self.fr_turn.set_position(0.);
        self.fl_turn.set_position(0.);
        self.br_turn.set_position(0.);
        self.fr_turn.set_position(0.);
    }*/

    pub fn set_wheels_zero(&mut self) {
        self.fr_turn.set(ControlMode::Position, 0.);
        self.fl_turn.set(ControlMode::Position, 0.);
        self.br_turn.set(ControlMode::Position, 0.);
        self.fr_turn.set(ControlMode::Position, 0.);
    }

    pub fn get_offsets(&self) -> [Angle; 4] {
        self.abs_offsets
    }
    pub fn get_encoder_positions(&self) -> Vec<Angle> {
        vec![
            Angle::new::<degree>((-self.fr_encoder.get_absolute() * 360.) - FR_OFFSET_DEGREES),
            Angle::new::<degree>((-self.fl_encoder.get_absolute() * 360.) - FL_OFFSET_DEGREES),
            Angle::new::<degree>((-self.bl_encoder.get_absolute() * 360.) - BL_OFFSET_DEGREES),
            Angle::new::<degree>((-self.br_encoder.get_absolute() * 360.) - BR_OFFSET_DEGREES),
        ]
    }

    pub async fn post_odo(&self) {
        Telemetry::put_number(
            "odo_x",
            self.odometry
                .robot_pose_estimate
                .get_position()
                .x
                .get::<meter>(),
        )
        .await;
        Telemetry::put_number(
            "odo_y",
            self.odometry
                .robot_pose_estimate
                .get_position()
                .y
                .get::<meter>(),
        )
        .await;
        Telemetry::put_number("angle", self.get_offset().get::<degree>()).await;
        Telemetry::put_number(
            "FOM",
            self.odometry
                .robot_pose_estimate
                .figure_of_merit
                .get::<meter>(),
        )
        .await;
    }

    pub fn update_odo_absolute(&mut self, pose: Vector2<Length>) {
        self.odometry.set_abs(pose);
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

        self.fr_drive.set(ControlMode::Percent, 0.);
        self.fr_turn.set(ControlMode::Percent, 0.);

        self.fl_drive.set(ControlMode::Percent, 0.);
        self.fl_turn.set(ControlMode::Percent, 0.);

        self.bl_drive.set(ControlMode::Percent, 0.);
        self.bl_turn.set(ControlMode::Percent, 0.);

        self.br_drive.set(ControlMode::Percent, 0.);
        self.br_turn.set(ControlMode::Percent, 0.);
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

        for (module, offset) in [&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn]
            .iter()
            .zip(self.abs_offsets)
        {
            let rev = -module.get_position() / SWERVE_TURN_RATIO;

            speeds.push(ModuleState {
                speed: 0.,
                angle: Angle::new::<revolution>(rev) + offset,
            });
        }

        speeds
    }

    pub fn print_offsets(&self) {
        let speeds = self.get_speeds();

        // println!("MOTOR: FR: {} FL: {} BL: {} BR: {}",
        //          Angle::new::<talon_encoder_tick>(-self.fr_turn.get_position()).get::<degree>(),
        //          Angle::new::<talon_encoder_tick>(-self.fl_turn.get_position()).get::<degree>(),
        //          Angle::new::<talon_encoder_tick>(-self.bl_turn.get_position()).get::<degree>(),
        //          Angle::new::<talon_encoder_tick>(-self.br_turn.get_position()).get::<degree>());
        //
        // println!("ABSOLUTE: FR: {} FL: {} BL: {} BR: {}",
        //          self.fr_encoder.get_absolute(),
        //          self.fl_encoder.get_absolute(),
        //          self.bl_encoder.get_absolute(),
        //          self.br_encoder.get_absolute());
        //
        // println!("OFFSETS: FR: {} FL: {} BL: {} BR: {}",
        //          speeds.get(0).unwrap().angle.get::<degree>(),
        //          speeds.get(1).unwrap().angle.get::<degree>(),
        //          speeds.get(2).unwrap().angle.get::<degree>(),
        //          speeds.get(3).unwrap().angle.get::<degree>());

        // println!("POSITION: FR: {} FL: {} BL: {} BR: {}",
        //          -self.fr_turn.get_position(),
        //          -self.fl_turn.get_position(),
        //          -self.bl_turn.get_position(),
        //          -self.br_turn.get_position()
        // );
    }

    fn normalize_angle(angle: f64) -> f64 {
        ((angle % 360.0) + 360.0) % 360.0
    }

    pub fn set_speeds(&mut self, fwd: f64, str: f64, rot: f64, style: SwerveControlStyle) {
        /*println!(
            "ODO XY: {}, {} ODO FOM: {}",
            self.odometry.robot_pose_estimate.get_position_meters().x,
            self.odometry.robot_pose_estimate.get_position_meters().y,
            self.odometry.robot_pose_estimate.figure_of_merit.get::<meter>()
        );*/
        let mut transform = Vector2::new(-str, fwd);
        match style {
            SwerveControlStyle::FieldOriented => {
                transform = Rotation2::new(self.get_offset_wrapped().get::<radian>()) * transform;
            }
            SwerveControlStyle::RobotOriented => {}
        }

        let wheel_speeds = self.kinematics.calculate(transform, -rot);

        let measured = self.get_speeds();

        // for  module in &measured {
        //     print!("{:.2} : ", module.angle.get::<degree>());
        // }
        // println!(" get_speeds values");
        // for encoder in self.get_encoder_positions() {
        //     print!("{:.2} : ", encoder.get::<degree>());
        // }
        // println!(" get_encoder_positions values");
        // for motor in [&self.fr_turn, &self.fl_turn, &self.bl_turn, &self.br_turn].iter() {
        //     print!("{:.2} : ", -motor.get_position()/SWERVE_TURN_RATIO);
        // }
        // println!(" motor get_position values (scaled by gear ratio");

        let positions = self.get_positions(&measured);

        let angle = self.get_offset_wrapped();

        let mut sensor_measurements = Vec::new();
        if let Some(odo_estimate) = self
            .odometry
            .calculate_arcs(positions, (angle + Angle::new::<degree>(180.)))
        {
            //println!("new odo pose estimate: x {} y {} fom {}",odo_estimate.get_position_meters().x, odo_estimate.get_position_meters().y, odo_estimate.figure_of_merit.get::<meter>());
            sensor_measurements.push(odo_estimate);
        }
        /*if let Some(limelight_lower_estimate) = self.limelight_lower.get_pose_estimate_2d() {
            sensor_measurements.push(limelight_lower_estimate);
        }

         */
        if let Some(limelight_upper_estimate) = self.limelight.get_pose_estimate_2d() {
            sensor_measurements.push(limelight_upper_estimate);
        }
        if sensor_measurements.len() != 0 {
            self.odometry.fuse_sensors_fom(sensor_measurements);
        }

        // if let Some(pose) = self.odometry.calculate(positions, (angle + Angle::new::<degree>(180.))) {
        //     self.odometry.set_abs(pose.get_position());
        // }

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
            -(wheel_speeds[0].angle.get::<revolution>() - self.abs_offsets[0].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
        self.fl_turn.set(
            ControlMode::Position,
            -(wheel_speeds[1].angle.get::<revolution>() - self.abs_offsets[1].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
        self.bl_turn.set(
            ControlMode::Position,
            -(wheel_speeds[2].angle.get::<revolution>() - self.abs_offsets[2].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
        self.br_turn.set(
            ControlMode::Position,
            -(wheel_speeds[3].angle.get::<revolution>() - self.abs_offsets[3].get::<revolution>())
                * SWERVE_TURN_RATIO,
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
        Angle::new::<radian>(-self.pigeon.get_rotation().z + PIGEON_OFFSET)
    }

    pub fn get_offset(&self) -> Angle {
        let mut difference = (self.get_angle() - self.offset).get::<degree>();

        Angle::new::<degree>(difference)
    }

    pub fn get_offset_wrapped(&self) -> Angle {
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

    pub fn reset_heading_offset(&mut self, offset: Angle) {
        self.offset = self.get_angle() + offset;
    }

    pub async fn lineup(&mut self, side: LineupSide, target_level: ElevatorPosition) -> bool {
        let mut last_error = Vector2::zeros();
        let mut i = Vector2::zeros();

        if let Some(target) = self.calculate_target_lineup_position(side, target_level) {
            let mut error_position =
                target.position - self.odometry.robot_pose_estimate.get_position_meters();

            let mut dt_angle = (self.get_angle() - self.offset).get::<degree>();

            dt_angle = (dt_angle + 180.) % 360. - 180.;
            if dt_angle < -180. {
                dt_angle += 360.
            };

            let dt_angle = Angle::new::<degree>(dt_angle);

            let mut error_angle = (-target.angle - dt_angle).get::<radian>();

            if error_position.abs().max() < SWERVE_DRIVE_IE {
                i += error_position;
            }

            error_angle *= SWERVE_TURN_KP * 1.2;
            error_position *= -SWERVE_DRIVE_KP * 1.65;

            let mut speed = error_position;

            let speed_s = speed;
            last_error = speed_s;

            if alliance_station().red() {
                speed.x *= -1.
            }

            Telemetry::put_number("error_position_x", error_position.x).await;
            Telemetry::put_number("error_position_y", error_position.y).await;
            Telemetry::put_number("error_angle", error_angle).await;

            Telemetry::put_number("speed_x", speed.x).await;
            Telemetry::put_number("speed_y", -speed.y).await;

            Telemetry::put_number("target_x", target.position.x).await;
            Telemetry::put_number("target_y", target.position.y).await;
            Telemetry::put_number("target_angle", target.angle.get::<radian>()).await;
            if error_position.magnitude().abs() < 0.0125 && error_angle.abs() < 0.015 {
                self.stop();
                //self.set_speeds(0., 0., 0., SwerveControlStyle::RobotOriented);
                // println!("dt at position");
                true
            } else {
                // println!("dt not at position");
                self.set_speeds(
                    speed.x,
                    speed.y,
                    error_angle,
                    SwerveControlStyle::FieldOriented,
                );
                false
            }
        } else {
            false
        }
    }

    // Find the position to line up to based on which scoring side we want.
    // It will return a vector with the x and y coordinates of the target position.
    // The target position will be to the side of the apriltag, half a robot length away from the edge
    // Will account for the robot's orientation with the hexagon lineup
    pub fn calculate_target_lineup_position(
        &mut self,
        side: LineupSide,
        target_level: ElevatorPosition,
    ) -> Option<LineupTarget> {
        let tag_id = self.limelight.get_saved_id();
        if tag_id == -1 {
            return None;
        }

        let tag_position = self.limelight.get_tag_position(tag_id)?;
        let tag_coords = tag_position.coordinate?;
        let tag_rotation = tag_position.quaternion?;

        let yaw = quaternion_to_yaw(tag_rotation);

        let mut side_distance = Length::new::<inch>(13. / 2.); // theoretical is 13. / 2.
        let forward_distance = Length::new::<inch>(16.25); //theoretical is 16.75
        let elevator_position = match target_level {
            ElevatorPosition::Bottom => Length::new::<inch>(-10.5),
            ElevatorPosition::L2 => Length::new::<inch>(-10.5),
            ElevatorPosition::L3 => Length::new::<inch>(-10.5),
            ElevatorPosition::L4 => Length::new::<inch>(-9.),
        }; //theoretical is -11.0

        let side_multiplier = match side {
            LineupSide::Left => -1.0,
            LineupSide::Right => 1.0,
        };
        side_distance *= side_multiplier;
        side_distance += elevator_position;

        let perpendicular_yaw = yaw + std::f64::consts::PI / 2.0;

        let offset_x = side_distance.get::<meter>() * f64::cos(perpendicular_yaw);
        let offset_y = side_distance.get::<meter>() * f64::sin(perpendicular_yaw);

        let forward_x = forward_distance.get::<meter>() * f64::cos(yaw);
        let forward_y = forward_distance.get::<meter>() * f64::sin(yaw);

        let target_pos = Vector2::new(
            (tag_coords.x + Length::new::<meter>(offset_x) + Length::new::<meter>(forward_x))
                .get::<meter>(),
            (tag_coords.y + Length::new::<meter>(offset_y) + Length::new::<meter>(forward_y))
                .get::<meter>(),
        );

        let mut robot_angle = Angle::new::<radian>(yaw)
            .add(Angle::new::<radian>(PI))
            .sub(Angle::new::<radian>(PI / 2.));

        robot_angle = Angle::new::<degree>(calculate_relative_target(
            self.get_offset_wrapped().get::<degree>(),
            robot_angle.get::<degree>(),
        ));
        if robot_angle.get::<degree>() > 180. {
            robot_angle -= Angle::new::<degree>(360.)
        } else if robot_angle.get::<degree>() < -180. {
            robot_angle += Angle::new::<degree>(360.)
        }

        Some(LineupTarget {
            position: target_pos,
            angle: robot_angle,
        })
    }

    // Set drivetrain speeds using tx and ty from the lower limelight.
    // Cameras are positioned on the robot such that the tag on the base of the reef is in the exact center of the limelight's fov when the robot is fully lined up.
    // Uses a basic PID: tx from the limelight (horizontal position of the tag on the screen) feeds into drivetrain strafe, while ty feeds into drivetrain forward.
    // pub fn lineup_2d(&mut self, side: LineupSide) -> bool {
    //     // The lower limelight points at the tag when lined up on the right, the upper when lined up on the left
    //     let mut limelight = self.limelight_lower.clone();
    //     match side {
    //         LineupSide::Left => {
    //             limelight = self.limelight.clone();
    //         }
    //         LineupSide::Right => {
    //             limelight = self.limelight_lower.clone();
    //         }
    //     }
    //
    //     // Only try if a tag is detected
    //     if limelight.get_id() != -1 {
    //         // Figure out target angle from the tagmap
    //         let tag_position = limelight.get_tag_position(limelight.get_id()).unwrap();
    //         let tag_rotation = tag_position.quaternion.unwrap();
    //         // None of us actually know how the quaternions provided by said map work, this is copied code
    //         // Flip the tag normal to be out the back of the tag and wrap to the [0, 360] range
    //         let tag_yaw = -(quaternion_to_yaw(tag_rotation) + std::f64::consts::PI);
    //         // % (std::f64::consts::PI * 2.);
    //         // We score out the left, so forward-to-the-tag isn't very helpful
    //         let mut perpendicular_yaw = tag_yaw + std::f64::consts::PI / 2.0;
    //         // Convert to angle on [0,180]
    //         if perpendicular_yaw > std::f64::consts::PI {
    //             perpendicular_yaw = perpendicular_yaw - (std::f64::consts::PI * 2.);
    //         } else if perpendicular_yaw < PI {
    //             perpendicular_yaw = perpendicular_yaw + (PI * 2.);
    //         }
    //
    //         let target_ty = match side {
    //             LineupSide::Left => TARGET_TY_LEFT,
    //             LineupSide::Right => TARGET_TY_RIGHT,
    //         };
    //         let target_tx = match side {
    //             LineupSide::Left => TARGET_TX_LEFT,
    //             LineupSide::Right => TARGET_TX_RIGHT,
    //         };
    //
    //         // Calculate errors (difference between where you are (tx, ty, or drivetrain angle) and where you want to be (0 deg, 0 deg, or perpendicular_yaw))
    //         // Center of the limelight screen is (0,0) so we don't have to subtract anything for ty and tx
    //         let error_ty = target_ty - limelight.get_ty().get::<degree>();
    //         let error_tx = target_tx - limelight.get_tx().get::<degree>();
    //         let error_yaw = perpendicular_yaw - self.get_offset().get::<radian>();
    //
    //         // Neither limelight faces perfectly straight out.
    //         let off_straight_multiplier_str = match side {
    //             LineupSide::Left => -1.0,
    //             LineupSide::Right => 1.0,
    //         };
    //         let off_straight_multiplier_fwd = match side {
    //             LineupSide::Left => 1.0,
    //             LineupSide::Right => -1.0,
    //         };
    //
    //         // Calculate PID stuff
    //         // KP - proportional: multiply the error by a tuned constant (KP)
    //         let str = constants::drivetrain::LINEUP_2D_TY_STR_KP * error_ty
    //             + off_straight_multiplier_str * LINEUP_2D_TX_STR_KP * error_tx;
    //
    //         let fwd = LINEUP_2D_TX_FWD_KP * error_tx
    //             + off_straight_multiplier_fwd * LINEUP_2D_TY_FWD_KP * error_yaw;
    //
    //         let mut transform = Vector2::new(fwd, str);
    //
    //         return if error_ty.abs() < TX_ACCEPTABLE_ERROR
    //             && error_tx.abs() < TY_ACCEPTABLE_ERROR
    //             && error_yaw.abs() < YAW_ACCEPTABLE_ERROR
    //         {
    //             println!("Drivetrain at target");
    //
    //             self.stop();
    //
    //             true
    //         } else {
    //             println!(
    //                 "Drivetrain not at target ty: {} tx: {} yaw: {}",
    //                 error_ty.abs(),
    //                 error_tx.abs(),
    //                 error_yaw.abs()
    //             );
    //
    //             self.set_speeds(
    //                 transform.x,
    //                 transform.y,
    //                 error_yaw * SWERVE_TURN_KP,
    //                 SwerveControlStyle::RobotOriented,
    //             );
    //
    //             false
    //         };
    //     } else {
    //         println!("can't lineup - no tag seen");
    //         false
    //     }
    // }
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
pub fn calculate_relative_target(current: f64, target: f64) -> f64 {
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
    use std::f64::consts::PI;

    use crate::subsystems::drivetrain::{calculate_relative_target, quaternion_to_yaw};
    use crate::subsystems::{FieldPosition, LineupSide};
    use uom::si::angle::radian;
    use uom::si::f32::Angle;
    use uom::si::f64::Length;
    use uom::si::length::meter;

    #[test]
    #[ignore]
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

    #[test]
    fn quaternion_tag_19() {
        let quaternion = Quaternion::new(0.5000000000000001, 0., 0., 0.8660254037844386);

        let yaw = -quaternion_to_yaw(quaternion);

        println!("{}", yaw);

        assert_eq!(yaw, -(2. * PI) / 3.)
    }
}
