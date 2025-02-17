use frcrs::limelight::{Limelight, LimelightResults};
use std::fs::File;

use crate::constants::vision;
use crate::constants::vision::ROBOT_CENTER_TO_LIMELIGHT_UPPER_INCHES;
use frcrs::telemetry::Telemetry;
use nalgebra::{Quaternion, Rotation2, Vector2, Vector3};
use serde_json::Value;
use uom::num::FromPrimitive;
use uom::si::length::inch;
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::meter,
};

use std::net::SocketAddr;
use tokio::time::Instant;
use crate::constants::pose_estimation::{LIMELIGHT_BASE_FOM, LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY, LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY};
use crate::swerve::odometry::PoseEstimate;

#[derive(Clone)]
pub struct Vision {
    tag_map_values: Value,
    limelight: Limelight,
    results: LimelightResults,
    last_results: LimelightResults,
    saved_id: i32,
    drivetrain_angle: Angle,
    last_drivetrain_angle: Angle,
    last_update_time: Instant,
    robot_position: Vector2<Length>,
    last_robot_position: Vector2<Length>,
}

pub struct FieldPosition {
    pub coordinate: Option<Vector3<Length>>,
    pub quaternion: Option<Quaternion<f64>>,
}

impl Vision {
    /// Creates new Vision struct from a limelight name (e.g., "limelight-ferris")
    /// Requires that wpilib's tagmap for the year has been put on the rio at "home/lvuser/tagmap.json".
    /// Remember to run NetworkTable::init from frcrs
    pub fn new(addr: SocketAddr) -> Self {
        let tagmap = File::open("/home/lvuser/tagmap.json").unwrap();
        let tag_values: Value = serde_json::from_reader(tagmap).unwrap();
        let limelight = Limelight::new(addr);

        Self {
            tag_map_values: tag_values,
            limelight,
            results: LimelightResults::default(),
            last_results: LimelightResults::default(),
            saved_id: 0,
            drivetrain_angle: Angle::new::<degree>(0.),
            last_drivetrain_angle: Angle::new::<degree>(0.),
            last_update_time: Instant::now(),
            robot_position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
            last_robot_position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
        }
    }
    /// Updates the results from the limelight, also posts telemetry data
    pub async fn update(&mut self, dt_angle: Angle, robot_position: Vector2<Length>) {
        self.last_results = self.results.clone();
        self.last_robot_position = self.robot_position;
        self.robot_position = robot_position;
        self.results = self.limelight.results().await.unwrap();
        self.last_drivetrain_angle = self.drivetrain_angle;
        self.drivetrain_angle = dt_angle;
        self.last_update_time = Instant::now();
        self.limelight
            .update_robot_orientation(-dt_angle.get::<radian>()) // Why do we use clockwise positive
            .await
            .unwrap();

        if !self.results.Fiducial.is_empty() {
            if self.results.Fiducial[0].fID != -1 && self.results.Fiducial[0].fID != self.saved_id {
                self.saved_id = self.results.Fiducial[0].fID;
            }

            Telemetry::put_number("id", self.results.Fiducial[0].fID as f64).await;
        }
        if let Some(dist) = self.get_dist() {
            Telemetry::put_number("dist from tag inches", dist.get::<inch>()).await;

        }
        Telemetry::put_number("tx", self.results.tx).await;
        Telemetry::put_number("ty", self.results.ty).await;
        Telemetry::put_number("saved_id", self.saved_id as f64).await;
    }
    /// Gets the targeted tag's angle from the limelight's equator as of the last update
    /// Beware: will return 0 if no tag is targeted
    pub fn get_ty(&self) -> Angle {
        Angle::new::<degree>(self.results.ty)
    }
    /// Gets the targeted tag's angle from the limelight's vertical centerline as of the last update
    /// Beware: will return 0 if no tag is targeted
    pub fn get_tx(&self) -> Angle {
        Angle::new::<degree>(self.results.tx)
    }

    /// Gets the id of the targeted tag as of the last update
    /// RETURNS -1 IF NO TAG FOUND
    pub fn get_id(&self) -> i32 {
        if self.results.Fiducial.len() != 0 {
            self.results.Fiducial[0].fID
        } else {
            -1
        }
    }

    pub fn get_last_results(&self) -> LimelightResults {
        self.last_results.clone()
    }
    pub fn get_saved_id(&self) -> i32 {
        self.saved_id
    }

    /// Returns the horizontal distance from the targeted tag as of the last update to the limelight lens
    /// Returns Option::None if no tag is being targeted
    /// Based on 2D targeting math (known height difference, angle) described here:
    /// https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory
    pub fn get_dist(&self) -> Option<Length> {
        match self.get_tag_position(self.get_id()) {
            Some(..) => {
                let tag_height = self
                    .get_tag_position(self.get_id())?
                    .coordinate
                    .unwrap()
                    .z
                    .get::<inch>();
                let height_diff = tag_height - vision::LIMELIGHT_UPPER_HEIGHT_INCHES;
                let pitch_to_tag: Angle = Angle::new::<degree>(
                    vision::LIMELIGHT_UPPER_PITCH_DEGREES + self.get_ty().get::<degree>(),
                );
                Some(Length::new::<inch>(height_diff) / f64::tan(pitch_to_tag.get::<radian>()))
            }
            None => None,
        }
    }

    /// Uses parsed tagmaps.json (the Vision object's tag_map_values) to return X, Y, and Z
    /// Uses the tagmaps file's coordinate standard: always blue origin, long side X, short side Y, height Z
    /// Returns Option::None if the requested tag doesn't exist in the tagmap
    pub fn get_tag_position(&self, id: i32) -> Option<FieldPosition> {
        let id_json = usize::from_i32(id - 1);
        match id_json {
            Some(_usize) => {
                let coords = Vector3::new(
                    Length::new::<meter>(
                        self.tag_map_values["tags"][id_json?]["pose"]["translation"]["x"]
                            .as_f64()?,
                    ),
                    Length::new::<meter>(
                        self.tag_map_values["tags"][id_json?]["pose"]["translation"]["y"]
                            .as_f64()?,
                    ),
                    Length::new::<meter>(
                        self.tag_map_values["tags"][id_json?]["pose"]["translation"]["z"]
                            .as_f64()?,
                    ),
                );

                let quaternion = Quaternion::new(
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["W"]
                        .as_f64()
                        .unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["X"]
                        .as_f64()
                        .unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["Y"]
                        .as_f64()
                        .unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["Z"]
                        .as_f64()
                        .unwrap(),
                );

                Some(FieldPosition {
                    coordinate: Some(coords),
                    quaternion: Some(quaternion),
                })
            }
            None => None,
        }
    }
    /// Estimates robot position (always blue origin) given a drivetrain angle (CCW+, always blue origin) and last updates' limelight measurements
    /// Uses 2D calculations: distance from tag center & angle to tag center
    /// Returns Option::None if no tag is currently targeted
    pub fn get_position_from_tag_2d(&self) -> Option<Vector2<Length>> {
        let id = self.get_id();
        let dist = self.get_dist()?;
        let drivetrain_angle = Angle::new::<radian>(-self.drivetrain_angle.get::<radian>());

        //println!("dist: {}", dist.get::<meter>());

        // Get tag position and rotation
        let tag_data = self.get_tag_position(id)?;
        let tag_xy = Vector2::new(tag_data.coordinate?.x, tag_data.coordinate?.y);

        let angle_to_tag: Angle =
            (drivetrain_angle) + Angle::new::<degree>(vision::LIMELIGHT_UPPER_YAW_DEGREES) - self.get_tx();
        //println!("angle to tag degrees: dt angle {} + ll yaw {} + tx {} = {}",drivetrain_angle.get::<degree>(),vision::LIMELIGHT_UPPER_YAW_DEGREES,self.get_tx().get::<degree>(),angle_to_tag.get::<degree>());

        // Calculate limelight's position relative to tag
        let limelight_to_tag: Vector2<Length> = Vector2::new(
            dist * f64::cos(angle_to_tag.get::<radian>()),
            dist * f64::sin(angle_to_tag.get::<radian>()),
        );

        // Calculate offset from robot center to limelight
        let robot_center_to_limelight_unrotated_inches: Vector2<f64> = Vector2::new(
            ROBOT_CENTER_TO_LIMELIGHT_UPPER_INCHES.x,
            ROBOT_CENTER_TO_LIMELIGHT_UPPER_INCHES.y,
        );

        // Rotate the limelight offset by drivetrain angle
        let drivetrain_angle_rotation = Rotation2::new(drivetrain_angle.get::<radian>());
        let robot_to_limelight_inches = drivetrain_angle_rotation * robot_center_to_limelight_unrotated_inches;
        let robot_to_limelight: Vector2<Length> = Vector2::new(
            Length::new::<inch>(robot_to_limelight_inches.x),
            Length::new::<inch>(robot_to_limelight_inches.y),
        );

        // Calculate final robot position
        Some(tag_xy - limelight_to_tag - robot_to_limelight)
    }

    /// Returns the botpose: x, y
    pub fn get_botpose_orb(&self) -> Option<Vector2<Length>> {
        let pose: Vector2<Length> = Vector2::new(
            Length::new::<meter>(self.results.botpose_orb_wpiblue[0]),
            Length::new::<meter>(self.results.botpose_orb_wpiblue[1]),
        );
        if pose.x.get::<meter>() == 0. {
            None
        } else {
            Some(pose)
        }
    }

    pub fn get_pose_estimate_orb(&self) -> Option<PoseEstimate> {
        if let Some(pose) = self.get_botpose_orb() {
            Some(PoseEstimate {
                position: pose,
                figure_of_merit: self.get_figure_of_merit()
            })
        } else {
            None
        }
    }
    pub fn get_pose_estimate_2d(&self) -> Option<PoseEstimate> {
        let pose = self.get_position_from_tag_2d()?;
        Some(PoseEstimate{
            position:pose,
            figure_of_merit: self.get_figure_of_merit()
        })
    }

    pub fn get_figure_of_merit(&self) -> Length {
        let dt = Instant::now() - self.last_update_time;
        let angular_velocity_rad_per_sec = (self.drivetrain_angle.get::<radian>() - self.last_drivetrain_angle.get::<radian>()) / dt.as_secs_f64();
        let robot_position_meters: Vector2<f64> = Vector2::new(
            self.robot_position.x.get::<meter>(),
            self.robot_position.y.get::<meter>(),
        );
        let last_robot_position_meters: Vector2<f64> = Vector2::new(
            self.last_robot_position.x.get::<meter>(),
            self.last_robot_position.y.get::<meter>(),
        );
        let linear_velocity_meters_per_sec = (robot_position_meters - last_robot_position_meters).magnitude() / dt.as_secs_f64();

        let mut fom_meters = LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY * angular_velocity_rad_per_sec.abs();
        fom_meters += LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY * linear_velocity_meters_per_sec.abs();
        fom_meters += LIMELIGHT_BASE_FOM;
        Length::new::<meter>(fom_meters)
    }

    pub fn get_botpose(&self) -> Vector2<Length> {
        Vector2::new(
            Length::new::<meter>(self.results.botpose_wpiblue[0]),
            Length::new::<meter>(self.results.botpose_wpiblue[1]),
        )
    }
}
