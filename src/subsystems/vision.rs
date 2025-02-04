use frcrs::limelight::{Limelight, LimelightResults};
use std::fs::File;

use crate::constants::vision;
use crate::constants::vision::ROBOT_CENTER_TO_LIMELIGHT_INCHES;
use frcrs::telemetry::Telemetry;
use nalgebra::{Quaternion, Vector2, Vector3};
use serde_json::Value;
use uom::num::FromPrimitive;
use uom::si::length::inch;
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::meter,
};

use std::net::SocketAddr;

#[derive(Clone)]
pub struct Vision {
    tag_map_values: Value,
    limelight: Limelight,
    results: LimelightResults,
    last_results: LimelightResults,
    saved_id: i32,
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
        }
    }
    /// Updates the results from the limelight, also posts telemetry data
    pub async fn update(&mut self, dt_angle: f64) {
        self.last_results = self.results.clone();
        self.results = self.limelight.results().await.unwrap();
        self.limelight
            .update_robot_orientation(dt_angle)
            .await
            .unwrap();

        if !self.results.Fiducial.is_empty() {
            if self.results.Fiducial[0].fID != -1 && self.results.Fiducial[0].fID != self.saved_id {
                self.saved_id = self.results.Fiducial[0].fID;
            }

            Telemetry::put_number("id", self.results.Fiducial[0].fID as f64).await;
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

    pub fn get_last_results(&self) -> LimelightResults {self.last_results.clone()}
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
                let height_diff = tag_height - vision::LIMELIGHT_HEIGHT_INCHES;
                let pitch_to_tag: Angle = Angle::new::<degree>(
                    vision::LIMELIGHT_PITCH_DEGREES + self.get_ty().get::<degree>(),
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
    pub fn get_position_from_tag_2d(&self, drivetrain_angle: Angle) -> Option<Vector2<Length>> {
        let id = self.get_id();
        let dist = self.get_dist()?;

        println!("dist: {}", dist.get::<meter>());

        // Get tag position and rotation
        let tag_data = self.get_tag_position(id)?;
        let tag_xy = Vector2::new(tag_data.coordinate?.x, tag_data.coordinate?.y);
        let tag_quaternion = tag_data.quaternion?;

        let tag_yaw = Angle::new::<radian>(f64::atan2(
            2.0 * (tag_quaternion.w * tag_quaternion.k + tag_quaternion.i * tag_quaternion.j),
            1.0 - 2.0 * (tag_quaternion.j * tag_quaternion.j + tag_quaternion.k * tag_quaternion.k),
        ));

        let angle_to_tag: Angle =
            drivetrain_angle + Angle::new::<degree>(vision::LIMELIGHT_YAW_DEGREES) - self.get_tx()
                + tag_yaw;

        // Calculate limelight's position relative to tag
        let limelight_to_tag: Vector2<Length> = Vector2::new(
            dist * f64::cos(angle_to_tag.get::<radian>()),
            dist * f64::sin(angle_to_tag.get::<radian>()),
        );

        // Calculate offset from robot center to limelight
        let robot_center_to_limelight_unrotated: Vector2<Length> = Vector2::new(
            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.x),
            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.y),
        );

        // Rotate the limelight offset by drivetrain angle
        let robot_to_limelight: Vector2<Length> = Vector2::new(
            Length::new::<meter>(
                robot_center_to_limelight_unrotated.x.get::<meter>()
                    * f64::cos(drivetrain_angle.get::<radian>())
                    - robot_center_to_limelight_unrotated.y.get::<meter>()
                        * f64::sin(drivetrain_angle.get::<radian>()),
            ),
            Length::new::<meter>(
                robot_center_to_limelight_unrotated.x.get::<meter>()
                    * f64::sin(drivetrain_angle.get::<radian>())
                    + robot_center_to_limelight_unrotated.y.get::<meter>()
                        * f64::cos(drivetrain_angle.get::<radian>()),
            ),
        );

        // Calculate final robot position
        Some(tag_xy - limelight_to_tag - robot_to_limelight)
    }

    /// Returns the botpose: x, y
    pub fn get_botpose_orb(&self) -> Vector2<Length> {
        Vector2::new(
            Length::new::<meter>(self.results.botpose_orb_wpiblue[0]),
            Length::new::<meter>(self.results.botpose_orb_wpiblue[1]),
        )
    }

    pub fn get_botpose(&self) -> Vector2<Length> {
        Vector2::new(
            Length::new::<meter>(self.results.botpose_wpiblue[0]),
            Length::new::<meter>(self.results.botpose_wpiblue[1]),
        )
    }
}
