use std::fs::File;
use frcrs::limelight::Limelight;
use frcrs::telemetry;
use frcrs::telemetry::Telemetry;
use nalgebra::{Quaternion, Vector2, Vector3};
use crate::constants::vision;
use serde_json::Value;
use uom::num::FromPrimitive;
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::meter,
};
use uom::si::length::inch;
use crate::constants::vision::ROBOT_CENTER_TO_LIMELIGHT_INCHES;

#[derive(Clone)]
pub struct Vision {
    tag_map_values: Value,
    limelight_name: String,
    last_id: i32,
    last_tx: Angle,
    last_ty: Angle,
    saved_id: i32,
}

pub struct FieldPosition {
    pub coordinate: Option<Vector3<Length>>,
    pub quaternion: Option<Quaternion<f64>>
}

impl Vision {
    /// Creates new Vision struct from a limelight name (e.g., "limelight-ferris")
    /// Requires that wpilib's tagmap for the year has been put on the rio at "home/lvuser/tagmap.json".
    /// Remember to run NetworkTable::init from frcrs
    pub fn new(name: String) -> Self {
        let tagmap = File::open("/home/lvuser/tagmap.json").unwrap();
        let tag_values: Value = serde_json::from_reader(tagmap).unwrap();
        let tx = Angle::new::<degree>(Limelight::get_tx(name.as_str()));
        let ty = Angle::new::<degree>(Limelight::get_ty(name.as_str()));
        let id = Limelight::get_tid(name.as_str());

        Self {
            tag_map_values: tag_values,
            limelight_name: name,
            last_id: id,
            last_tx: tx,
            last_ty: ty,
            saved_id: id,
        }
    }
    /// Updates the last_id, last_tx, and last_ty variables with new data from NetworkTables
    pub async fn update(&mut self) {
        self.last_id = Limelight::get_tid(self.limelight_name.as_str());
        self.last_tx = Angle::new::<degree>(Limelight::get_tx(self.limelight_name.as_str()));
        self.last_ty = Angle::new::<degree>(Limelight::get_ty(self.limelight_name.as_str()));

        if self.last_id != -1 && self.last_id != self.saved_id {
            self.saved_id = self.last_id;
        }

        Telemetry::put_number("id", self.last_id as f64).await;
        Telemetry::put_number("tx", self.last_tx.value as f64).await;
        Telemetry::put_number("ty", self.last_ty.value as f64).await;
        Telemetry::put_number("saved_id", self.saved_id as f64).await;
    }
    /// Gets the targeted tag's angle from the limelight's equator as of the last update
    /// Beware: will return 0 if no tag is targeted
    pub fn get_ty(&self) -> Angle{
        self.last_ty
    }
    /// Gets the targeted tag's angle from the limelight's vertical centerline as of the last update
    /// Beware: will return 0 if no tag is targeted
    pub fn get_tx(&self) -> Angle{
        self.last_tx
    }

    /// Gets the id of the targeted tag as of the last update
    /// RETURNS -1 IF NO TAG FOUND
    pub fn get_id(&self) -> i32 {
        self.last_id
    }

    pub fn get_saved_id(&self) -> i32 {
        return if self.last_id == -1 {
            self.saved_id
        } else {
            self.last_id
        }
    }

    /// Returns the horizontal distance from the targeted tag as of the last update to the limelight lens
    /// Returns Option::None if no tag is being targeted
    /// Based on 2D targeting math (known height difference, angle) described here:
    /// https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory
    pub fn get_dist(&self) -> Option<Length> {
        match self.get_tag_position(self.get_id()) {
            Some(..) => {
                let tag_height = self.get_tag_position(self.get_id())?.coordinate.unwrap().z;
                let height_diff = tag_height - Length::new::<inch>(vision::LIMELIGHT_HEIGHT_INCHES);
                let pitch_to_tag: Angle = Angle::new::<degree>(vision::LIMELIGHT_PITCH_DEGREES) + self.get_ty();
                Some(height_diff / f64::tan(pitch_to_tag.get::<radian>()))
            }
            None => {
                None
            }
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
                    Length::new::<meter>(self.tag_map_values["tags"][id_json?]["pose"]["translation"]["x"].as_f64()?),
                    Length::new::<meter>(self.tag_map_values["tags"][id_json?]["pose"]["translation"]["y"].as_f64()?),
                    Length::new::<meter>(self.tag_map_values["tags"][id_json?]["pose"]["translation"]["z"].as_f64()?),
                );

                let quaternion = Quaternion::new(
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["W"].as_f64().unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["X"].as_f64().unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["Y"].as_f64().unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["Z"].as_f64().unwrap(),
                );

                Some(FieldPosition {
                    coordinate: Some(coords),
                    quaternion: Some(quaternion)
                })
            }
            None => {
                None
            }
        }
    }
    /// Estimates robot position (always blue origin) given a drivetrain angle (CCW+, always blue origin) and last updates' limelight measurements
    /// Uses 2D calculations: distance from tag center & angle to tag center
    /// Returns Option::None if no tag is currently targeted
    pub fn get_position_from_tag_2d(&self, drivetrain_angle: Angle) -> Option<Vector2<Length>> {
        //println!("dt angle as vision sees (deg): {}", drivetrain_angle.get::<degree>());
        let id = self.get_id();
        let dist = self.get_dist();
        match self.get_tag_position(id) {
            Some(..) => {
                let tag_xy = Vector2::new(
                    self.get_tag_position(id)?.coordinate.unwrap().x,
                    self.get_tag_position(id)?.coordinate.unwrap().y,
                );
                let angle_to_tag: Angle = drivetrain_angle + Angle::new::<degree>(vision::LIMELIGHT_YAW_DEGREES) - self.get_tx();
                let limelight_to_tag: Vector2<Length> = Vector2::new(
                    dist? * f64::cos(angle_to_tag.get::<radian>()),
                    dist? * f64::sin(angle_to_tag.get::<radian>())
                );
                let robot_center_to_limelight_unrotated: Vector2<Length> =
                    Vector2::new
                        (
                            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.x),
                            Length::new::<inch>(ROBOT_CENTER_TO_LIMELIGHT_INCHES.y)
                        );
                let robot_to_limelight: Vector2<Length> = Vector2::new(
                    // Uses 2D vector rotation formula: https://matthew-brett.github.io/teaching/rotation_2d.html
                    Length::new::<meter>(
                        robot_center_to_limelight_unrotated.x.get::<meter>() * f64::cos(drivetrain_angle.get::<radian>())
                            - robot_center_to_limelight_unrotated.y.get::<meter>() * f64::sin(drivetrain_angle.get::<radian>())
                    ),
                    Length::new::<meter>(
                        robot_center_to_limelight_unrotated.x.get::<meter>() * f64::sin(drivetrain_angle.get::<radian>())
                            + robot_center_to_limelight_unrotated.y.get::<meter>() * f64::cos(drivetrain_angle.get::<radian>())
                    )
                );
                Some(tag_xy - limelight_to_tag - robot_to_limelight)
            }
            None => {
                None
            }
        }
    }
}