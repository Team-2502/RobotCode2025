use std::ops::Sub;

use frcrs::alliance_station;
use frcrs::networktables::SmartDashboard;
use nalgebra::{Rotation2, Vector2};
use uom::si::{
    angle::{radian, degree},
    f64::{Angle, Length},
    length::meter,
};
use crate::constants::HALF_FIELD_WIDTH_METERS;
use crate::constants::pose_estimation::{ARC_ODOMETRY_MINIMUM_DELTA_THETA_RADIANS, DRIFT_RATIO, START_POSITION_FOM};

#[derive(Default, Clone)]
pub struct ModuleReturn {
    pub distance: Length,
    pub angle: Angle,
}

impl From<ModuleReturn> for Vector2<f64> {
    fn from(val: ModuleReturn) -> Self {
        let angle = val.angle.get::<radian>();
        let distance = val.distance.get::<meter>();

        Rotation2::new(-angle) * Vector2::x() * distance
    }
}

impl Sub for ModuleReturn {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            distance: self.distance - rhs.distance,
            angle: self.angle,
        }
    }
}
#[derive(Clone)]
pub struct PoseEstimate {
    position: Vector2<Length>,
    pub figure_of_merit: Length
}
impl PoseEstimate {
    pub fn get_position_meters(&self) -> Vector2<f64> {
        Vector2::new(
            self.position.x.get::<meter>(),
            self.position.y.get::<meter>(),
        )
    }
    pub fn get_position(&self) -> Vector2<Length> {
        self.position
    }
    pub fn set_absolute(&mut self, position: Vector2<Length>) {
        self.position = position;
    }
}

pub struct Odometry {
    last_modules: Vec<ModuleReturn>,
    pub robot_pose_estimate: PoseEstimate,
}

impl Default for Odometry {
    fn default() -> Self {
        Self::new()
    }
}

impl Odometry {
    pub fn new() -> Self {
        let last_modules = Vec::new();
        let position:Vector2<Length> = Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.));

        Self {
            last_modules,
            robot_pose_estimate: PoseEstimate {position, figure_of_merit: Length::new::<meter>(START_POSITION_FOM) },
        }
    }

    pub fn set_abs(&mut self, position: Vector2<Length>) {
        self.robot_pose_estimate.set_absolute(position);
    }

    pub fn calculate(&mut self, positions: Vec<ModuleReturn>, angle: Angle) -> Option<PoseEstimate> {
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return None
        }

        let mut deltas: Vec<ModuleReturn> = positions
            .iter()
            .zip(self.last_modules.iter())
            .map(|(n, o)| n.to_owned() - o.to_owned())
            .collect();

        for module in &mut deltas {
            module.angle += angle;
        }

        let mut delta: Vector2<f64> = deltas.into_iter().map(Into::<Vector2<f64>>::into).sum();

        delta /= positions.len() as f64;
        let delta_length: Vector2<Length> = Vector2::new(
            Length::new::<meter>(delta.x),
            Length::new::<meter>(delta.y)
        );
        self.last_modules = positions;

        Some(PoseEstimate {
            position: self.robot_pose_estimate.get_position() + delta_length,
            figure_of_merit: self.robot_pose_estimate.figure_of_merit + Length::new::<meter>(delta.magnitude() * DRIFT_RATIO)
        })

    }
    pub fn calculate_arcs(&mut self, positions: Vec<ModuleReturn>, drivetrain_angle: Angle) -> Option<PoseEstimate>{
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return None
        }
        println!("drivetrain angle for calculate_arcs: {}", drivetrain_angle.get::<radian>());

        let mut theta_deltas : Vec<Angle> = positions
            .iter()
            .zip(self.last_modules.iter())
            .map(|(current_modules, last_modules)|
                current_modules.to_owned().angle - last_modules.to_owned().angle)
            .collect();
        let mut arc_lengths: Vec<Length> = positions
            .iter()
            .zip(self.last_modules.iter())
            .map(|(current_modules, last_modules)|
                (current_modules.to_owned().distance - last_modules.to_owned().distance) / 2.)
            .collect();
        let mut arc_radii: Vec<Length> = arc_lengths.clone()
            .iter()
            .zip(theta_deltas.clone())
            .map(|(arc_length, theta_delta)|
                Length::new::<meter>((arc_length.get::<meter>() / (2. * std::f64::consts::PI) * ((2. * std::f64::consts::PI) / theta_delta.get::<radian>()))))
            .collect();
        let mut arc_centers: Vec<Vector2<Length>> = self.last_modules.clone()
            .iter()
            .zip(arc_radii.clone())
            .zip(theta_deltas.clone())
            .map(|((last_module, arc_radius), theta_delta)| {
                let mut angle_to_center = last_module.angle;
                if theta_delta.get::<radian>() < 0. {
                    angle_to_center -= Angle::new::<degree>(90.);
                } else {
                    angle_to_center += Angle::new::<degree>(90.);
                }
                Vector2::new(
                    Length::new::<meter>(arc_radius.get::<meter>() * (-angle_to_center.get::<radian>()).cos()),
                    Length::new::<meter>(arc_radius.get::<meter>() * (-angle_to_center.get::<radian>()).sin()),
                )
            }).collect();
        let mut delta_positions: Vec<Vector2<Length>> = arc_centers.clone()
            .iter()
            .zip(arc_radii.clone())
            .zip(positions.clone())
            .zip(theta_deltas.clone())
            .zip(self.last_modules.clone())
            .map(|((((arc_center, arc_radius), current_module), theta_delta), last_module)|{
                let mut endpoint_angle_to_center = current_module.angle;
                if theta_delta.get::<radian>() < 0. {
                    endpoint_angle_to_center -= Angle::new::<degree>(90.);
                } else {
                    endpoint_angle_to_center += Angle::new::<degree>(90.);
                }
                let endpoint_to_center = Vector2::new(
                    Length::new::<meter>(arc_radius.get::<meter>() * (-endpoint_angle_to_center.get::<radian>()).cos()),
                    Length::new::<meter>(arc_radius.get::<meter>() * (-endpoint_angle_to_center.get::<radian>()).sin()),
                );
                if theta_delta.get::<radian>().abs() < ARC_ODOMETRY_MINIMUM_DELTA_THETA_RADIANS {
                    Vector2::new(
                        Length::new::<meter>( Vector2::from((last_module.clone() - current_module.clone())).x),
                        Length::new::<meter>( Vector2::from((last_module.clone() - current_module.clone())).y),
                    )
                } else {
                    arc_center - endpoint_to_center
                }
            }).collect();
        let drivetrain_angle_rotation = Rotation2::new(drivetrain_angle.get::<radian>());

        // Rotate all by the drivetrain angle to get into field coordinates
        delta_positions = delta_positions.clone()
            .iter()
            .map(|delta_position| {
                let mut delta_position_meters = Vector2::new(
                    delta_position.x.get::<meter>(),
                    delta_position.y.get::<meter>()
                );
                delta_position_meters = drivetrain_angle_rotation.clone() * delta_position_meters;
                Vector2::new(
                    Length::new::<meter>(delta_position_meters.x),
                    Length::new::<meter>(delta_position_meters.y),
                )
            }).collect();

        // Average the module deltas to get robot delta
        let mut delta: Vector2<Length> = delta_positions.iter().sum();
        let mut delta_meters: Vector2<f64> = Vector2::new(delta.x.get::<meter>(), delta.y.get::<meter>());
        delta_meters /= delta_positions.len() as f64;
        delta = Vector2::new(
            Length::new::<meter>(delta_meters.x),
            Length::new::<meter>(delta_meters.y),
        );
        self.last_modules = positions;

        Some(PoseEstimate {
            position: self.robot_pose_estimate.get_position() + delta,
            figure_of_merit: self.robot_pose_estimate.figure_of_merit + Length::new::<meter>(delta_meters.magnitude() * DRIFT_RATIO),
        })
    }
    pub fn fuse_sensors_fom(&mut self, pose_estimates: Vec<PoseEstimate>) {
        let fom_inverse_squared_values: Vec<f64> = pose_estimates
            .iter()
            .map(|pose_estimate| (1. / (pose_estimate.figure_of_merit.get::<meter>() * pose_estimate.figure_of_merit.get::<meter>())))
            .collect();
        let poses_times_fom_inverse_squared_values: Vec<Vector2<f64>> = pose_estimates
            .iter()
            .zip(fom_inverse_squared_values.clone())
            .map(|(pose_estimate, inverse_squared_value)| {
                Vector2::new(
                    pose_estimate.position.x.get::<meter>(),
                    pose_estimate.position.y.get::<meter>(),
                ) * inverse_squared_value
            }).collect();
        let mut robot_pose_estimate_meters_x: f64 = 0.;
        let mut robot_pose_estimate_meters_y: f64 = 0.;
        let mut inverse_squared_sum: f64 = 0.;
        poses_times_fom_inverse_squared_values.iter().zip(fom_inverse_squared_values.clone()).for_each(|(pose_estimate, fom_inverse_squared_value)| {
            robot_pose_estimate_meters_x += pose_estimate.x * fom_inverse_squared_value;
            robot_pose_estimate_meters_y += pose_estimate.y * fom_inverse_squared_value;
            inverse_squared_sum += fom_inverse_squared_value;
        });
        robot_pose_estimate_meters_x /= inverse_squared_sum;
        robot_pose_estimate_meters_y /= inverse_squared_sum;

        let mut min_sensor_fom: f64 = 1000.;
        pose_estimates.iter().for_each(|pose_estimate| {
            if pose_estimate.figure_of_merit.get::<meter>() < min_sensor_fom {
                min_sensor_fom = pose_estimate.figure_of_merit.get::<meter>();
            }
        });

        self.robot_pose_estimate = PoseEstimate {
            position: Vector2::new(
                Length::new::<meter>(robot_pose_estimate_meters_x),
                Length::new::<meter>(robot_pose_estimate_meters_y),
            ),
            figure_of_merit: Length::new::<meter>(min_sensor_fom),
        }
    }
}
