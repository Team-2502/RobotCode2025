use std::ops::Sub;

use crate::constants::pose_estimation::{
    ARC_ODOMETRY_MINIMUM_DELTA_THETA_RADIANS, DRIFT_RATIO, MIN_FOM, START_POSITION_FOM,
};
use frcrs::alliance_station;
use frcrs::networktables::SmartDashboard;
use nalgebra::{Rotation2, Vector2};
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::meter,
};
use crate::constants::{HALF_FIELD_LENGTH_METERS, HALF_FIELD_WIDTH_METERS};

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
    pub(crate) position: Vector2<Length>,
    pub figure_of_merit: Length,
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
        let position: Vector2<Length> =
            Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.));

        Self {
            last_modules,
            robot_pose_estimate: PoseEstimate {
                position,
                figure_of_merit: Length::new::<meter>(START_POSITION_FOM),
            },
        }
    }

    pub fn set_abs(&mut self, position: Vector2<Length>) {
        self.robot_pose_estimate.set_absolute(position);
        self.robot_pose_estimate.figure_of_merit = Length::new::<meter>(START_POSITION_FOM);
    }

    /// Set position, mirrored for red
    pub fn set(&mut self, position: Vector2<Length>) {
        self.robot_pose_estimate.figure_of_merit = Length::new::<meter>(START_POSITION_FOM);

        if alliance_station().red() {
            let half_width = Length::new::<meter>(HALF_FIELD_WIDTH_METERS);
            let half_length = Length::new::<meter>(HALF_FIELD_LENGTH_METERS);

            self.robot_pose_estimate.set_absolute(Vector2::new(
                half_width - position.x + half_width,
                half_length - position.y + half_length
            ));
        } else {
            self.robot_pose_estimate.set_absolute(position);
        }
    }

    pub fn calculate(
        &mut self,
        positions: Vec<ModuleReturn>,
        angle: Angle,
    ) -> Option<PoseEstimate> {
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return None;
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

        let delta_length: Vector2<Length> =
            Vector2::new(Length::new::<meter>(delta.x), Length::new::<meter>(delta.y));
        println!("delta: x {} y {}", delta.x, delta.y);
        self.last_modules = positions;
        let fom = self.robot_pose_estimate.figure_of_merit
            + Length::new::<meter>(delta.magnitude() * DRIFT_RATIO);

        Some(PoseEstimate {
            position: self.robot_pose_estimate.get_position() + delta_length,
            figure_of_merit: self.robot_pose_estimate.figure_of_merit
                + Length::new::<meter>(delta.magnitude() * DRIFT_RATIO),
        })
    }
    /// Calculates the amount the robot has moved since this was last called based on swerve motor positions.
    /// Returns a PoseEstimate adding that to the odometry struct's previous best PoseEstimate.
    /// Uses arc odometry - see 1690's Software Sessions Part II for an explanation.
    /// This will return None at least once, so don't .unwrap() it.
    /// Remember that our angle standard is clockwise-positive on both modules and drivetrain.
    pub fn calculate_arcs(
        &mut self,
        positions: Vec<ModuleReturn>,
        drivetrain_angle: Angle,
    ) -> Option<PoseEstimate> {
        // Hopefully we have the same number of modules as the last time this was called.
        // If not (i.e., this is the first time we called it), return None
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return None;
        }
        //println!("drivetrain angle for calculate_arcs: {}", drivetrain_angle.get::<radian>());

        // Create a Vec with the angles between the current module position and the module position when this function was last called
        // If you're lost figuring out how this works, read the docs on .iter, .zip, and .map
        let mut theta_deltas: Vec<Angle> = positions
            .iter()
            .zip(self.last_modules.iter())
            .map(|(current_modules, last_modules)| {
                //println!("theta delta: {}", (current_modules.to_owned().angle - last_modules.to_owned().angle).get::<radian>());
                current_modules.to_owned().angle - last_modules.to_owned().angle
            })
            .collect();

        // We know that the arc length is equal to how far the module has travelled since this was last called
        let mut arc_lengths: Vec<Length> = positions
            .iter()
            .zip(self.last_modules.iter())
            .map(|(current_modules, last_modules)| {
                //println!("arc length: {}", ((current_modules.to_owned().distance - last_modules.to_owned().distance)).get::<meter>());
                (current_modules.to_owned().distance - last_modules.to_owned().distance)
            })
            .collect();

        // Because we know the arc length and the arc's angle, we can calculate its radius
        // The formula for arc length is arc_length = radius * angle_in_radians
        let mut arc_radii: Vec<Length> = arc_lengths
            .clone()
            .iter()
            .zip(theta_deltas.clone())
            .map(|(arc_length, theta_delta)| {
                //println!("arc radius: {}", (arc_length.get::<meter>() / (2. * std::f64::consts::PI) * ((2. * std::f64::consts::PI) / theta_delta.get::<radian>().abs())));
                Length::new::<meter>(
                    (arc_length.get::<meter>() / theta_delta.get::<radian>().abs()),
                )
            })
            .collect();

        // We can declare the last-module position (starting position) 0,0 in a new robot-oriented coordinate system
        // We know that the arc center is exactly one radius away from there
        // The direction is a vector perpendicular to where the wheel was facing then
        // There are two possible perpendicular vectors - one to the left, and one to the right
        // We can choose the right one by looking at which direction the arc curves in, which we can get from the theta delta
        let mut arc_centers: Vec<Vector2<Length>> = self
            .last_modules
            .clone()
            .iter()
            .zip(arc_radii.clone())
            .zip(theta_deltas.clone())
            .map(|((last_module, arc_radius), theta_delta)| {
                let mut angle_to_center = last_module.angle;
                if theta_delta.get::<radian>() < 0. {
                    // Clockwise is positive, so a theta delta angle less than zero means the wheel turned counterclockwise and the arc curves left
                    angle_to_center -= Angle::new::<degree>(90.); // Rotate the angle_to_center to be perpendicular to the left
                } else {
                    angle_to_center += Angle::new::<degree>(90.); // Or to the right if theta_delta was clockwise
                }
                //println!("angle_to_center degrees: {}", angle_to_center.get::<degree>());
                //println!("center x: {}", arc_radius.get::<meter>() * (-angle_to_center.get::<radian>()).cos());
                //println!("center y: {}", arc_radius.get::<meter>() * (-angle_to_center.get::<radian>()).sin());
                Vector2::new(
                    Length::new::<meter>(
                        arc_radius.get::<meter>() * (-angle_to_center.get::<radian>()).cos(),
                    ), // Negative sign in there because cosine and sine assume counterclockwise-positive
                    Length::new::<meter>(
                        arc_radius.get::<meter>() * (-angle_to_center.get::<radian>()).sin(),
                    ),
                )
            })
            .collect();

        // We can find the vector from the endpoint to the center in the same way we find it for the start point to the center
        // From there, (start to center) - (end to center) = (start to end)
        let mut delta_positions: Vec<Vector2<Length>> = arc_centers
            .clone()
            .iter()
            .zip(arc_radii.clone())
            .zip(positions.clone()) // Zipping more than once leaves you with a Vec of tuples that look like ((start thing , zipped in first), zipped in second)
            .zip(theta_deltas.clone())
            .zip(self.last_modules.clone())
            .map(
                |((((arc_center, arc_radius), current_module), theta_delta), last_module)| {
                    // Endpoint to center, same way as for start point ot center
                    let mut endpoint_angle_to_center = current_module.angle;
                    if theta_delta.get::<radian>() < 0. {
                        endpoint_angle_to_center -= Angle::new::<degree>(90.);
                    } else {
                        endpoint_angle_to_center += Angle::new::<degree>(90.);
                    }
                    //println!("endpoint angle to center degrees: {}", endpoint_angle_to_center.get::<degree>());
                    let endpoint_to_center = Vector2::new(
                        Length::new::<meter>(
                            arc_radius.get::<meter>()
                                * (-endpoint_angle_to_center.get::<radian>()).cos(),
                        ),
                        Length::new::<meter>(
                            arc_radius.get::<meter>()
                                * (-endpoint_angle_to_center.get::<radian>()).sin(),
                        ),
                    );
                    //println!("endpoint to center x: {}", arc_radius.get::<meter>() * (-endpoint_angle_to_center.get::<radian>()).cos());
                    //println!("endpoint to center y: {}",arc_radius.get::<meter>() * (-endpoint_angle_to_center.get::<radian>()).sin());

                    // If theta_delta is very small, we shouldn't mess around with arc odometry, since the arc center point would be miles away
                    // In this case we return a vector constructed the same way as in the regular calculate function
                    if theta_delta.get::<radian>().abs() < ARC_ODOMETRY_MINIMUM_DELTA_THETA_RADIANS
                        || theta_delta.get::<radian>().is_nan()
                    {
                        //println!("delta theta too small");
                        Vector2::new(
                            Length::new::<meter>(
                                Vector2::from((current_module.clone() - last_module.clone())).x,
                            ),
                            Length::new::<meter>(
                                Vector2::from((current_module.clone() - last_module.clone())).y,
                            ),
                        )
                    } else {
                        //println!("delta x: {}", (arc_center - endpoint_to_center).x.get::<meter>());
                        //println!("delta y: {}", (arc_center - endpoint_to_center).y.get::<meter>());
                        arc_center - endpoint_to_center
                    }
                },
            )
            .collect();

        // Rotate all by the drivetrain angle to get from robot-relative coordinates to field-relative coordinates
        let drivetrain_angle_rotation = Rotation2::new(drivetrain_angle.get::<radian>()); // -1 * drivetrain angle because we use clockwise positive and the Rotation2 * Vector2 math assumes CCW+
        delta_positions = delta_positions
            .clone()
            .iter()
            .map(|delta_position| {
                let mut delta_position_meters = Vector2::new(
                    delta_position.x.get::<meter>(), // The rotation * vector math doesn't work for Vector<Length>s
                    delta_position.y.get::<meter>(),
                );
                delta_position_meters = drivetrain_angle_rotation.clone() * delta_position_meters; // A Rotation2 times a Vector2<f64> returns a Vector2<f64> of the vector rotated by the rotation
                Vector2::new(
                    Length::new::<meter>(delta_position_meters.x),
                    Length::new::<meter>(delta_position_meters.y),
                )
            })
            .collect();

        // Average the module deltas to get robot delta
        let mut delta: Vector2<Length> = delta_positions.iter().sum();
        let mut delta_meters: Vector2<f64> =
            Vector2::new(delta.x.get::<meter>(), delta.y.get::<meter>()); //Can't divide a Length by a f64, apparently.
        delta_meters /= delta_positions.len() as f64;
        delta = Vector2::new(
            Length::new::<meter>(delta_meters.x),
            Length::new::<meter>(delta_meters.y),
        );
        self.last_modules = positions;

        // We estimate the error accumulated by driving around to be DRIFT_RATIO * the distance driven.
        // We estimate distance driven to be the average of the arc lengths rather than the length of the delta vector to increase FoM when the modules are turning quickly
        let mut average_arc_length_meters: f64 = 0.;
        arc_lengths.iter().for_each(|arc_length| {
            average_arc_length_meters += arc_length.get::<meter>().abs();
        });
        average_arc_length_meters /= arc_lengths.len() as f64;
        let figure_of_merit = self.robot_pose_estimate.figure_of_merit
            + Length::new::<meter>(average_arc_length_meters * DRIFT_RATIO);

        Some(PoseEstimate {
            position: self.robot_pose_estimate.get_position() + delta,
            figure_of_merit: figure_of_merit,
        })
    }

    /// Uses a figure of merit calculation to estimate the true robot pose based on a set of sensor measurements and their confidence (figure of merit).
    /// Each sensor measurement should have a figure of merit value estimating how far it is likely to be from the true measurement.
    /// This was taken from 1690's Software Sessions Part II - they'll explain it better than code comments can.
    pub fn fuse_sensors_fom(&mut self, pose_estimates: Vec<PoseEstimate>) {
        // Extremely low figure of merit values can make some numbers in the calculation very, very large
        // This bit was created to try to solve an error that turned out to be a different problem - I'm not sure if it actually needs to be here.
        let pose_estimates: Vec<PoseEstimate> = pose_estimates
            .iter()
            .map(|pose_estimate| {
                if pose_estimate.figure_of_merit.get::<meter>() < MIN_FOM {
                    PoseEstimate {
                        position: pose_estimate.position.clone(),
                        figure_of_merit: Length::new::<meter>(MIN_FOM),
                    }
                } else {
                    pose_estimate.clone()
                }
            })
            .collect();

        // The formula for the FoM calculation is:
        // (Robot pose) = (((Sensor 1 estimate) / (Sensor 1 FoM)^2) + ((Sensor 2 estimate) / (Sensor 2 FoM)^2) + ...) / ((1 / (Sensor 1 FoM)^2) +  (1 / (Sensor 2 FoM)^2) + ...)
        // Look at 1690's Software Sessions Part II for it actually written out on a slide

        // A vec of every element on the bottom of the equation
        let fom_inverse_squared_values: Vec<f64> = pose_estimates
            .iter()
            .map(|pose_estimate| {
                (1. / (pose_estimate.figure_of_merit.get::<meter>()
                    * pose_estimate.figure_of_merit.get::<meter>()))
            })
            .collect();

        // A vec of every element on the top of the equation
        let poses_times_fom_inverse_squared_values: Vec<Vector2<f64>> = pose_estimates
            .iter()
            .zip(fom_inverse_squared_values.clone())
            .map(|(pose_estimate, inverse_squared_value)| {
                Vector2::new(
                    pose_estimate.position.x.get::<meter>(),
                    pose_estimate.position.y.get::<meter>(),
                ) * inverse_squared_value
            })
            .collect();

        // Sum everything and divide the top by the bottom
        let mut robot_pose_estimate_meters_x: f64 = 0.;
        let mut robot_pose_estimate_meters_y: f64 = 0.;
        let mut inverse_squared_sum: f64 = 0.;
        poses_times_fom_inverse_squared_values
            .iter()
            .zip(fom_inverse_squared_values.clone())
            .for_each(|(pose_estimate, fom_inverse_squared_value)| {
                robot_pose_estimate_meters_x += pose_estimate.x;
                robot_pose_estimate_meters_y += pose_estimate.y;
                inverse_squared_sum += fom_inverse_squared_value;
            });
        robot_pose_estimate_meters_x /= inverse_squared_sum;
        robot_pose_estimate_meters_y /= inverse_squared_sum;

        // Robot FoM is equal to the smallest sensor FoM
        let mut min_sensor_fom_meters: f64 = 1000.;
        pose_estimates.iter().for_each(|pose_estimate| {
            if pose_estimate.figure_of_merit.get::<meter>() < min_sensor_fom_meters {
                min_sensor_fom_meters = pose_estimate.figure_of_merit.get::<meter>();
            }
        });

        self.robot_pose_estimate = PoseEstimate {
            position: Vector2::new(
                Length::new::<meter>(robot_pose_estimate_meters_x),
                Length::new::<meter>(robot_pose_estimate_meters_y),
            ),
            figure_of_merit: Length::new::<meter>(min_sensor_fom_meters),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn calculate_arcs_forward_forward_forward() {
        //last modules forward, new modules forward, drivetrain forward
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });

        let drivetrain_angle = Angle::new::<degree>(0.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(1.), Length::new::<meter>(0.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }

    #[test]
    fn calculate_arcs_forward_forward_left() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });

        let drivetrain_angle = Angle::new::<degree>(-90.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(1.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }

    #[test]
    fn calculate_arcs_forward_forward_backward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });

        let drivetrain_angle = Angle::new::<degree>(180.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(-1.), Length::new::<meter>(0.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
    #[test]
    fn calculate_arcs_forward_forward_right() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(0.),
        });

        let drivetrain_angle = Angle::new::<degree>(90.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(-1.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
    #[test]
    fn calculate_arcs_backward_backward_forward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });

        let drivetrain_angle = Angle::new::<degree>(0.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(-1.), Length::new::<meter>(0.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }

    #[test]
    fn calculate_arcs_backward_backward_backward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-180.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });

        let drivetrain_angle = Angle::new::<degree>(-180.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(1.), Length::new::<meter>(0.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
    #[test]
    fn calculate_arcs_forward_backward_forward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-180.),
        });

        let drivetrain_angle = Angle::new::<degree>(0.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        // This one just needs to not crash - it should never happen in real life
    }
    #[test]
    fn calculate_arcs_left_left_forward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-90.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-90.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-90.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(-90.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });

        let drivetrain_angle = Angle::new::<degree>(0.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(1.)),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
    #[test]
    fn calculate_arcs_forward_left_forward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });

        let drivetrain_angle = Angle::new::<degree>(0.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(
                Length::new::<meter>(std::f64::consts::FRAC_2_PI),
                Length::new::<meter>(std::f64::consts::FRAC_2_PI),
            ),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
    #[test]
    fn calculate_arcs_forward_left_backward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(-90.),
        });

        let drivetrain_angle = Angle::new::<degree>(-180.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(
                Length::new::<meter>(-std::f64::consts::FRAC_2_PI),
                Length::new::<meter>(-std::f64::consts::FRAC_2_PI),
            ),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
    #[test]
    fn calculate_arcs_forward_right_forward() {
        let mut odometry = Odometry::new();

        let mut last_modules = Vec::new();
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });
        last_modules.push(ModuleReturn {
            distance: Length::new::<meter>(0.),
            angle: Angle::new::<degree>(0.),
        });

        odometry.last_modules = last_modules;

        let mut new_modules = Vec::new();
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(90.),
        });
        new_modules.push(ModuleReturn {
            distance: Length::new::<meter>(1.),
            angle: Angle::new::<degree>(90.),
        });

        let drivetrain_angle = Angle::new::<degree>(0.);

        let estimated_pose = odometry
            .calculate_arcs(new_modules, drivetrain_angle)
            .unwrap();
        let correct_pose = PoseEstimate {
            position: Vector2::new(
                Length::new::<meter>(std::f64::consts::FRAC_2_PI),
                Length::new::<meter>(-std::f64::consts::FRAC_2_PI),
            ),
            figure_of_merit: Length::new::<meter>(0.05 + START_POSITION_FOM),
        };

        println!(
            "Estimated pose x: {} Correct pose x: {}",
            estimated_pose.get_position_meters().x,
            correct_pose.get_position_meters().x
        );
        println!(
            "Estimated pose y: {} Correct pose y: {}",
            estimated_pose.get_position_meters().y,
            correct_pose.get_position_meters().y,
        );
        println!(
            "Estimated pose FoM: {} Correct pose FoM: {}",
            estimated_pose.figure_of_merit.get::<meter>(),
            correct_pose.figure_of_merit.get::<meter>()
        );

        assert!(
            (estimated_pose.get_position_meters().x - correct_pose.get_position_meters().x).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.get_position_meters().y - correct_pose.get_position_meters().y).abs()
                <= 0.0001
        );
        assert!(
            (estimated_pose.figure_of_merit.get::<meter>()
                - correct_pose.figure_of_merit.get::<meter>())
            .abs()
                <= 0.0001
        );
    }
}
