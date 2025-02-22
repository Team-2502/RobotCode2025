use std::f64::consts::FRAC_PI_2;
use uom::si::f64::*;
use uom::si::{
    angle::{degree, radian},
    f64::Length,
    length::inch,
};

use nalgebra::{ComplexField, Rotation2, Vector2};
use crate::constants::drivetrain::SWERVE_TURN_RATIO;

pub type WheelSpeeds = Vec<ModuleState>;
pub type ModulePosition = Vector2<f64>;
#[derive(Debug, PartialEq, PartialOrd)]
pub struct ModuleState {
    pub speed: f64,
    pub angle: Angle,
}

impl ModuleState {
    /// change this module to be equivalent, but close to other
    pub fn optimize(mut self, other: &ModuleState) -> Self {
        let angle = self.angle.get::<degree>();
        let other_angle = other.angle.get::<degree>();

        let mut difference = (angle - other_angle + 180.) % 360. - 180.;
        if difference < -180. {
            difference += 360.
        };

        if difference.abs() > 90. {
            self.speed *= -1.;

            if difference > 0. {
                difference += -180.;
            } else {
                difference += 180.;
            }
        }

        self.angle = other.angle + Angle::new::<degree>(difference);

        self
    }
}

pub struct Swerve {
    positions: Vec<ModulePosition>,
}

impl Swerve {
    pub fn rectangle(width: Length, height: Length) -> Self {
        let mut positions: Vec<ModulePosition> = Vec::new();

        let x = width.get::<inch>() / 2.0;
        let y = height.get::<inch>() / 2.0;
        positions.push(ModulePosition::new(x, y));
        positions.push(ModulePosition::new(-x, y));
        positions.push(ModulePosition::new(-x, -y));
        positions.push(ModulePosition::new(x, -y));

        Self { positions }
    }

    /// Calculate module speeds from the given transform and rotation
    ///
    /// input units are from -1 to 1, with 1 being the maximum motor speed
    ///
    /// positive x is right
    /// positive y is forward
    /// positive rotation is clockwise
    pub fn calculate(&self, transform: Vector2<f64>, rotation: f64) -> WheelSpeeds {
        let center_of_rotation = Vector2::zeros(); // relative to center of robot
        let rotation_transform = Rotation2::new(-FRAC_PI_2); // 90 degrees

        let mut speeds = Vec::new();

        for module in self.positions.clone() {
            // constant component
            let mut vector = transform;

            let position = module - center_of_rotation;
            let unit_rotation = rotation_transform * position.normalize();

            // rotation
            vector += unit_rotation * rotation;

            let angle = f64::atan2(vector.x, vector.y);

            speeds.push(ModuleState {
                speed: vector.magnitude(),
                angle: Angle::new::<radian>(angle),
            });
        }

        // normalize speeds
        let max = speeds.iter().map(|m| m.speed.abs()).fold(0., f64::max);
        if max > 1. {
            speeds.iter_mut().for_each(|s| s.speed /= max);
        }

        /*SmartDashboard::put_number("fl turn".to_owned(), wa2);
        SmartDashboard::put_number("fr turn".to_owned(), wa1);
        SmartDashboard::put_number("bl turn".to_owned(), wa3);
        SmartDashboard::put_number("br turn".to_owned(), wa4);*/

        speeds
    }
}

pub trait ToTalonEncoder {
    fn talon_encoder_ticks(&self) -> f64;
    fn from_talon_encoder_ticks(&self) -> f64;
}

impl ToTalonEncoder for f64 {
    fn talon_encoder_ticks(&self) -> f64 {
        self / ((360.) / (2048. * SWERVE_TURN_RATIO))
    }

    fn from_talon_encoder_ticks(&self) -> f64 {
        self * ((360.) / (2048. * SWERVE_TURN_RATIO))
    }
}

#[cfg(test)]
mod tests {

    use nalgebra::Vector2;
    use uom::si::{
        angle::degree,
        f64::{Angle, Length},
        length::inch,
    };

    use crate::swerve::kinematics::ModuleState;

    use super::Swerve;

    #[test]
    fn forward() {
        let width = Length::new::<inch>(25.);
        let swerve = Swerve::rectangle(width, width);

        let positions = swerve.calculate(Vector2::new(0.0, 1.0), 0.);

        assert_eq!(positions[0].angle, Angle::new::<degree>(0.))
    }

    #[test]
    fn right() {
        let width = Length::new::<inch>(25.);
        let swerve = Swerve::rectangle(width, width);

        let positions = swerve.calculate(Vector2::new(1.0, 0.0), 0.);

        assert_eq!(positions[0].angle, Angle::new::<degree>(90.))
    }

    #[test]
    fn clockwise() {
        let width = Length::new::<inch>(25.);
        let swerve = Swerve::rectangle(width, width);

        let positions = swerve.calculate(Vector2::new(0.0, 0.0), 1.);

        assert_eq!(positions[0].angle, Angle::new::<degree>(135.));
        //assert_eq!(positions[0].speed, 1.);
        assert_eq!(positions[1].angle, Angle::new::<degree>(45.));
    }

    #[test]
    fn opposite() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(180.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(0.),
        };
        let goal = ModuleState {
            speed: -1.,
            angle: Angle::new::<degree>(0.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn close_cycle() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(45.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(360.),
        };
        let goal = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(360. + 45.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn close() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(45.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(0.),
        };
        let goal = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(45.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn close_opposite() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(45. + 180.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(0.),
        };
        let goal = ModuleState {
            speed: -1.,
            angle: Angle::new::<degree>(45.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn zero_negative() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(-360. * 2.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(0.),
        };
        let goal = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(0.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn zero_negative_far() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(-360. * 2.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(360. + 180.),
        };
        let goal = ModuleState {
            speed: -1.,
            angle: Angle::new::<degree>(360. + 180.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn negative_far_close() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(-360. * 2. + 1.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(360. + 180.),
        };
        let goal = ModuleState {
            speed: -1.,
            angle: Angle::new::<degree>(360. + 180. + 1.),
        };

        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }

    #[test]
    fn negative_far_close_below() {
        let this = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(-360. * 2. - 30.),
        };
        let other = ModuleState {
            speed: 1.,
            angle: Angle::new::<degree>(360. + 180.),
        };
        let goal = ModuleState {
            speed: -1.,
            angle: Angle::new::<degree>(360. + 180. - 30.),
        };
        let to = this.optimize(&other);

        assert_eq!(to, goal);
    }
}
