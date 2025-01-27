use std::ops::Sub;

use frcrs::alliance_station;
use frcrs::networktables::SmartDashboard;
use nalgebra::{Rotation2, Vector2};
use uom::si::{
    angle::radian,
    f64::{Angle, Length},
    length::meter,
};

use crate::constants::HALF_FIELD_WIDTH_METERS;

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

pub struct Odometry {
    last_modules: Vec<ModuleReturn>,
    pub position: Vector2<f64>,
}

impl Default for Odometry {
    fn default() -> Self {
        Self::new()
    }
}

impl Odometry {
    pub fn new() -> Self {
        let last_modules = Vec::new();
        let position = Vector2::new(0., 0.);

        Self {
            last_modules,
            position,
        }
    }

    pub fn set(&mut self, position: Vector2<f64>) {
        if alliance_station().red() {
            self.position.x = position.x;
            self.position.y = HALF_FIELD_WIDTH_METERS - position.y;
        } else {
            self.position = position;
        }
    }

    pub fn set_abs(&mut self, position: Vector2<f64>) {
        self.position = position;
    }

    pub fn calculate(&mut self, positions: Vec<ModuleReturn>, angle: Angle) {
        if positions.len() != self.last_modules.len() {
            self.last_modules = positions;
            return;
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

        self.position += -delta;
        self.last_modules = positions;

        SmartDashboard::set_position(self.position, angle);
    }
}
