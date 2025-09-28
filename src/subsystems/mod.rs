mod climber;
mod drivetrain;
mod elevator;
mod indexer;
mod led;
mod vision;

pub use climber::*;
pub use drivetrain::*;
pub use elevator::*;
pub use indexer::*;
pub use vision::*;
use crate::auto::Auto;

pub enum Odometry {
    Localized,
    Normal,
}

impl Odometry {
    pub fn name(&self) -> &'static str {
        match self {
            Odometry::Localized => "localized",
            Odometry::Normal => "normal",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![
            Odometry::Localized,
            Odometry::Normal,
        ]
    }
    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

}
