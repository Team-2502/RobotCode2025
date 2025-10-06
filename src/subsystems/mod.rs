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

pub enum Odometry {
    Normal,
    Localized,
}

impl Odometry {
    pub fn name(&self) -> &'static str {
        match self {
            Odometry::Normal => "normal",
            Odometry::Localized => "localized",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![Odometry::Normal, Odometry::Localized]
    }
    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }
}
