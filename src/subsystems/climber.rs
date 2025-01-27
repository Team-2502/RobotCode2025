use crate::constants::robotmap::climber::*;
use frcrs::solenoid::{ModuleType, Solenoid};
use std::time::Duration;
use tokio::time::sleep;

pub struct Climber {
    raise: Solenoid,
    grab: Solenoid,
}

impl Default for Climber {
    fn default() -> Self {
        Self::new()
    }
}

impl Climber {
    pub fn new() -> Self {
        Self {
            raise: Solenoid::new(ModuleType::Rev, RAISE),
            grab: Solenoid::new(ModuleType::Rev, GRAB),
        }
    }

    pub fn toggle_raise(&self) {
        self.raise.toggle();
    }

    pub fn set_raise(&self, engaged: bool) {
        self.raise.set(engaged);
    }

    pub fn toggle_grab(&self) {
        self.grab.toggle();
    }

    pub fn set_grab(&self, engaged: bool) {
        self.grab.set(engaged);
    }

    pub async fn climb(&self) {
        self.set_raise(true);
        sleep(Duration::from_secs(1)).await;
        self.set_grab(true);
    }
}
