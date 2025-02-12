use crate::constants::robotmap::climber::*;
use crate::Ferris;
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
        self.grab.set(!engaged);
    }

    pub async fn climb(ferris: Ferris) {
        if let Ok(climber) = ferris.climber.try_borrow_mut() {
            climber.set_raise(true);
            sleep(Duration::from_secs(1)).await;
            climber.set_grab(true);
            sleep(Duration::from_secs_f64(0.25)).await;
            climber.set_raise(false);
        }
    }

    pub fn fall(&self) {
        self.set_grab(false);
        self.set_raise(false);
    }

    pub async fn reverse_climb(&self) {
        self.set_raise(true);
        sleep(Duration::from_secs_f64(0.25)).await;
        self.set_grab(false);
        sleep(Duration::from_secs_f64(0.25)).await;
        self.set_raise(false);
    }
}
