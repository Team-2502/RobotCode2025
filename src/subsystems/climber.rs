use std::time::Duration;
use frcrs::solenoid::{ModuleType, Solenoid};
use tokio::time::sleep;
use crate::constants::robotmap::climber::*;

pub struct Climber {
    fl: Solenoid,
    fr: Solenoid,
    bl: Solenoid,
    br: Solenoid,

    left: Solenoid,
    right: Solenoid,
}

impl Climber {
    pub fn new() -> Self {
        Self {
            fl: Solenoid::new(ModuleType::Rev, FL),
            fr: Solenoid::new(ModuleType::Rev, FR),
            bl: Solenoid::new(ModuleType::Rev, BL),
            br: Solenoid::new(ModuleType::Rev, BR),
            left: Solenoid::new(ModuleType::Rev, LEFT),
            right: Solenoid::new(ModuleType::Rev, RIGHT)
        }
    }

    pub fn toggle_raise(&self) {
        self.fl.toggle();
        self.fr.toggle();
        self.bl.toggle();
        self.br.toggle();
    }

    pub fn set_raise(&self, engaged: bool) {
        self.fl.set(engaged);
        self.fr.set(engaged);
        self.bl.set(engaged);
        self.br.set(engaged);
    }

    pub fn toggle_grab(&self) {
        self.left.toggle();
        self.right.toggle();
    }

    pub fn set_grab(&self, engaged: bool) {
        self.left.set(engaged);
        self.right.set(engaged);
    }

    pub async fn climb(&self) {
        self.toggle_raise();
        sleep(Duration::from_secs(1)).await;
        self.toggle_grab();
    }
}