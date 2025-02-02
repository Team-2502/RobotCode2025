use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use crate::constants::robotmap;
use ::frcrs::led::Led;
use tokio::task::{AbortHandle, spawn_local};
use tokio::time::sleep;

pub enum LedStatus {
    Disabled
}

#[derive(Clone)]
pub struct LedSubsystem {
    led: Led,
    handle: Option<AbortHandle>,
}

impl LedSubsystem {
    pub fn init() -> Self {
        let led = Led::new(robotmap::led::PORT, robotmap::led::COUNT);

        Self {
            led,
            handle: None
        }
    }

    pub fn set_state(&mut self, state: LedStatus) {
        if let Some(handle) = self.handle.take() {
            handle.abort();
        }

        let mut led_clone = self.led.clone();

        let future = match state {
            LedStatus::Disabled => {
                async move {
                    LedSubsystem::disabled(&mut led_clone).await;
                }
            }
        };

        let handle = spawn_local(future).abort_handle();

        self.handle = Some(handle);
    }

    async fn disabled(led: &mut Led) {
        loop {
            for i in (0..=robotmap::led::COUNT).step_by(2) {
                led.set_rgb(i, 255, 0, 0);
            }

            for i in (1..=robotmap::led::COUNT).step_by(2) {
                led.set_rgb(i, 0, 0, 0);
            }

            led.set_data();
            sleep(Duration::from_secs_f64(0.5)).await;
        }
    }
}
