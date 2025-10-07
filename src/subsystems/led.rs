use crate::constants::robotmap;
use crate::subsystems::Vision;
use ::frcrs::led::Led;
use frcrs::limelight::Limelight;
use std::cell::RefCell;
use std::cmp::PartialEq;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::task::{spawn_local, AbortHandle};
use tokio::time::sleep;
use uom::si::length::Units::meter;

#[derive(Clone, PartialEq)]
pub enum LedStatus {
    Disabled,
    GotCoral,
    LinedUp,
}

#[derive(Clone)]
pub struct LedSubsystem {
    led: Led,
    handle: Option<AbortHandle>,
    current_state: LedStatus,
    limelight: Vision,
}

impl LedSubsystem {
    pub fn init() -> Self {
        let led = Led::new(robotmap::led::PORT, robotmap::led::COUNT);
        let limelight = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        Self {
            led,
            handle: None,
            current_state: LedStatus::Disabled,
            limelight: limelight,
        }
    }

    pub fn set_state(&mut self, state: LedStatus) {
        if state == self.current_state {
            return;
        } else {
            self.current_state = state;
        }

        if let Some(handle) = self.handle.take() {
            handle.abort();
        }

        let mut led_clone = self.led.clone();

        let future = match self.current_state {
            LedStatus::Disabled => tokio::spawn(async move {
                LedSubsystem::disabled(&mut led_clone).await;
            }),
            LedStatus::GotCoral => tokio::spawn(async move {
                LedSubsystem::got_coral(&mut led_clone).await;
            }),
            LedStatus::LinedUp => tokio::spawn(async move {
                LedSubsystem::lined_up(&mut led_clone).await;
            }),
        };

        let handle = spawn_local(future).abort_handle();

        self.handle = Some(handle);
    }

    async fn disabled(led: &mut Led) {
        loop {
            for i in (1..=robotmap::led::COUNT) {
                led.set_rgb(i, 255, 0, 0);
            }

            for i in (1..=robotmap::led::COUNT) {
                led.set_rgb(i, 0, 0, 0);
            }

            led.set_data();
            sleep(Duration::from_secs_f64(0.5)).await;
        }
    }

    async fn got_coral(led: &mut Led) {
        for i in (1..=robotmap::led::COUNT) {
            led.set_rgb(i, 255, 239, 0);
        }

        led.set_data();
    }

    async fn lined_up(led: &mut Led) {
        for i in (1..=robotmap::led::COUNT) {
            led.set_rgb(i, 0, 255, 0);
        }

        led.set_data();
    }

    async fn fom_leds(&self, led: &mut Led) {
        let fom_raw = self
            .limelight
            .get_figure_of_merit()
            .get::<uom::si::length::meter>();

        let fom_scaled = (fom_raw * 10.0).clamp(1.0, 10.0);
        let fom_int = fom_scaled.round() as i32;
        let total_leds = robotmap::led::COUNT;
        let leds_to_light = (fom_int * total_leds) / 10;

        for i in 1..=total_leds {
            if i <= leds_to_light {
                led.set_rgb(i, 0, 255, 0);
            } else {
                led.set_rgb(i, 0, 0, 0);
            }
        }
    }
}
