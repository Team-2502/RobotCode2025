use crate::constants::robotmap;
use ::frcrs::led::Led;

pub struct led {
    led: Led,
}

impl led {
    pub fn init() -> Self {
        let led = Led::new(robotmap::led::PORT, robotmap::led::COUNT);
        Self { led }
    }

    pub fn set(&self, idx: i32, r: i32, g: i32, b: i32) {
        self.led.set_rgb(idx, r, g, b);
    }

    pub fn flush(&self) {
        self.led.flush();
    }

    //level = what branch level robot is at, set led strip to some fraction of total depending on branch
    pub async fn elevator_led_update(&self, level: i32) {
        self.led.flush().await;

        match level {
            1 => self.solid_height(1),
            2 => self.solid_height(2),
            3 => self.solid_height(3),
            4 => self.solid_height(4),
            _ => println!("level {} not supported", level),
        }
    }

    fn solid_height(&self, level: i32) {
        for i in { robotmap::led::COUNT } * { level / 4 } {
            self.led.set_rgb(i, 0, 255, 0);
        }
    }
    //want to make fn that will adjust ledstrip according to reefside limelight detections
    //pub fn limelight_led_update(&self, ) {}
}
