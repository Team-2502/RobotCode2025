pub const FPS_LIMIT: f64 = 250.;

pub mod robotmap {
    pub mod swerve {
        pub const PIGEON: i32 = 9;

        pub const FR_DRIVE: i32 = 1;
        pub const FR_TURN: i32 = 2;

        pub const FL_DRIVE: i32 = 3;
        pub const FL_TURN: i32 = 4;

        pub const BL_DRIVE: i32 = 5;
        pub const BL_TURN: i32 = 6;

        pub const BR_DRIVE: i32 = 7;
        pub const BR_TURN: i32 = 8;
    }

    pub mod elevator {
        pub const LEFT: i32 = 10;
        pub const RIGHT: i32 = 11;
    }

    pub mod indexer {
        pub const MOTOR: i32 = 3;
    }

    pub mod climber {
        pub const RAISE: i32 = 0;
        pub const GRAB: i32 = 1;
    }
}

// TODO: get 2025 field dimensions
pub const HALF_FIELD_WIDTH_METERS: f64 = 4.1148; // 54/4 feet
pub const HALF_FIELD_LENGTH_METERS: f64 = 8.2296; // 54/2 feet

pub mod vision {
    use nalgebra::Vector2;

    pub const LIMELIGHT_PITCH_DEGREES: f64 = 18.1;
    pub const LIMELIGHT_YAW_DEGREES:f64 = 180.;
    pub const LIMELIGHT_HEIGHT_INCHES: f64 = 11.75;
    pub const ROBOT_CENTER_TO_LIMELIGHT_INCHES: Vector2<f64> = Vector2::new(
        -14.45,
        0.
    );
}

pub mod drivetrain {
    use std::f64::consts::PI;

    pub const SWERVE_TURN_KP: f64 = 0.3;

    pub const SWERVE_ROTATIONS_TO_INCHES: f64 = (1. / 5.906) * (4. * PI);

    pub const SWERVE_DRIVE_KP: f64 = 0.3;
    pub const SWERVE_DRIVE_KI: f64 = 0.;
    pub const SWERVE_DRIVE_KD: f64 = 0.;
    pub const SWERVE_DRIVE_KF: f64 = 0.; // Velocity ff
    pub const SWERVE_DRIVE_KFA: f64 = 0.; // Acceleration ff

    pub const SWERVE_DRIVE_MAX_ERR: f64 = 0.15;
    pub const SWERVE_DRIVE_SUGGESTION_ERR: f64 = 0.35;
    pub const SWERVE_DRIVE_IE: f64 = 0.0; //0.175; // integral enable

    pub const PODIUM_SHOT_ANGLE: f64 = 34.34; // degrees
}
pub mod elevator{
    pub const BOTTOM: f64 = 0.0;
    pub const L2: f64 = 0.0;
    pub const L3: f64 = 0.0;
    pub const L4: f64 = 39.7;
}