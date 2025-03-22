use uom::si::f64::Length;
use uom::si::length::meter;

pub const FPS_LIMIT: f64 = 250.;

pub mod robotmap {
    pub mod swerve {
        pub const PIGEON: i32 = 9;

        pub const FR_DRIVE: i32 = 1;
        pub const FR_TURN: i32 = 2;
        pub const FR_ENCODER: i32 = 13;

        pub const FL_DRIVE: i32 = 4;
        pub const FL_TURN: i32 = 3;
        pub const FL_ENCODER: i32 = 14;

        pub const BL_DRIVE: i32 = 5;
        pub const BL_TURN: i32 = 6;
        pub const BL_ENCODER: i32 = 15;

        pub const BR_DRIVE: i32 = 7;
        pub const BR_TURN: i32 = 8;
        pub const BR_ENCODER: i32 = 16;

        pub const RIGHT_LINEUP_LASER:i32 = 18;
        pub const LEFT_LINEUP_LASER:i32 = 19;
    }

    pub mod elevator {
        pub const LEFT: i32 = 10;
        pub const RIGHT: i32 = 11;
    }

    pub mod indexer {
        pub const MOTOR: i32 = 12;
        pub const LASER_CAN: i32 = 0; // Cant save can id
        pub const INDEXER_775_MOTOR: i32 = 17;
    }

    pub mod climber {
        pub const CLIMBER_MOTOR_ID: i32 = 20;
    }

    pub mod led {
        pub const PORT: i32 = 1;
        pub const COUNT: i32 = 0;
    }
}

// TODO: get 2025 field dimensions
pub const HALF_FIELD_WIDTH_METERS: f64 = 17.55 / 2.;
pub const HALF_FIELD_LENGTH_METERS: f64 = 8.05 / 2.;

pub mod vision {
    use nalgebra::Vector2;

    pub const LIMELIGHT_UPPER_PITCH_DEGREES: f64 = -36.4; //Last measured: -37.3 past -37.0
    pub const LIMELIGHT_UPPER_YAW_DEGREES: f64 = 90.; // Counterclockwise positive
    pub const LIMELIGHT_UPPER_HEIGHT_INCHES: f64 = 20.92;
    pub const ROBOT_CENTER_TO_LIMELIGHT_UPPER_INCHES: Vector2<f64> = Vector2::new(11.118, 10.352);

    // Increase distance by 13.5% for every 20 degrees of absolute value of tx
    // Set this to 0 for new robots
    pub const TX_FUDGE_FACTOR: f64 = 0.135 / 20.;
}

pub mod drivetrain {
    use std::f64::consts::PI;

    pub const FR_OFFSET_DEGREES: f64 = 0.081299 * 360.;
    pub const FL_OFFSET_DEGREES: f64 = 0.0942 * 360.;
    pub const BR_OFFSET_DEGREES: f64 = -0.056641 * 360.;
    pub const BL_OFFSET_DEGREES: f64 = 0.170898 * 360.;

    pub const PIGEON_OFFSET: f64 = -0.4;

    pub const SWERVE_TURN_KP: f64 = 0.6;

    pub const SWERVE_ROTATIONS_TO_INCHES: f64 = (1. / 6.75) * (3.65 * PI);
    pub const SWERVE_TURN_RATIO: f64 = 12.8;

    pub const SWERVE_DRIVE_KP: f64 = 0.7;
    pub const SWERVE_DRIVE_KI: f64 = 2.;
    pub const SWERVE_DRIVE_KD: f64 = 50.;
    pub const SWERVE_DRIVE_KF: f64 = 0.; // Velocity ff
    pub const SWERVE_DRIVE_KFA: f64 = 0.; // Acceleration ff

    pub const SWERVE_DRIVE_MAX_ERR: f64 = 0.15;
    pub const SWERVE_DRIVE_SUGGESTION_ERR: f64 = 0.35;
    pub const SWERVE_DRIVE_IE: f64 = 0.175; //0.175; // integral enable

    pub const LINEUP_2D_TX_STR_KP: f64 = 0.005;
    pub const LINEUP_2D_TX_FWD_KP: f64 = 0.005;
    pub const LINEUP_2D_TY_STR_KP: f64 = 0.005;
    pub const LINEUP_2D_TY_FWD_KP: f64 = 0.005;
    pub const TARGET_TY_LEFT: f64 = -7.4;
    pub const TARGET_TY_RIGHT: f64 = 3.45;
    pub const TARGET_TX_LEFT: f64 = -7.4;
    pub const TARGET_TX_RIGHT: f64 = 3.45;
    pub const TX_ACCEPTABLE_ERROR: f64 = 1.8;
    pub const TY_ACCEPTABLE_ERROR: f64 = 1.8;
    pub const YAW_ACCEPTABLE_ERROR: f64 = 0.02;

    pub const LINEUP_DRIVE_KP: f64 = 1.;
    pub const LINEUP_DRIVE_KI: f64 = 10.;
    pub const LINEUP_DRIVE_KD: f64 = 10.;
    pub const LINEUP_DRIVE_IE: f64 = 0.25;

    pub const CANRANGE_DEBOUNCE_TIME_SECONDS: f64 = 0.02;
    pub const REEF_SENSOR_TARGET_DISTANCE_METERS: f64 = 0.381;
}
pub mod pose_estimation {
    pub const ARC_ODOMETRY_MINIMUM_DELTA_THETA_RADIANS: f64 = 0.000001;
    pub const MIN_FOM: f64 = 0.00001;
    pub const START_POSITION_FOM: f64 = 0.01; //meters
    pub const DRIFT_RATIO: f64 = 0.02; // Robot odometry pose estimate drifts [ratio] meters for every meter driven
    pub const LIMELIGHT_BASE_FOM: f64 = 0.001; // meters
    pub const LIMELIGHT_INACCURACY_PER_DEGREE_TX: f64 = 0.015; // Meters of inaccuracy per degree of tx absolute value (our limelights are miscalibrated)
    pub const LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY: f64 = 2.; // Meters of inaccuracy per (radian/second) of drivetrain angular velocity
    pub const LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY: f64 = 2.; //  Meters of inaccuracy per (meter/second) of drivetrain linear velocity
}
pub mod elevator {
    pub const BOTTOM: f64 = 0.25; // unit is rotations
    pub const L2: f64 = 0.05; // unit is rotations
    pub const L3: f64 = 13.; // unit is rotations
    pub const L4: f64 = 42.9; // unit is rotations
    pub const ELEVATOR_TRAPEZOID_DT_MS: u64 = 50; // sleep.await this long in between updating the elevator trapezoidal when running its async function
    pub const POSITION_TOLERANCE: f64 = 0.25; // unit is rotations. finish elevator async move when within this distance of target
}
pub mod indexer {
    pub const LASER_TRIP_DISTANCE_MM: i32 = 2;
    pub const INDEXER_LASER_DEBOUNCE_TIME_SECONDS: f64 = 0.08;
    pub const INTAKE_SPEED: f64 = -0.25;
    pub const BOTTOM_SPEED: f64 = -0.35;
    pub const L2_SPEED: f64 = -0.2;
    pub const L3_SPEED: f64 = -0.4;
    pub const L4_SPEED: f64 = -0.4;
}
pub mod climber {
    pub const CLIMB_SPEED: f64 = 0.3;
    pub const FALL_SPEED: f64 = -0.3;
}
pub mod joystick_map {
    // Joystick IDs (set in driver station)
    pub const RIGHT_DRIVE: i32 = 0;
    pub const LEFT_DRIVE: i32 = 1;
    pub const OPERATOR: i32 = 2;

    //Right drive
    pub const LINEUP_LEFT: usize = 3;
    pub const LINEUP_RIGHT: usize = 4;
    pub const INTAKE: usize = 1;
    pub const RESET_HEADING: usize = 5;
    pub const CLIMB: usize = 2;
    pub const CLIMB_FALL: usize = 6;

    //Left drive
    pub const SLOW_MODE: usize = 1;

    pub const SCORE_L2: usize = 2;
    pub const SCORE_L3: usize = 3;
    pub const SCORE_L4: usize = 4;

    //Operator
    pub const ELEVATOR_TRAPEZOID_TO_STORED_TARGET: usize = 1;
    pub const ELEVATOR_TRAPEZOID_TO_STORED_TARGET_ASYNC: usize = 2;
    pub const ELEVATOR_UP_MANUAL: usize = 3;
    pub const ELEVATOR_DOWN_MANUAL: usize = 4;
    pub const CLIMB_FULL: usize = 8;
    pub const SET_TARGET_L2: usize = 14;
    pub const SET_TARGET_L3: usize = 15;
    pub const SET_TARGET_L4: usize = 16;
    pub const WHEELS_ZERO: usize = 5;
}
