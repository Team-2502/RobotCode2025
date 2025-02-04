pub const FPS_LIMIT: f64 = 250.;

pub mod robotmap {
    pub mod swerve {
        pub const PIGEON: i32 = 9;

        pub const FR_DRIVE: i32 = 1;
        pub const FR_TURN: i32 = 2;

        pub const FL_DRIVE: i32 = 4;
        pub const FL_TURN: i32 = 3;

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
        pub const MOTOR: i32 = 12;
        pub const LASER_CAN: i32 = 13;
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
    pub const LIMELIGHT_YAW_DEGREES: f64 = 180.;
    pub const LIMELIGHT_HEIGHT_INCHES: f64 = 11.75;
    pub const ROBOT_CENTER_TO_LIMELIGHT_INCHES: Vector2<f64> = Vector2::new(-14.45, 0.);
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


    pub const LINEUP_2D_TX_KP: f64 = 0.01;
    pub const LINEUP_2D_TY_KP: f64 = -0.0025;
    pub const LINEUP_2D_TX_KS: f64 = 0.0;
    pub const LINEUP_2D_TY_KS: f64 = 0.0;
    pub const LINEUP_2D_TX_KD: f64 = 0.0;
    pub const LINEUP_2D_TY_KD: f64 = 0.0;

}
pub mod elevator {
    pub const BOTTOM: f64 = 0.0; // unit is rotations
    pub const L2: f64 = 1.; // unit is rotations
    pub const L3: f64 = 15.75; // unit is rotations
    pub const L4: f64 = 39.7; // unit is rotations
    pub const ELEVATOR_TRAPEZOID_DT_MS: u64 = 50; // sleep.await this long in between updating the elevator trapezoidal when running its async function
    pub const POSITION_TOLERANCE: f64 = 0.25; // unit is rotations. finish elevator async move when within this distance of target
}
pub mod indexer {
    pub const LASER_TRIP_DISTANCE_MM: i32 = 100;
}
pub mod joystick_map {
    // Joystick IDs (set in driver station)
    pub const RIGHT_DRIVE: i32 = 0;
    pub const LEFT_DRIVE: i32 = 1;
    pub const OPERATOR: i32 = 2;

    //Right drive
    pub const LINEUP_LEFT: usize = 3;
    pub const LINEUP_RIGHT: usize = 4;

    //Left drive
    pub const INDEXER_IN: usize = 1;
    pub const INDEXER_OUT: usize = 2;

    //Operator
    pub const ELEVATOR_TRAPEZOID_TO_STORED_TARGET: usize = 1;
    pub const ELEVATOR_TRAPEZOID_TO_STORED_TARGET_ASYNC: usize = 2;
    pub const ELEVATOR_UP_MANUAL: usize = 3;
    pub const ELEVATOR_DOWN_MANUAL: usize = 4;
    pub const CLIMB_FULL: usize = 8;
    pub const SET_TARGET_L2: usize = 14;
    pub const SET_TARGET_L3: usize = 15;
    pub const SET_TARGET_L4: usize = 16;
}
