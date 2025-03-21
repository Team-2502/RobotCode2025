use crate::constants::drivetrain::SWERVE_TURN_KP;
use crate::constants::joystick_map::*;
use crate::subsystems::{Drivetrain, DrivetrainControlState, SwerveControlStyle};
use crate::Controllers;
use frcrs::{alliance_station, deadzone};
use frcrs::input::RobotState;
use nalgebra::ComplexField;
use uom::si::angle::{degree, radian};

// pub async fn teleop(controllers: &mut Controllers, robot: &mut Ferris, executor: &LocalSet, dt: Duration) {
//     let TeleopState {
//         ref mut drivetrain_state,
//     } = *robot.teleop_state.deref().borrow_mut();
//
//     if let Ok(mut drivetrain) = robot.drivetrain.try_borrow_mut() {
//         drivetrain.update_limelight().await;
//         drivetrain.post_odo().await;
//
//         if controllers.right_drive.get(3) {
//             drivetrain.lineup(LineupSide::Left).await;
//         } else if controllers.right_drive.get(4) {
//             drivetrain.lineup(LineupSide::Right).await;
//         } else {
//             control_drivetrain(&mut drivetrain, controllers, drivetrain_state).await;
//         }
//     }
// }

pub async fn control_drivetrain(
    drivetrain: &mut Drivetrain,
    controllers: &mut Controllers,
    state: &mut DrivetrainControlState,
) {
    let right_drive = &mut controllers.right_drive;
    let left_drive = &mut controllers.left_drive;
    let saved_angle = &mut state.saved_angle;

    let joystick_range = 0.04..1.;
    let power_translate = if left_drive.get(SLOW_MODE) {
        0.0..0.3
    } else {
        0.0..1.
    };
    let power_rotate = if left_drive.get(SLOW_MODE) {
        0.0..0.2
    } else {
        0.0..1.
    };
    let mut deadly = deadzone(left_drive.get_y(), &joystick_range, &power_translate);
    let mut deadlx = deadzone(left_drive.get_x(), &joystick_range, &power_translate);
    let deadrz = deadzone(right_drive.get_z(), &joystick_range, &power_rotate);

    let hold_angle = deadrz == 0. && right_drive.get(3);

    if !hold_angle {
        *saved_angle = Some(drivetrain.get_angle());
    }

    let rot = if hold_angle {
        if let Some(saved_angle) = (saved_angle).as_ref() {
            let error = drivetrain.get_angle() - *saved_angle;
            -error.get::<radian>() * SWERVE_TURN_KP
        } else {
            0.
        }
    }
    /*else if right_drive.get(HOLD_90) {
        let angle = (drivetrain.get_angle() - drivetrain.offset).get::<degree>();
        let goal = (angle / 90.).round() * 90.;
        let error = angle - goal;
        -error.to_radians() * SWERVE_TURN_KP
    } */
    else {
        deadrz
    };

    // Flip because the driver is facing the other way
    if alliance_station().red() {
        deadlx *= -1.;
        deadly *= -1.;
    }

    drivetrain.set_speeds(deadly, deadlx, rot, SwerveControlStyle::FieldOriented);

    if right_drive.get(RESET_HEADING) {
        drivetrain.reset_heading();
    }
}
