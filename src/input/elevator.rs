pub async fn control_elevator(shooter: &mut Shooter, controllers: &mut Controllers) {
    if let Ok(mut elevator) = self.elevator.try_borrow_mut() {
        // Setting the target position
        if self
            .controllers
            .operator
            .get(constants::joystick_map::SET_TARGET_L2)
        {
            elevator.set_target(ElevatorPosition::L2);
        } else if self
            .controllers
            .operator
            .get(constants::joystick_map::SET_TARGET_L3)
        {
            elevator.set_target(ElevatorPosition::L3);
        } else if self
            .controllers
            .operator
            .get(constants::joystick_map::SET_TARGET_L4)
        {
            elevator.set_target(ElevatorPosition::L4);
        }

        // Setting the actual elevator operation
        if self
            .controllers
            .operator
            .get(constants::joystick_map::ELEVATOR_UP_MANUAL)
        {
            // Manual up
            elevator.set_speed(0.1);
        } else if self
            .controllers
            .operator
            .get(constants::joystick_map::ELEVATOR_DOWN_MANUAL)
        {
            // Manual down
            elevator.set_speed(-0.1);
        } else if self
            .controllers
            .operator
            .get(constants::joystick_map::ELEVATOR_TRAPEZOID_TO_STORED_TARGET)
        {
            // Trapezoidal to stored target
            elevator.run_to_target_trapezoid();
        } else {
            elevator.set_speed(0.0);
        }
    }
}