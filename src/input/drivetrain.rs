pub async fn control_drivetrain(drivetrain: &mut Drivetrain, controllers: &mut Controllers) {
    if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
        //drivetrain.update_limelight().await;
        drivetrain.post_odo().await;

        if self
            .controllers
            .right_drive
            .get(constants::joystick_map::LINEUP_LEFT)
        {
            drivetrain.lineup(LineupSide::Left).await;
        } else if self
            .controllers
            .right_drive
            .get(constants::joystick_map::LINEUP_RIGHT)
        {
            drivetrain.lineup(LineupSide::Right).await;
        } else {
            drive(&mut drivetrain, &mut self.controllers, drivetrain_state).await;
        }
    }
}