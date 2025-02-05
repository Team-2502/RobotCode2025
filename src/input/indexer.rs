pub async fn control_indexer(indexer: &mut Indexer, controllers: &mut Controllers) {
    if let Ok(indexer) = self.indexer.try_borrow_mut() {
        if self
            .controllers
            .left_drive
            .get(constants::joystick_map::INDEXER_OUT)
        {
            // Out the front
            indexer.set_speed(0.3);
        } else if self
            .controllers
            .left_drive
            .get(constants::joystick_map::INDEXER_IN)
        {
            // In, score out the left
            indexer.set_speed(-0.5);
        } else {
            indexer.set_speed(0.0);
        }
    }
}