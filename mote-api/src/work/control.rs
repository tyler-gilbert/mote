// This needs some type of simulator
// Work: from a guidance point, adjust the motors to advance to the guidance point

use crate::messages::pubsub;

#[derive(Default)]
/// Context for doing the control work
pub struct Context {}

impl Context {
    /// Function for updating the data
    pub fn update(
        &mut self,
        _timestamp: units::Time,
        _guidance_message: pubsub::Guidance,
    ) -> Option<pubsub::Control> {
        None
    }
}
