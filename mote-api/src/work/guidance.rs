// This needs some sample guidance routes
// Work: from a given location compute a heading and a distance

use crate::messages::pubsub;

#[derive(Default)]
/// Context for doing the control work
pub struct Context {}

impl Context {
    /// Function for updating the data
    pub fn update(
        &mut self,
        _timestamp: units::Time,
        _position: pubsub::Position,
        _route: &pubsub::Route,
    ) -> Option<pubsub::Guidance> {
        None
    }
}
