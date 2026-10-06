// This needs some sample lidar scans
// Work: from the lidar scans determine position (including heading)

use crate::messages::router;

#[derive(Default)]
/// Context for doing the control work
pub struct Context {
    full_scan: heapless::Deque<router::ScanPoint, 200>,
}

impl Context {
    /// Function for updating the data
    pub fn update(
        &mut self,
        _timestamp: units::Time,
        _scan: Option<router::Scan>,
        _imu: Option<router::Imu>,
        _reference_frame: &router::ReferenceFrame,
    ) -> Option<router::Position> {
        None
    }
}
