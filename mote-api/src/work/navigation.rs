// This needs some sample lidar scans
// Work: from the lidar scans determine position (including heading)

use crate::messages::pubsub;

#[derive(Default)]
/// Context for doing the control work
pub struct Context {
    full_scan: heapless::Deque<pubsub::ScanPoint, 200>,
}

impl Context {
    /// Function for updating the data
    pub fn update(
        &mut self,
        _timestamp: units::Time,
        scan: Option<pubsub::Scan>,
        _imu: Option<pubsub::Imu>,
        _reference_frame: &pubsub::ReferenceFrame,
    ) -> Option<pubsub::Position> {
        if let Some(scan) = scan {
            for point in scan.iter() {
                if self.full_scan.is_full() {
                    let _ = self.full_scan.pop_front();
                }
                let _ = self.full_scan.push_back(point.clone());
            }
        }
        None
    }
}
