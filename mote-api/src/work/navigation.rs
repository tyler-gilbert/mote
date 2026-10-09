// This needs some sample lidar scans
// Work: from the lidar scans determine position (including heading)

use crate::messages::router;

mod corner;
mod dead_reckoning;
mod fix;
mod frame;

pub use corner::{CornerKind, CornerObservation, detect_corners};
pub use fix::fix_from_corners;
pub use frame::{
    LocalPose, LocalPosition, METRES_PER_DEGREE, baseline_bearing, heading_to_degrees,
    lidar_to_robot, northeast_local, to_global, to_local, wrap_2pi, wrap_pi,
};

/// Scalar conversion from sensor milliseconds to seconds.
pub const MS_PER_S: units::Scalar = units::Scalar::new(1_000.0);

/// Mean earth radius, retained for the eventual frame-conversion implementation.
pub const EARTH_RADIUS_M: units::Length = units::Length::new(6_371_000.0);

/// Interior angle of the southwest reference structure.
pub const SW_INTERIOR_ANGLE: units::PlaneAngle =
    units::PlaneAngle::new(2.0 * core::f32::consts::PI / 3.0);

/// Interior angle of the northeast reference structure.
///
/// Both structures are 120-degree isosceles triangles; the robot sees the
/// northeast one from its concave side.
pub const NE_INTERIOR_ANGLE: units::PlaneAngle =
    units::PlaneAngle::new(2.0 * core::f32::consts::PI / 3.0);

/// C1: lidar angle zero is aligned with the robot's forward axis.
pub const LIDAR_ZERO_IS_FORWARD: bool = true;

/// C1: lidar angles increase clockwise when viewed from above.
pub const LIDAR_CLOCKWISE: bool = true;

/// C2: position headings use the compass convention: north is zero and clockwise is positive.
pub const HEADING_IS_COMPASS: bool = true;

/// C2: local north is aligned with true north in the flat-earth frame.
pub const LOCAL_NORTH_IS_TRUE_NORTH: bool = true;

/// C5: the gyro z-axis reports positive angular velocity for counter-clockwise yaw.
pub const GYRO_Z_CCW_POSITIVE: bool = true;

/// Maximum interval represented by one dead-reckoning integration step.
pub const MAX_DT: units::Time = units::Time::new(100.0);

/// Minimum interval between emitted dead-reckoning positions.
pub const DR_OUTPUT_PERIOD_MS: units::Time = units::Time::new(100.0);

/// Minimum accepted lidar return range.
pub const MIN_RANGE: units::Length = units::Length::new(0.05);

/// Maximum accepted lidar return range.
pub const MAX_RANGE: units::Length = units::Length::new(6.0);

/// Range jump between adjacent bins that marks the edge of a reference structure.
///
/// The structures stand at least 0.25 m in front of anything behind them; the
/// threshold sits below that to tolerate range noise, and well above the
/// bin-to-bin range change along a wall.
pub const EDGE_JUMP: units::Length = units::Length::new(0.2);

/// Smallest accepted ratio of the shorter to the longer leg of a structure.
pub const MIN_LEG_RATIO: units::Scalar = units::Scalar::new(0.7);

/// Maximum number of points exposed by one binned revolution, including overlap bins.
pub const REVOLUTION_POINT_CAPACITY: usize = EXTENDED_BIN_COUNT;

/// Largest acceptable total-least-squares wall-fit residual.
pub const MAX_RMS: units::Length = units::Length::new(0.02);

/// Maximum deviation from a structure's nominal interior angle.
pub const ANGLE_TOL: units::PlaneAngle =
    units::PlaneAngle::new(15.0 * core::f32::consts::PI / 180.0);

/// Number of one-degree bins used to represent a lidar revolution.
pub const BIN_COUNT: usize = 360;

/// Number of bins past 360 degrees that repeat the start of the revolution.
///
/// A reference structure crossing the 0-degree seam is then contiguous in
/// the extended bins. A structure with 0.2 m legs subtends roughly 40 degrees
/// at half a metre, so a quarter revolution covers it.
pub const OVERLAP_BIN_COUNT: usize = 90;

/// Total number of bins stored per revolution, including the overlap.
pub const EXTENDED_BIN_COUNT: usize = BIN_COUNT + OVERLAP_BIN_COUNT;

/// Minimum number of populated angular bins for a complete revolution.
pub const MIN_BINS: u16 = 150;

/// Maximum disagreement between position fixes from the two corners.
///
/// Both fixes share the baseline heading, so this bounds the difference between
/// the measured and expected vertex separation.
pub const MAX_FIX_DISAGREEMENT: units::Length = units::Length::new(0.05);

/// Maximum allowed position jump when fusing a fix with dead reckoning.
pub const MAX_JUMP: units::Length = units::Length::new(0.5);

const MAX_FINITE: units::Scalar = units::Scalar::new(core::f32::MAX);
const MIN_FINITE: units::Scalar = units::Scalar::new(-core::f32::MAX);

const ONE_DEGREE: units::PlaneAngle = units::PlaneAngle::new(core::f32::consts::PI / 180.0);
const HALF_BIN: units::PlaneAngle = units::PlaneAngle::new(core::f32::consts::PI / 360.0);

pub(crate) fn is_finite_scalar(value: units::Scalar) -> bool {
    value >= MIN_FINITE && value <= MAX_FINITE
}

pub(crate) fn is_finite_length(value: units::Length) -> bool {
    is_finite_scalar(value / units::Length::new(1.0))
}

pub(crate) fn is_finite_angle(value: units::PlaneAngle) -> bool {
    is_finite_scalar(value / units::PlaneAngle::new(1.0))
}

pub(crate) fn is_finite_time(value: units::Time) -> bool {
    is_finite_scalar(value / units::Time::new(1.0))
}

/// A lidar revolution accumulated into uniformly spaced angle bins.
///
/// Bins `BIN_COUNT..EXTENDED_BIN_COUNT` mirror bins `0..OVERLAP_BIN_COUNT`, so
/// structures crossing the 0-degree seam can be detected without reordering.
pub struct Revolution {
    range_m: [units::Length; EXTENDED_BIN_COUNT],
    last_angle: units::PlaneAngle,
    filled: u16,
    start_timestamp: units::Time,
}

impl Revolution {
    fn new(start_timestamp: units::Time) -> Self {
        Self {
            range_m: [units::Length::new(0.0); EXTENDED_BIN_COUNT],
            last_angle: units::PlaneAngle::new(0.0),
            filled: 0,
            start_timestamp,
        }
    }

    /// Build a binned revolution from unit-typed lidar samples.
    pub fn from_points(
        start_timestamp: units::Time,
        points: &[(units::PlaneAngle, units::Length)],
    ) -> Self {
        let mut revolution = Self::new(start_timestamp);
        for &(angle, distance) in points {
            if is_finite_angle(angle)
                && is_finite_length(distance)
                && distance > units::Length::new(0.0)
            {
                revolution.insert(wrap_2pi(angle), distance);
            }
        }
        revolution
    }

    fn insert(&mut self, angle: units::PlaneAngle, distance: units::Length) {
        let angle = wrap_2pi(angle);
        let bin = angle_bin(angle);
        let previous = self.range_m[bin];
        if previous <= units::Length::new(0.0) {
            self.filled += 1;
            self.range_m[bin] = distance;
        } else if distance < previous {
            self.range_m[bin] = distance;
        }
        if bin < OVERLAP_BIN_COUNT {
            self.range_m[bin + BIN_COUNT] = self.range_m[bin];
        }
        self.last_angle = angle;
    }

    /// Number of populated bins in this revolution, excluding overlap bins.
    pub const fn filled_bins(&self) -> u16 {
        self.filled
    }

    /// Timestamp of the first accepted point in this revolution.
    pub const fn start_timestamp(&self) -> units::Time {
        self.start_timestamp
    }

    /// Return the populated bins as robot-frame points in increasing lidar-angle order.
    ///
    /// The sequence continues through the overlap bins, whose indices are
    /// `BIN_COUNT` or greater and repeat the points of the first
    /// [`OVERLAP_BIN_COUNT`] bins.
    pub fn points(&self) -> RevolutionPoints<'_> {
        RevolutionPoints {
            revolution: self,
            next_bin: 0,
        }
    }
}

/// Iterator over the populated points in a [`Revolution`].
pub struct RevolutionPoints<'a> {
    revolution: &'a Revolution,
    next_bin: usize,
}

impl Iterator for RevolutionPoints<'_> {
    type Item = (units::Length, units::Length, units::Length, usize);

    fn next(&mut self) -> Option<Self::Item> {
        while self.next_bin < EXTENDED_BIN_COUNT {
            let bin = self.next_bin;
            self.next_bin += 1;
            let range = self.revolution.range_m[bin];
            if range <= units::Length::new(0.0) {
                continue;
            }

            let angle = bin_angle(bin % BIN_COUNT);
            let (x, y) = lidar_to_robot(angle, range);
            return Some((x, y, range, bin));
        }
        None
    }
}

fn angle_bin(angle: units::PlaneAngle) -> usize {
    let target = wrap_2pi(angle + HALF_BIN);
    let mut lower = units::PlaneAngle::new(0.0);
    for bin in 0..BIN_COUNT {
        let upper = lower + ONE_DEGREE;
        if target < upper {
            return bin;
        }
        lower = upper;
    }
    0
}

fn bin_angle(bin: usize) -> units::PlaneAngle {
    ONE_DEGREE * units::Scalar::new(bin as f32)
}

/// The result of processing one accepted lidar revolution.
pub struct NavigationReport {
    /// Timestamp of the first accepted point in the revolution.
    pub timestamp: units::Time,
    /// Number of populated bins in the revolution.
    pub filled_bins: u16,
    /// Geometrically verified corner observations.
    pub corners: heapless::Vec<CornerObservation, 4>,
    /// The fix accepted from this revolution, if one was available.
    pub pose: Option<router::Position>,
}

/// Context for doing the navigation work.
#[derive(Default)]
pub struct Context {
    current_revolution: Option<Revolution>,
    completed: heapless::Deque<Revolution, 4>,
    reports: heapless::Deque<NavigationReport, 4>,
    revolution_wraps: u32,
    accepted_revolutions: u32,
    discarded_revolutions: u32,
    accepted_points: u32,
    discarded_points: u32,
    dead_reckoning: Option<dead_reckoning::DeadReckoning>,
    last_accepted_fix_timestamp: Option<units::Time>,
}

impl Context {
    /// Advance navigation with an optional LiDAR scan and IMU sample.
    pub fn update(
        &mut self,
        timestamp: units::Time,
        position: Option<&router::Position>,
        scan: Option<router::Scan>,
        imu: Option<router::Imu>,
        reference_frame: &router::ReferenceFrame,
    ) -> Option<router::Position> {
        if self.dead_reckoning.is_none() {
            if let Some(position) = position {
                self.dead_reckoning = Some(dead_reckoning::DeadReckoning::from_position(
                    position,
                    reference_frame,
                ));
            }
        }

        let dead_reckoning_position = imu.and_then(|imu| {
            self.dead_reckoning
                .as_mut()
                .and_then(|dead_reckoning| dead_reckoning.update(timestamp, imu, reference_frame))
        });

        let Some(scan) = scan else {
            return dead_reckoning_position;
        };

        let accepted_revolutions_before = self.accepted_revolutions;
        for point in scan.point_cloud {
            if !is_finite_angle(point.angle)
                || !is_finite_length(point.distance)
                || point.distance < MIN_RANGE
                || point.distance > MAX_RANGE
            {
                self.discarded_points += 1;
                continue;
            }

            self.accepted_points += 1;
            let angle = wrap_2pi(point.angle);
            let wraps = self
                .current_revolution
                .as_ref()
                .map(|revolution| {
                    revolution.last_angle - angle > units::PlaneAngle::new(core::f32::consts::PI)
                })
                .unwrap_or(false);
            if wraps {
                self.finish_revolution(timestamp, reference_frame);
            }

            self.current_revolution
                .get_or_insert_with(|| Revolution::new(timestamp))
                .insert(angle, point.distance);
        }

        let fix_position = (self.accepted_revolutions > accepted_revolutions_before)
            .then(|| self.reports.back().and_then(|report| report.pose.clone()))
            .flatten();
        fix_position.or(dead_reckoning_position)
    }

    fn finish_revolution(
        &mut self,
        timestamp: units::Time,
        reference_frame: &router::ReferenceFrame,
    ) {
        let Some(revolution) = self.current_revolution.take() else {
            return;
        };
        self.revolution_wraps += 1;
        if revolution.filled_bins() < MIN_BINS {
            self.discarded_revolutions += 1;
            return;
        }

        self.accepted_revolutions += 1;
        let report = self.process_revolution(&revolution, timestamp, reference_frame);
        if let Err(revolution) = self.completed.push_back(revolution) {
            let _ = self.completed.pop_front();
            let _ = self.completed.push_back(revolution);
        }
        if let Err(report) = self.reports.push_back(report) {
            let _ = self.reports.pop_front();
            let _ = self.reports.push_back(report);
        }
    }

    fn process_revolution(
        &mut self,
        revolution: &Revolution,
        timestamp: units::Time,
        reference_frame: &router::ReferenceFrame,
    ) -> NavigationReport {
        let corners = detect_corners(revolution);
        let prior_heading = self
            .dead_reckoning
            .as_ref()
            .map(|dead_reckoning| dead_reckoning.pose().heading);
        let pose = fix_from_corners(reference_frame, &corners, prior_heading)
            .and_then(|fix| self.accept_fix(fix, timestamp, reference_frame));
        NavigationReport {
            timestamp: revolution.start_timestamp,
            filled_bins: revolution.filled_bins(),
            corners,
            pose,
        }
    }

    fn accept_fix(
        &mut self,
        fix: LocalPose,
        timestamp: units::Time,
        reference_frame: &router::ReferenceFrame,
    ) -> Option<router::Position> {
        let pose = if let Some(dead_reckoning) = self.dead_reckoning.as_ref() {
            let prior = dead_reckoning.pose();
            let has_recent_fix = self
                .last_accepted_fix_timestamp
                .map(|last| timestamp - last <= units::Time::new(5_000.0))
                .unwrap_or(false);
            if has_recent_fix
                && (local_distance(prior.position, fix.position) > MAX_JUMP
                    || heading_difference(prior.heading, fix.heading)
                        > units::PlaneAngle::new(core::f32::consts::FRAC_PI_4))
            {
                return None;
            }
            if self.last_accepted_fix_timestamp.is_some() {
                blend_pose(prior, fix, units::Scalar::new(0.7), units::Scalar::new(0.5))
            } else {
                fix
            }
        } else {
            fix
        };

        let dead_reckoning = self
            .dead_reckoning
            .get_or_insert_with(|| dead_reckoning::DeadReckoning::from_local_pose(pose));
        dead_reckoning.set_pose(pose);
        self.last_accepted_fix_timestamp = Some(timestamp);
        Some(dead_reckoning.position(timestamp, reference_frame))
    }

    /// Remove the oldest accepted revolution, if one is waiting for processing.
    pub fn take_completed_revolution(&mut self) -> Option<Revolution> {
        self.completed.pop_front()
    }

    /// Remove the oldest integrated navigation report, if one is waiting.
    pub fn take_navigation_report(&mut self) -> Option<NavigationReport> {
        self.reports.pop_front()
    }

    /// View the revolution currently being assembled, if any.
    pub fn current_revolution(&self) -> Option<&Revolution> {
        self.current_revolution.as_ref()
    }

    /// Number of angle-wrap boundaries observed in the input stream.
    pub const fn revolution_wraps(&self) -> u32 {
        self.revolution_wraps
    }

    /// Number of wrapped revolutions that met [`MIN_BINS`].
    pub const fn accepted_revolutions(&self) -> u32 {
        self.accepted_revolutions
    }

    /// Number of wrapped revolutions discarded for insufficient angular coverage.
    pub const fn discarded_revolutions(&self) -> u32 {
        self.discarded_revolutions
    }

    /// Number of lidar points accepted after quality and range filtering.
    pub const fn accepted_points(&self) -> u32 {
        self.accepted_points
    }

    /// Number of lidar points rejected by quality or range filtering.
    pub const fn discarded_points(&self) -> u32 {
        self.discarded_points
    }
}

fn local_distance(first: LocalPosition, second: LocalPosition) -> units::Length {
    ((first.east - second.east) * (first.east - second.east)
        + (first.north - second.north) * (first.north - second.north))
        .sqrt()
}

fn heading_difference(first: units::PlaneAngle, second: units::PlaneAngle) -> units::PlaneAngle {
    wrap_pi(first - second).abs()
}

fn blend_pose(
    dead_reckoned: LocalPose,
    fix: LocalPose,
    position_alpha: units::Scalar,
    heading_alpha: units::Scalar,
) -> LocalPose {
    let east = dead_reckoned.position.east
        + (fix.position.east - dead_reckoned.position.east) * position_alpha;
    let north = dead_reckoned.position.north
        + (fix.position.north - dead_reckoned.position.north) * position_alpha;
    let heading_error = wrap_pi(fix.heading - dead_reckoned.heading);
    LocalPose {
        position: LocalPosition { east, north },
        heading: wrap_2pi(dead_reckoned.heading + heading_error * heading_alpha),
    }
}
