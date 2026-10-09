//! Coordinate and frame-convention helpers for navigation.
//!
//! The navigation frame is a flat, scaled local tangent plane. Its origin is
//! the southwest reference vertex, its x axis points east, and its y axis
//! points north. The scale supplied by the router frame is world metres per
//! lidar metre.

use crate::messages::router;

/// Metres in one degree in the flat-earth frame used by navigation.
pub const METRES_PER_DEGREE: units::Scalar = units::Scalar::new(111_319.5);

const TWO_PI: units::PlaneAngle = units::PlaneAngle::new(2.0 * core::f32::consts::PI);
const PI: units::PlaneAngle = units::PlaneAngle::new(core::f32::consts::PI);

/// Local tangent-plane position in lidar metres, with the SW vertex as origin.
#[derive(Clone, Copy)]
pub struct LocalPosition {
    /// Eastward offset from the SW vertex, in lidar metres.
    pub east: units::Length,
    /// Northward offset from the SW vertex, in lidar metres.
    pub north: units::Length,
}

impl Default for LocalPosition {
    fn default() -> Self {
        Self {
            east: units::Length::new(0.0),
            north: units::Length::new(0.0),
        }
    }
}

/// Local pose: a position and a compass-convention heading.
#[derive(Clone, Copy)]
pub struct LocalPose {
    /// Local position in lidar metres.
    pub position: LocalPosition,
    /// Compass-convention heading, in radians.
    pub heading: units::PlaneAngle,
}

impl Default for LocalPose {
    fn default() -> Self {
        Self {
            position: LocalPosition::default(),
            heading: units::PlaneAngle::new(0.0),
        }
    }
}

/// Convert a global coordinate to local lidar metres relative to the SW vertex.
pub fn to_local(frame: &router::ReferenceFrame, coordinate: &router::Coordinate) -> LocalPosition {
    let latitude_delta =
        degrees_as_scalar(coordinate.latitude) - degrees_as_scalar(frame.southwest.latitude);
    let longitude_delta =
        degrees_as_scalar(coordinate.longitude) - degrees_as_scalar(frame.southwest.longitude);
    let scale = units::Scalar::from(frame.scale);

    let north_world = units::Length::new(1.0) * latitude_delta * METRES_PER_DEGREE;
    let east_world = units::Length::new(1.0) * longitude_delta * METRES_PER_DEGREE;

    LocalPosition {
        east: east_world / scale,
        north: north_world / scale,
    }
}

/// Convert a local lidar-metre position back to the global flat-earth frame.
pub fn to_global(frame: &router::ReferenceFrame, local: LocalPosition) -> router::Coordinate {
    let scale = units::Scalar::from(frame.scale);
    let metres_per_degree = units::Length::new(1.0) * METRES_PER_DEGREE;
    let east_degrees = (local.east * scale) / metres_per_degree;
    let north_degrees = (local.north * scale) / metres_per_degree;

    router::Coordinate {
        latitude: units::imperial::Degrees::new(1.0)
            * (degrees_as_scalar(frame.southwest.latitude) + north_degrees),
        longitude: units::imperial::Degrees::new(1.0)
            * (degrees_as_scalar(frame.southwest.longitude) + east_degrees),
    }
}

/// Return the NE vertex in local lidar metres relative to the SW vertex.
pub fn northeast_local(frame: &router::ReferenceFrame) -> LocalPosition {
    to_local(frame, &frame.northeast)
}

/// Return the compass bearing from the SW vertex to the NE vertex.
pub fn baseline_bearing(frame: &router::ReferenceFrame) -> units::PlaneAngle {
    let baseline = northeast_local(frame);
    let east = baseline.east / units::Length::new(1.0);
    let north = baseline.north / units::Length::new(1.0);
    wrap_2pi(angle_from_scalar(east.atan2(north)))
}

/// Convert a lidar sample to robot-frame coordinates.
///
/// Lidar angles are zero-forward and clockwise-positive. Robot coordinates
/// are x-forward and y-left, so the y component has the opposite sign.
pub fn lidar_to_robot(
    angle: units::PlaneAngle,
    distance: units::Length,
) -> (units::Length, units::Length) {
    let x = distance * angle.cos();
    let y = distance * angle.sin() * units::Scalar::new(-1.0);
    (x, y)
}

/// Wrap an angle to (-π, π].
pub fn wrap_pi(angle: units::PlaneAngle) -> units::PlaneAngle {
    let wrapped = wrap_2pi(angle);
    if wrapped > PI {
        wrapped - TWO_PI
    } else {
        wrapped
    }
}

/// Wrap an angle to [0, 2π).
pub fn wrap_2pi(angle: units::PlaneAngle) -> units::PlaneAngle {
    let mut wrapped = angle;
    while wrapped >= TWO_PI {
        wrapped -= TWO_PI;
    }
    while wrapped < units::PlaneAngle::new(0.0) {
        wrapped += TWO_PI;
    }
    wrapped
}

/// Convert a radian compass heading to normalized compass degrees.
pub fn heading_to_degrees(angle: units::PlaneAngle) -> units::imperial::Degrees {
    units::imperial::Degrees::from(wrap_2pi(angle))
}

fn degrees_as_scalar(degrees: units::imperial::Degrees) -> units::Scalar {
    degrees / units::imperial::Degrees::new(1.0)
}

fn angle_from_scalar(value: units::Scalar) -> units::PlaneAngle {
    units::PlaneAngle::new(1.0) * value
}
