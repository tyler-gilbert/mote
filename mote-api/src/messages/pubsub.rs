use serde::{Deserialize, Serialize};

/// Acceleration measurements along the three axes.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Accel {
    /// Acceleration along the x-axis.
    pub x: units::Acceleration,
    /// Acceleration along the y-axis.
    pub y: units::Acceleration,
    /// Acceleration along the z-axis.
    pub z: units::Acceleration,
}

/// Angular acceleration measurements along the three axes.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Gyro {
    /// Angular acceleration around the x-axis.
    pub x: units::AngularAcceleration,
    /// Angular acceleration around the y-axis.
    pub y: units::AngularAcceleration,
    /// Angular acceleration around the z-axis.
    pub z: units::AngularAcceleration,
}

/// Inertial measurement unit data containing acceleration and angular acceleration.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Imu {
    /// Linear acceleration measurements.
    pub accel: Accel,
    /// Angular acceleration measurements.
    pub gyro: Gyro,
}

/// A single measured point in a scan.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct ScanPoint {
    /// Quality value reported for this measurement.
    pub quality: u8,
    /// Angle of the measurement.
    pub angle: units::PlaneAngle,
    /// Distance measured at this angle.
    pub distance: units::Length,
}

/// A collection of points captured during a scan.
pub type Scan = heapless::vec::Vec<ScanPoint, 100>;

/// Guidance describing a target heading and distance.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Guidance {
    /// Heading toward the target.
    pub heading: units::PlaneAngle,
    /// Distance to the target.
    pub distance: units::Length,
}

impl Default for Guidance {
    fn default() -> Self {
        Self {
            heading: units::PlaneAngle::new(0.0),
            distance: units::Length::new(0.0),
        }
    }
}

/// Position expressed as north/east offsets and a heading.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Position {
    /// Northward position offset.
    pub north: units::Length,
    /// Eastward position offset.
    pub east: units::Length,
    /// Heading in degrees.
    pub heading: units::imperial::Degrees,
}

impl Default for Position {
    fn default() -> Self {
        Self {
            north: units::Length::new(0.0),
            east: units::Length::new(0.0),
            heading: units::imperial::Degrees::new(0.0),
        }
    }
}

/// Angular velocity commands for the left and right sides.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Control {
    /// Angular velocity command for the left side.
    pub left: units::AngularVelocity,
    /// Angular velocity command for the right side.
    pub right: units::AngularVelocity,
}

/// A message published between mote components.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub enum Message {
    /// No message or no available data.
    None,
    /// Request to enable control mode.
    EnableControlMode,
    /// Request to disable control mode.
    DisableControlMode,
    /// Guidance to a target.
    Guidance(Guidance),
    /// Position estimate.
    Position(Position),
    /// Left and right angular velocity commands.
    Control(Control),
    /// Inertial measurement data.
    Imu(Imu),
}

impl alloc::fmt::Debug for Message {
    fn fmt(&self, _formatter: &mut alloc::fmt::Formatter) -> alloc::fmt::Result {
        Ok(())
    }
}

impl Message {
    /// Wraps guidance data in a message.
    pub fn new_from_guidance(value: Guidance) -> Self {
        Self::Guidance(value)
    }

    /// Wraps position data in a message.
    pub fn new_from_position(value: Position) -> Self {
        Self::Position(value)
    }

    /// Wraps control data in a message.
    pub fn new_from_control(value: Control) -> Self {
        Self::Control(value)
    }
}
