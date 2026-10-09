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

/// Angular velocity measurements along the three axes.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Gyro {
    /// Angular velocity around the x-axis.
    pub x: units::AngularVelocity,
    /// Angular velocity around the y-axis.
    pub y: units::AngularVelocity,
    /// Angular velocity around the z-axis.
    pub z: units::AngularVelocity,
}

/// Inertial measurement unit data containing acceleration and angular velocity.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Imu {
    /// Timestamp
    pub timestamp: units::Time,
    /// Linear acceleration measurements.
    pub accel: Accel,
    /// Angular velocity measurements.
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

/// Lidar point cloud
pub type PointCloud = heapless::vec::Vec<ScanPoint, 100>;

/// Waypoint to navigate to
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Scan {
    /// Timestamp
    pub timestamp: units::Time,
    /// Lidar point cloud
    pub point_cloud: PointCloud,
}

/// Waypoint to navigate to
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Coordinate {
    /// Latitude waypoint position
    pub latitude: units::imperial::Degrees,
    /// Longitude waypoint position
    pub longitude: units::imperial::Degrees,
}

impl Default for Coordinate {
    fn default() -> Self {
        Self {
            latitude: units::imperial::Degrees::new(0.0),
            longitude: units::imperial::Degrees::new(0.0),
        }
    }
}

/// A collection of points captured during a scan.
pub type Route = heapless::vec::Vec<Coordinate, 100>;

/// Guidance describing a target heading and distance.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Guidance {
    /// Timestamp
    pub timestamp: units::Time,
    /// Heading toward the target.
    pub heading: units::PlaneAngle,
    /// Distance to the target.
    pub distance: units::Length,
}

impl Default for Guidance {
    fn default() -> Self {
        Self {
            timestamp: units::Time::new(0.0),
            heading: units::PlaneAngle::new(0.0),
            distance: units::Length::new(0.0),
        }
    }
}

/// Position expressed as north/east offsets and a heading.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq, Default)]
pub struct ReferenceFrame {
    /// Postion of the southwest reference point
    pub southwest: Coordinate,
    /// Position of the northeast reference point
    pub northeast: Coordinate,
    /// Lidar distance scale factor
    pub scale: f32,
}

/// Position expressed as north/east offsets and a heading.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Position {
    /// Timestamp
    pub timestamp: units::Time,
    /// Northward position offset.
    pub coordinate: Coordinate,
    /// Heading in degrees.
    pub heading: units::imperial::Degrees,
}

impl Default for Position {
    fn default() -> Self {
        Self {
            timestamp: units::Time::new(0.0),
            coordinate: Default::default(),
            heading: units::imperial::Degrees::new(0.0),
        }
    }
}

/// Angular velocity commands for the left and right sides.
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct Control {
    /// Timestamp
    pub timestamp: units::Time,
    /// Motor drive settings
    pub motor_drive: MotorDrive,
}

/// Telemetry selection for pubsub messages
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq, Default)]
pub enum TelemetrySelection {
    /// Do not send any pubsub telemetry
    #[default]
    None,
    /// Send pubsub control telemetry
    Control,
    /// Send pubsub guidance telemetry
    Guidance,
    /// Send pubsub navigation telemetry
    Navigation,
    /// Send pubsub imu telemetry
    Imu,
    /// Send data for remote operation
    RemoteControl,
    /// Send pubsub lidar telemetry
    Lidar,
    /// Position Inputs
    PositionInputs,
    /// All
    All
}

impl TelemetrySelection {
    /// Send control telemtry
    pub fn is_send_control(&self) -> bool {
        *self == Self::Control || *self == Self::RemoteControl || *self == Self::All
    }

    /// Send guidance telemtry
    pub fn is_send_guidance(&self) -> bool {
        *self == Self::Guidance || *self == Self::RemoteControl|| *self == Self::All
    }

    /// Send nav telemtry
    pub fn is_send_navigation(&self) -> bool {
        *self == Self::Navigation || *self == Self::RemoteControl|| *self == Self::All
    }

    /// Send imu telemtry
    pub fn is_send_imu(&self) -> bool {
        *self == Self::Imu || *self == Self::RemoteControl || *self == Self::PositionInputs|| *self == Self::All
    }

    /// Send lidar telemtry
    pub fn is_send_lidar(&self) -> bool {
        *self == Self::Lidar || *self == Self::PositionInputs|| *self == Self::All
    }
}

/// Direct motor drive command
#[derive(Clone, defmt::Format, Serialize, Deserialize, PartialEq)]
pub struct MotorDrive {
    /// left drive speed
    pub left: units::AngularVelocity,
    /// right drive speed
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
    /// Enable pubsub telemetry
    TelemetrySelection(TelemetrySelection),
    /// Reference frame
    ReferenceFrame(ReferenceFrame),
    /// Motor Drive
    MotorDrive(MotorDrive),
    /// Guidance to a target.
    Guidance(Guidance),
    /// Position estimate.
    Position(Position),
    /// Left and right angular velocity commands.
    Control(Control),
    /// Inertial measurement data.
    Imu(Imu),
    /// A batch of LiDAR scan points.
    LidarScan(Scan),
    /// Guidance Route
    Route(Route),
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

    /// Wraps LiDAR scan data in a message.
    pub fn new_from_lidar_scan(value: Scan) -> Self {
        Self::LidarScan(value)
    }
}
