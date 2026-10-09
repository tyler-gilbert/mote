//! Position and heading fixes from reference-corner observations.
//!
//! Corner observations are expressed in the robot frame: x points forward,
//! y points left, and their bearing is counter-clockwise from forward. The
//! local navigation frame is east/north, while headings use the compass
//! convention (north is zero and clockwise is positive).

use super::{
    CORNER_BISECTOR_OFFSET, CornerKind, CornerObservation, LocalPose, LocalPosition,
    MAX_FIX_DISAGREEMENT, baseline_bearing, is_finite_angle, is_finite_length, is_finite_scalar,
    northeast_local, wrap_2pi,
};
use crate::messages::router;

const EPSILON: units::Scalar = units::Scalar::new(1.0e-6);
const ZERO_LENGTH: units::Length = units::Length::new(0.0);
const HALF: units::Scalar = units::Scalar::new(0.5);

#[derive(Clone, Copy)]
struct ObservationVector {
    x: units::Length,
    y: units::Length,
}

#[derive(Clone, Copy)]
struct SelectedObservation {
    observation: CornerObservation,
    vector: ObservationVector,
}

/// Recover a local position and compass heading from one or two corner
/// observations.
pub fn fix_from_corners(
    frame: &router::ReferenceFrame,
    observations: &[CornerObservation],
) -> Option<LocalPose> {
    let selected = select_observations(observations);
    let southwest = selected[0];
    let northeast = selected[1];

    match (southwest, northeast) {
        (Some(southwest), Some(northeast)) => two_corner_fix(frame, southwest, northeast),
        (Some(observation), None) | (None, Some(observation)) => one_corner_fix(frame, observation),
        (None, None) => None,
    }
}

fn select_observations(observations: &[CornerObservation]) -> [Option<SelectedObservation>; 2] {
    let mut selected: [Option<SelectedObservation>; 2] = [None, None];

    for &observation in observations {
        let Some(vector) = observation_vector(&observation) else {
            continue;
        };
        if !is_finite_scalar(observation.score)
            || !is_finite_length(observation.range_m)
            || !is_finite_angle(observation.bearing_rad)
            || !is_finite_angle(observation.bisector_rad)
            || observation.score < units::Scalar::new(0.0)
            || observation.range_m <= ZERO_LENGTH
        {
            continue;
        }

        let slot = match observation.kind {
            CornerKind::Southwest => &mut selected[0],
            CornerKind::Northeast => &mut selected[1],
        };
        let candidate = SelectedObservation {
            observation,
            vector,
        };
        let replace = slot
            .as_ref()
            .map(|current| observation.score > current.observation.score)
            .unwrap_or(true);
        if replace {
            *slot = Some(candidate);
        }
    }

    selected
}

fn one_corner_fix(
    frame: &router::ReferenceFrame,
    selected: SelectedObservation,
) -> Option<LocalPose> {
    let heading = heading_from_bisector(frame, selected.observation.bisector_rad)?;
    let corner = corner_position(frame, selected.observation.kind);
    let offset = rotate_robot_vector(selected.vector, heading);

    Some(LocalPose {
        position: LocalPosition {
            east: corner.east - offset.x,
            north: corner.north - offset.y,
        },
        heading,
    })
}

fn two_corner_fix(
    frame: &router::ReferenceFrame,
    southwest: SelectedObservation,
    northeast: SelectedObservation,
) -> Option<LocalPose> {
    let southwest_heading = heading_from_bisector(frame, southwest.observation.bisector_rad)?;
    let northeast_heading = heading_from_bisector(frame, northeast.observation.bisector_rad)?;
    let southwest_position = position_from_observation(frame, southwest, southwest_heading);
    let northeast_position = position_from_observation(frame, northeast, northeast_heading);

    let disagreement = distance(
        southwest_position.east - northeast_position.east,
        southwest_position.north - northeast_position.north,
    );
    if disagreement > MAX_FIX_DISAGREEMENT {
        return None;
    }

    let southwest_score = max_scalar(southwest.observation.score, units::Scalar::new(0.0));
    let northeast_score = max_scalar(northeast.observation.score, units::Scalar::new(0.0));
    let total_score = southwest_score + northeast_score;
    let (southwest_weight, northeast_weight) = if total_score > EPSILON {
        (southwest_score / total_score, northeast_score / total_score)
    } else {
        (HALF, HALF)
    };
    let position = LocalPosition {
        east: southwest_position.east * southwest_weight
            + northeast_position.east * northeast_weight,
        north: southwest_position.north * southwest_weight
            + northeast_position.north * northeast_weight,
    };

    let baseline = northeast_local(frame);
    let baseline_length = distance(baseline.east, baseline.north);
    if baseline_length <= ZERO_LENGTH {
        return None;
    }

    let sw_to_ne_robot = ObservationVector {
        x: northeast.vector.x - southwest.vector.x,
        y: northeast.vector.y - southwest.vector.y,
    };
    let sw_to_ne_length = distance(sw_to_ne_robot.x, sw_to_ne_robot.y);
    if sw_to_ne_length <= ZERO_LENGTH {
        return None;
    }
    let robot_bearing = angle_from_atan2(
        sw_to_ne_robot.y / sw_to_ne_length,
        sw_to_ne_robot.x / sw_to_ne_length,
    );
    let heading = wrap_2pi(baseline_bearing(frame) + robot_bearing);

    Some(LocalPose { position, heading })
}

fn position_from_observation(
    frame: &router::ReferenceFrame,
    selected: SelectedObservation,
    heading: units::PlaneAngle,
) -> LocalPosition {
    let corner = corner_position(frame, selected.observation.kind);
    let offset = rotate_robot_vector(selected.vector, heading);
    LocalPosition {
        east: corner.east - offset.x,
        north: corner.north - offset.y,
    }
}

fn corner_position(frame: &router::ReferenceFrame, kind: CornerKind) -> LocalPosition {
    match kind {
        CornerKind::Southwest => LocalPosition::default(),
        CornerKind::Northeast => northeast_local(frame),
    }
}

fn heading_from_bisector(
    frame: &router::ReferenceFrame,
    robot_bisector: units::PlaneAngle,
) -> Option<units::PlaneAngle> {
    Some(wrap_2pi(
        baseline_bearing(frame) + CORNER_BISECTOR_OFFSET + robot_bisector,
    ))
}

/// Rotate a robot-frame (forward, left) vector into local (east, north)
/// coordinates using a compass heading.
fn rotate_robot_vector(vector: ObservationVector, heading: units::PlaneAngle) -> ObservationVector {
    ObservationVector {
        x: vector.x * heading.sin() - vector.y * heading.cos(),
        y: vector.x * heading.cos() + vector.y * heading.sin(),
    }
}

fn observation_vector(observation: &CornerObservation) -> Option<ObservationVector> {
    let range = observation.range_m;
    if range <= ZERO_LENGTH {
        return None;
    }
    Some(ObservationVector {
        x: range * observation.bearing_rad.cos(),
        y: range * observation.bearing_rad.sin(),
    })
}

fn distance(x: units::Length, y: units::Length) -> units::Length {
    (x * x + y * y).sqrt()
}

fn angle_from_atan2(y: units::Scalar, x: units::Scalar) -> units::PlaneAngle {
    units::PlaneAngle::new(1.0) * y.atan2(x)
}

fn max_scalar(first: units::Scalar, second: units::Scalar) -> units::Scalar {
    if first > second { first } else { second }
}
