//! Reference-structure detection from one accumulated lidar revolution.
//!
//! Both reference structures are isosceles triangles with a 120-degree apex,
//! and each stands at least 0.25 m in front of anything behind it. In a
//! revolution, a structure is therefore a run of adjacent bins that opens with
//! an empty bin or a jump towards the robot and closes with an empty bin or
//! another jump.
//!
//! Walking along a run, the chord angle from its first point to each later
//! point holds steady along the first wall (0, 0, 0, ...) and sweeps once the
//! points turn onto the second wall (10, 20, 30, ...). The apex is the point
//! where that happens. Each wall is then fitted with a line, and the vertex is
//! their intersection.
//!
//! The detector works on the robot-frame points exposed by
//! [`Revolution::points`], keeping the lidar's clockwise angle convention out
//! of the geometry.

use super::{
    ANGLE_TOL, BIN_COUNT, EDGE_JUMP, MAX_RMS, MIN_LEG_RATIO, NE_INTERIOR_ANGLE,
    REVOLUTION_POINT_CAPACITY, Revolution, SW_INTERIOR_ANGLE, is_finite_length, wrap_2pi, wrap_pi,
};

const MIN_WALL_POINTS: usize = 4;
const ZERO_LENGTH: units::Length = units::Length::new(0.0);
const ONE: units::Scalar = units::Scalar::new(1.0);
const TWO: units::Scalar = units::Scalar::new(2.0);

/// The two reference structures that can be identified by the detector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CornerKind {
    /// The southwest reference structure, seen from its convex side.
    Southwest,
    /// The northeast reference structure, seen from its concave side.
    Northeast,
}

/// A geometrically verified reference-corner observation in robot coordinates.
#[derive(Clone, Copy)]
pub struct CornerObservation {
    /// Which reference structure produced this observation.
    pub kind: CornerKind,
    /// Refined vertex in robot-frame forward/left coordinates.
    pub vertex: (units::Length, units::Length),
    /// Range from the robot to the refined vertex.
    pub range_m: units::Length,
    /// Robot-relative bearing, with forward at zero and counter-clockwise positive.
    pub bearing_rad: units::PlaneAngle,
    /// Measured interior angle between the two directed walls.
    pub interior_angle_rad: units::PlaneAngle,
    /// Robot-frame direction of the directed-wall bisector.
    ///
    /// Diagnostic only: the structure's tilt is not controlled, so fixes use
    /// the vertex alone.
    pub bisector_rad: units::PlaneAngle,
    /// A dimensionless quality score in the range 0..1.
    pub score: units::Scalar,
}

#[derive(Clone, Copy)]
struct Point {
    x: units::Length,
    y: units::Length,
    range: units::Length,
    bin: usize,
}

struct Line {
    x: units::Length,
    y: units::Length,
    angle: units::PlaneAngle,
    rms: units::Length,
}

/// Detect the best southwest and northeast reference structures in a revolution.
///
/// At most one observation of each kind is returned.
pub fn detect_corners(revolution: &Revolution) -> heapless::Vec<CornerObservation, 4> {
    let mut points = heapless::Vec::<Point, REVOLUTION_POINT_CAPACITY>::new();
    for (x, y, range, bin) in revolution.points() {
        if is_finite_length(x) && is_finite_length(y) && is_finite_length(range) {
            let _ = points.push(Point { x, y, range, bin });
        }
    }

    let mut southwest: Option<CornerObservation> = None;
    let mut northeast: Option<CornerObservation> = None;
    for start in 0..points.len() {
        if !is_structure_start(&points, start) {
            continue;
        }
        let end = structure_end(&points, start);
        let Some(observation) = observe_structure(&points[start..=end]) else {
            continue;
        };
        let best = match observation.kind {
            CornerKind::Southwest => &mut southwest,
            CornerKind::Northeast => &mut northeast,
        };
        if best.is_none_or(|best| observation.score > best.score) {
            *best = Some(observation);
        }
    }

    let mut result = heapless::Vec::new();
    for observation in [southwest, northeast].into_iter().flatten() {
        let _ = result.push(observation);
    }
    result
}

/// A structure starts at a point whose previous bin is empty or more than
/// [`EDGE_JUMP`] farther away.
///
/// Only bins `1..=BIN_COUNT` may start a structure. Every start in the
/// revolution is then visited exactly once, and a structure crossing the
/// 0-degree seam continues into the overlap bins.
fn is_structure_start(points: &[Point], index: usize) -> bool {
    let point = points[index];
    if point.bin == 0 || point.bin > BIN_COUNT {
        return false;
    }
    match index.checked_sub(1).map(|previous| points[previous]) {
        Some(previous) if previous.bin + 1 == point.bin => previous.range - point.range > EDGE_JUMP,
        _ => true,
    }
}

/// Index of the last point before an empty bin or a range jump.
fn structure_end(points: &[Point], start: usize) -> usize {
    let mut end = start;
    while let Some(next) = points.get(end + 1) {
        let current = points[end];
        if next.bin != current.bin + 1 || (next.range - current.range).abs() > EDGE_JUMP {
            break;
        }
        end += 1;
    }
    end
}

fn observe_structure(run: &[Point]) -> Option<CornerObservation> {
    if run.len() < 2 * MIN_WALL_POINTS + 1 {
        return None;
    }
    let first = run[0];
    let last = run[run.len() - 1];
    let apex = apex_index(run)?;

    // The apex sample straddles both walls, so it is left out of both fits.
    let first_wall = line_fit(&run[..apex])?;
    let second_wall = line_fit(&run[apex + 1..])?;
    if first_wall.rms > MAX_RMS || second_wall.rms > MAX_RMS {
        return None;
    }
    let (vertex_x, vertex_y) = line_intersection(&first_wall, &second_wall)?;

    let first_leg = distance(first.x - vertex_x, first.y - vertex_y);
    let second_leg = distance(last.x - vertex_x, last.y - vertex_y);
    let (shorter, longer) = if first_leg < second_leg {
        (first_leg, second_leg)
    } else {
        (second_leg, first_leg)
    };
    if longer <= ZERO_LENGTH || shorter < longer * MIN_LEG_RATIO {
        return None;
    }

    // Ranges falling towards the apex mean the robot sees the outside of the
    // triangle; rising ranges mean it is looking into it.
    let kind = if run[apex].range < first.range {
        CornerKind::Southwest
    } else {
        CornerKind::Northeast
    };
    let nominal = match kind {
        CornerKind::Southwest => SW_INTERIOR_ANGLE,
        CornerKind::Northeast => NE_INTERIOR_ANGLE,
    };

    let first_ray = direction(first_wall.x - vertex_x, first_wall.y - vertex_y)?;
    let second_ray = direction(second_wall.x - vertex_x, second_wall.y - vertex_y)?;
    let opening = wrap_pi(second_ray - first_ray);
    let interior = opening.abs();
    let angle_error = (interior - nominal).abs();
    if angle_error > ANGLE_TOL {
        return None;
    }

    let range = distance(vertex_x, vertex_y);
    let bearing = direction(vertex_x, vertex_y)?;
    // A perfect 120-degree isosceles triangle scores 1.
    let score = (shorter * (ONE - angle_error / ANGLE_TOL)) / longer;

    Some(CornerObservation {
        kind,
        vertex: (vertex_x, vertex_y),
        range_m: range,
        bearing_rad: bearing,
        interior_angle_rad: interior,
        bisector_rad: wrap_2pi(first_ray + opening / TWO),
        score,
    })
}

/// Find where the chord angle from the first point stops holding steady and
/// starts sweeping.
///
/// That is the point farthest from the base of the triangle, the chord from
/// the first to the last point. A chord of length `c` at angle `a` sits
/// `c * |sin(base - a)|` from the base.
fn apex_index(run: &[Point]) -> Option<usize> {
    let first = run[0];
    let last = run[run.len() - 1];
    let base = direction(last.x - first.x, last.y - first.y)?;

    let mut apex = None;
    let mut height = ZERO_LENGTH;
    for (index, point) in run.iter().enumerate().skip(1) {
        let dx = point.x - first.x;
        let dy = point.y - first.y;
        let Some(angle) = direction(dx, dy) else {
            continue;
        };
        let offset = (distance(dx, dy) * (base - angle).sin()).abs();
        if offset > height {
            height = offset;
            apex = Some(index);
        }
    }
    apex
}

/// Total-least-squares line through `points`.
fn line_fit(points: &[Point]) -> Option<Line> {
    if points.len() < MIN_WALL_POINTS {
        return None;
    }
    let count = units::Scalar::new(points.len() as f32);
    let mean_x = points.iter().fold(ZERO_LENGTH, |sum, point| sum + point.x) / count;
    let mean_y = points.iter().fold(ZERO_LENGTH, |sum, point| sum + point.y) / count;
    let mut xx = units::Area::new(0.0);
    let mut xy = units::Area::new(0.0);
    let mut yy = units::Area::new(0.0);
    for point in points {
        let dx = point.x - mean_x;
        let dy = point.y - mean_y;
        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
    }
    if xx + yy <= units::Area::new(0.0) {
        return None;
    }
    let angle = angle_from_scalar((xy + xy).into_scalar().atan2((xx - yy).into_scalar()) / TWO);
    let mut residual = units::Area::new(0.0);
    for point in points {
        let cross = (point.x - mean_x) * angle.sin() - (point.y - mean_y) * angle.cos();
        residual += cross * cross;
    }
    Some(Line {
        x: mean_x,
        y: mean_y,
        angle,
        rms: (residual / count).sqrt(),
    })
}

fn line_intersection(first: &Line, second: &Line) -> Option<(units::Length, units::Length)> {
    let direction_cross = (second.angle - first.angle).sin();
    if direction_cross.abs() < units::Scalar::new(0.1) {
        return None;
    }
    let between_x = second.x - first.x;
    let between_y = second.y - first.y;
    let between_cross = between_x * second.angle.sin() - between_y * second.angle.cos();
    let distance_along_first = between_cross / direction_cross;
    Some((
        first.x + distance_along_first * first.angle.cos(),
        first.y + distance_along_first * first.angle.sin(),
    ))
}

/// Direction of the vector `(x, y)`, or `None` for a zero-length vector.
fn direction(x: units::Length, y: units::Length) -> Option<units::PlaneAngle> {
    let length = distance(x, y);
    (length > ZERO_LENGTH).then(|| angle_from_scalar((y / length).atan2(x / length)))
}

fn distance(x: units::Length, y: units::Length) -> units::Length {
    (x * x + y * y).sqrt()
}

fn angle_from_scalar(value: units::Scalar) -> units::PlaneAngle {
    units::PlaneAngle::new(1.0) * value
}

trait AreaScalar {
    fn into_scalar(self) -> units::Scalar;
}

impl AreaScalar for units::Area {
    fn into_scalar(self) -> units::Scalar {
        self / units::Area::new(1.0)
    }
}
