//! Reference-corner detection from one accumulated lidar revolution.
//!
//! The detector deliberately works on the robot-frame points exposed by
//! [`Revolution::points`]. This keeps the lidar's clockwise angle convention
//! out of the geometry code and makes the convex/concave prior explicit.

use super::{
    ANGLE_TOL, EXTREMUM_DEPTH, EXTREMUM_MARGIN, MAX_CLUSTER_GAP, MAX_RANGE, MAX_RMS, MIN_SCORE,
    NE_INTERIOR_ANGLE, REVOLUTION_POINT_CAPACITY, Revolution, SW_INTERIOR_ANGLE, VERTEX_SNAP,
    WALL_FIT_LEN, WALL_FIT_MIN_PATH, is_finite_length,
};

const MAX_CLUSTERS: usize = 32;
const MAX_WALL_POINTS: usize = 64;
const MIN_CLUSTER_POINTS: usize = 12;
const MIN_WALL_POINTS: usize = 5;
const ROBOT_X: units::Length = units::Length::new(0.0);
const ROBOT_Y: units::Length = units::Length::new(0.0);
const ZERO_LENGTH: units::Length = units::Length::new(0.0);

const ONE: units::Scalar = units::Scalar::new(1.0);
const TWO: units::Scalar = units::Scalar::new(2.0);
const ONE_DEGREE: units::PlaneAngle = units::PlaneAngle::new(core::f32::consts::PI / 180.0);

/// The two reference structures that can be identified by the detector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CornerKind {
    /// The convex 120-degree corner at the southwest reference vertex.
    Southwest,
    /// The concave 60-degree corner at the northeast reference vertex.
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

#[derive(Clone, Copy)]
struct Direction {
    angle: units::PlaneAngle,
}

struct LineFit {
    centroid: Point,
    direction: Direction,
    rms: units::Length,
}

struct WallFit {
    line: LineFit,
    points: heapless::Vec<Point, MAX_WALL_POINTS>,
    path_length: units::Length,
}

struct Candidate {
    observation: CornerObservation,
}

/// Detect the best SW and NE reference-corner observations in a revolution.
///
/// A revolution is first rotated at its largest angular gap, so a real
/// structure crossing the raw 0-degree bin is not split. Each continuous
/// cluster is then tested using both range extrema and two independent line
/// fits. At most one observation of each kind is returned.
pub fn detect_corners(revolution: &Revolution) -> heapless::Vec<CornerObservation, 4> {
    let mut points = heapless::Vec::<Point, REVOLUTION_POINT_CAPACITY>::new();
    for (x, y, range, bin) in revolution.points() {
        if is_finite_length(x)
            && is_finite_length(y)
            && is_finite_length(range)
            && x >= ZERO_LENGTH - MAX_RANGE
            && x <= MAX_RANGE
            && y >= ZERO_LENGTH - MAX_RANGE
            && y <= MAX_RANGE
            && range >= ZERO_LENGTH
        {
            let _ = points.push(Point { x, y, range, bin });
        }
    }

    let mut ordered = heapless::Vec::<Point, REVOLUTION_POINT_CAPACITY>::new();
    if !points.is_empty() {
        let start = largest_gap_start(&points);
        for offset in 0..points.len() {
            let index = (start + offset) % points.len();
            let _ = ordered.push(points[index]);
        }
    }

    let clusters = clusters(&ordered);
    let mut best_sw: Option<Candidate> = None;
    let mut best_ne: Option<Candidate> = None;

    for &(start, end) in &clusters {
        if end - start < MIN_CLUSTER_POINTS {
            continue;
        }
        for seed in (start + EXTREMUM_MARGIN)..(end - EXTREMUM_MARGIN) {
            if is_extremum(&ordered, seed, start, end, false)
                && let Some(candidate) =
                    verify_seed(&ordered, start, end, seed, CornerKind::Southwest)
            {
                replace_if_better(&mut best_sw, candidate);
            }
            if is_extremum(&ordered, seed, start, end, true)
                && let Some(candidate) =
                    verify_seed(&ordered, start, end, seed, CornerKind::Northeast)
            {
                replace_if_better(&mut best_ne, candidate);
            }
        }
    }

    let mut result = heapless::Vec::new();
    if let Some(candidate) = best_sw {
        if candidate.observation.score >= MIN_SCORE {
            let _ = result.push(candidate.observation);
        }
    }
    if let Some(candidate) = best_ne {
        if candidate.observation.score >= MIN_SCORE {
            let _ = result.push(candidate.observation);
        }
    }
    result
}

fn largest_gap_start(points: &[Point]) -> usize {
    if points.len() < 2 {
        return 0;
    }
    let mut largest_gap = 0usize;
    let mut largest_index = 0usize;
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let gap = bin_gap(points[index].bin, points[next].bin);
        if gap > largest_gap {
            largest_gap = gap;
            largest_index = index;
        }
    }
    (largest_index + 1) % points.len()
}

fn clusters(points: &[Point]) -> heapless::Vec<(usize, usize), MAX_CLUSTERS> {
    let mut result = heapless::Vec::new();
    if points.is_empty() {
        return result;
    }

    let mut start = 0;
    for index in 1..points.len() {
        let previous = points[index - 1];
        let current = points[index];
        let jump_limit = max_length(
            units::Length::new(0.15),
            max_length(previous.range, current.range) * units::Scalar::new(0.10),
        );
        if bins_angle(bin_gap(previous.bin, current.bin)) > MAX_CLUSTER_GAP
            || (previous.range - current.range).abs() > jump_limit
        {
            if index - start >= MIN_CLUSTER_POINTS {
                let _ = result.push((start, index));
            }
            start = index;
        }
    }
    if points.len() - start >= MIN_CLUSTER_POINTS {
        let _ = result.push((start, points.len()));
    }
    result
}

fn is_extremum(points: &[Point], index: usize, start: usize, end: usize, maximum: bool) -> bool {
    if index < start + EXTREMUM_MARGIN || index + EXTREMUM_MARGIN >= end {
        return false;
    }
    let point = points[index].range;
    let previous = points[index - 1].range;
    let next = points[index + 1].range;
    let at_extremum = if maximum {
        point >= previous && point >= next
    } else {
        point <= previous && point <= next
    };
    if !at_extremum {
        return false;
    }

    let before = points[index - EXTREMUM_MARGIN].range;
    let after = points[index + EXTREMUM_MARGIN].range;
    if maximum {
        point - before >= EXTREMUM_DEPTH && point - after >= EXTREMUM_DEPTH
    } else {
        before - point >= EXTREMUM_DEPTH && after - point >= EXTREMUM_DEPTH
    }
}

fn verify_seed(
    points: &[Point],
    cluster_start: usize,
    cluster_end: usize,
    seed: usize,
    kind: CornerKind,
) -> Option<Candidate> {
    let left = fit_wall(points, cluster_start, seed, seed, -1)?;
    let right = fit_wall(points, seed + 1, cluster_end, seed, 1)?;
    if left.line.rms > MAX_RMS || right.line.rms > MAX_RMS {
        return None;
    }

    let (vertex_x, vertex_y) = line_intersection(&left.line, &right.line)?;
    let seed_point = points[seed];
    if distance(vertex_x - seed_point.x, vertex_y - seed_point.y) > VERTEX_SNAP {
        return None;
    }

    let ray_left = directed_ray(&left.points, vertex_x, vertex_y)?;
    let ray_right = directed_ray(&right.points, vertex_x, vertex_y)?;
    let interior = super::wrap_pi(ray_right.angle - ray_left.angle).abs();
    let bisector_delta = super::wrap_pi(ray_right.angle - ray_left.angle) / TWO;
    if bisector_delta.cos().abs() <= units::Scalar::new(1.0e-6) {
        return None;
    }
    let bisector = super::wrap_2pi(ray_left.angle + bisector_delta);

    let outside_wedge =
        (ROBOT_X - vertex_x) * bisector.cos() + (ROBOT_Y - vertex_y) * bisector.sin();
    let nominal = match kind {
        CornerKind::Southwest => SW_INTERIOR_ANGLE,
        CornerKind::Northeast => NE_INTERIOR_ANGLE,
    };
    let angle_error = (interior - nominal).abs();
    let convexity_ok = match kind {
        CornerKind::Southwest => outside_wedge < ZERO_LENGTH,
        CornerKind::Northeast => outside_wedge > ZERO_LENGTH,
    };
    if angle_error >= ANGLE_TOL || !convexity_ok {
        return None;
    }

    let range = distance(vertex_x, vertex_y);
    if range <= ZERO_LENGTH {
        return None;
    }
    let angle_score = ONE - angle_error / ANGLE_TOL;
    let rms_score = ONE - max_length(left.line.rms, right.line.rms) / MAX_RMS;
    let wall_score = min_scalar(
        (left.path_length + right.path_length) / units::Length::new(0.5),
        ONE,
    );
    let score = scalar_product(units::Scalar::new(0.40), wall_score)
        + scalar_product(units::Scalar::new(0.35), angle_score)
        + scalar_product(units::Scalar::new(0.25), rms_score);

    Some(Candidate {
        observation: CornerObservation {
            kind,
            vertex: (vertex_x, vertex_y),
            range_m: range,
            bearing_rad: angle_from_atan2(vertex_y / range, vertex_x / range),
            interior_angle_rad: interior,
            bisector_rad: bisector,
            score: min_scalar(max_scalar(score, units::Scalar::new(0.0)), ONE),
        },
    })
}

fn fit_wall(
    points: &[Point],
    start: usize,
    end: usize,
    seed: usize,
    direction: isize,
) -> Option<WallFit> {
    let mut selected = heapless::Vec::<Point, MAX_WALL_POINTS>::new();
    let mut path_length = ZERO_LENGTH;
    let mut previous = points[seed];
    let mut index = seed as isize + direction;

    while index >= start as isize && index < end as isize {
        let current = points[index as usize];
        let jump_limit = max_length(
            units::Length::new(0.15),
            max_length(previous.range, current.range) * units::Scalar::new(0.10),
        );
        let angular_gap = if direction < 0 {
            bin_gap(current.bin, previous.bin)
        } else {
            bin_gap(previous.bin, current.bin)
        };
        if bins_angle(angular_gap) > MAX_CLUSTER_GAP
            || (previous.range - current.range).abs() > jump_limit
        {
            break;
        }
        path_length += distance(current.x - previous.x, current.y - previous.y);
        if path_length > WALL_FIT_LEN {
            break;
        }
        if path_length >= WALL_FIT_MIN_PATH {
            if selected.push(current).is_err() {
                break;
            }
        }
        previous = current;
        index += direction;
    }

    if selected.len() < MIN_WALL_POINTS {
        return None;
    }
    let line = line_fit(&selected)?;
    Some(WallFit {
        line,
        points: selected,
        path_length,
    })
}

fn line_fit(points: &[Point]) -> Option<LineFit> {
    if points.len() < MIN_WALL_POINTS {
        return None;
    }
    let mut count = units::Scalar::new(0.0);
    for _ in points {
        count += units::Scalar::new(1.0);
    }
    let mean_x = points
        .iter()
        .map(|point| point.x)
        .fold(ZERO_LENGTH, |sum, value| sum + value)
        / count;
    let mean_y = points
        .iter()
        .map(|point| point.y)
        .fold(ZERO_LENGTH, |sum, value| sum + value)
        / count;
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
    let theta = (xy + xy).into_scalar().atan2((xx - yy).into_scalar());
    let direction = Direction {
        angle: angle_from_scalar(theta),
    };
    let mut residual = units::Area::new(0.0);
    for point in points {
        let dx = point.x - mean_x;
        let dy = point.y - mean_y;
        let cross = dx * direction.angle.sin() - dy * direction.angle.cos();
        residual += cross * cross;
    }
    let rms = (residual / count).sqrt();
    Some(LineFit {
        centroid: Point {
            x: mean_x,
            y: mean_y,
            range: ZERO_LENGTH,
            bin: 0,
        },
        direction,
        rms,
    })
}

fn line_intersection(first: &LineFit, second: &LineFit) -> Option<(units::Length, units::Length)> {
    let direction_cross = (second.direction.angle - first.direction.angle).sin();
    if direction_cross.abs() < units::Scalar::new(0.1) {
        return None;
    }
    let between_x = second.centroid.x - first.centroid.x;
    let between_y = second.centroid.y - first.centroid.y;
    let between_cross =
        between_x * second.direction.angle.sin() - between_y * second.direction.angle.cos();
    let distance_along_first = between_cross / direction_cross;
    Some((
        first.centroid.x + distance_along_first * first.direction.angle.cos(),
        first.centroid.y + distance_along_first * first.direction.angle.sin(),
    ))
}

fn directed_ray(
    points: &[Point],
    vertex_x: units::Length,
    vertex_y: units::Length,
) -> Option<Direction> {
    let mut farthest = None;
    let mut farthest_distance = units::Area::new(0.0);
    for point in points {
        let dx = point.x - vertex_x;
        let dy = point.y - vertex_y;
        let squared = dx * dx + dy * dy;
        if squared > farthest_distance {
            farthest_distance = squared;
            farthest = Some((dx, dy));
        }
    }
    let (x, y) = farthest?;
    let length = distance(x, y);
    (length > ZERO_LENGTH).then_some(Direction {
        angle: angle_from_atan2(y / length, x / length),
    })
}

fn replace_if_better(slot: &mut Option<Candidate>, candidate: Candidate) {
    let should_replace = slot
        .as_ref()
        .map(|current| candidate.observation.score > current.observation.score)
        .unwrap_or(true);
    if should_replace {
        *slot = Some(candidate);
    }
}

fn bin_gap(first: usize, second: usize) -> usize {
    (second + super::BIN_COUNT - first) % super::BIN_COUNT
}

fn bins_angle(count: usize) -> units::PlaneAngle {
    let mut result = units::PlaneAngle::new(0.0);
    for _ in 0..count {
        result += ONE_DEGREE;
    }
    result
}

fn distance(x: units::Length, y: units::Length) -> units::Length {
    (x * x + y * y).sqrt()
}

fn scalar_product(first: units::Scalar, second: units::Scalar) -> units::Scalar {
    (units::Length::new(1.0) * first * second) / units::Length::new(1.0)
}

fn angle_from_atan2(y: units::Scalar, x: units::Scalar) -> units::PlaneAngle {
    units::PlaneAngle::new(1.0) * y.atan2(x)
}

fn angle_from_scalar(value: units::Scalar) -> units::PlaneAngle {
    units::PlaneAngle::new(1.0) * value
}

fn max_length(first: units::Length, second: units::Length) -> units::Length {
    if first > second { first } else { second }
}

fn max_scalar(first: units::Scalar, second: units::Scalar) -> units::Scalar {
    if first > second { first } else { second }
}

fn min_scalar(first: units::Scalar, second: units::Scalar) -> units::Scalar {
    if first < second { first } else { second }
}

trait AreaScalar {
    fn into_scalar(self) -> units::Scalar;
}

impl AreaScalar for units::Area {
    fn into_scalar(self) -> units::Scalar {
        self / units::Area::new(1.0)
    }
}
