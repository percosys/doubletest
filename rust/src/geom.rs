//! Small vector helpers (no external linear-algebra dependency).

pub type V2 = [f64; 2];
pub type V3 = [f64; 3];

#[inline] pub fn sub(a: V3, b: V3) -> V3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
#[inline] pub fn add(a: V3, b: V3) -> V3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
#[inline] pub fn scale(a: V3, s: f64) -> V3 { [a[0] * s, a[1] * s, a[2] * s] }
#[inline] pub fn dot(a: V3, b: V3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
#[inline] pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline] pub fn len(a: V3) -> f64 { dot(a, a).sqrt() }
#[inline] pub fn norm(a: V3) -> V3 { let l = len(a); if l > 0.0 { scale(a, 1.0 / l) } else { a } }

#[inline] pub fn dist2(a: V2, b: V2) -> f64 { (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) }
#[inline] pub fn dist(a: V2, b: V2) -> f64 { dist2(a, b).sqrt() }

/// Distance from point p to the segment a-b.
pub fn point_segment_dist(p: V2, a: V2, b: V2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    dist(p, [a[0] + t * dx, a[1] + t * dy])
}

pub const IDENTITY4: [[f64; 4]; 4] =
    [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]];
