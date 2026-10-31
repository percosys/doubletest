//! Least-squares fits and polyline thinning (methods M5, M10).

use crate::geom::*;

#[derive(Clone, Copy, Debug)]
pub struct CircleFit {
    pub cx: f64,
    pub cy: f64,
    pub r: f64,
    /// RMS radial residual.
    pub rms: f64,
    /// Largest angular gap between points (radians): near 2π means a short arc.
    pub gap: f64,
}

/// Algebraic (Kåsa) circle fit: solve x²+y² = 2ax+2by+c in least squares, on centred points.
pub fn fit_circle(pts: &[V2]) -> Option<CircleFit> {
    let n = pts.len();
    if n < 3 { return None; }
    let (mx, my) = (pts.iter().map(|p| p[0]).sum::<f64>() / n as f64, pts.iter().map(|p| p[1]).sum::<f64>() / n as f64);
    let mut m = [[0.0f64; 3]; 3];
    let mut rhs = [0.0f64; 3];
    for p in pts {
        let (x, y) = (p[0] - mx, p[1] - my);
        let row = [2.0 * x, 2.0 * y, 1.0];
        let b = x * x + y * y;
        for i in 0..3 {
            for j in 0..3 { m[i][j] += row[i] * row[j]; }
            rhs[i] += row[i] * b;
        }
    }
    let sol = solve3(m, rhs)?;
    let (a, b, c) = (sol[0], sol[1], sol[2]);
    let r2 = c + a * a + b * b;
    if !(r2 > 0.0) { return None; }
    let r = r2.sqrt();
    let rms = (pts.iter().map(|p| (((p[0] - mx - a).powi(2) + (p[1] - my - b).powi(2)).sqrt() - r).powi(2)).sum::<f64>() / n as f64).sqrt();
    let mut ang: Vec<f64> = pts.iter().map(|p| (p[1] - my - b).atan2(p[0] - mx - a)).collect();
    ang.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut gap = ang[0] + std::f64::consts::TAU - ang[n - 1];
    for w in ang.windows(2) { gap = gap.max(w[1] - w[0]); }
    Some(CircleFit { cx: a + mx, cy: b + my, r, rms, gap })
}

fn solve3(mut m: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let piv = (col..3).max_by(|&i, &j| m[i][col].abs().partial_cmp(&m[j][col].abs()).unwrap())?;
        if m[piv][col].abs() < 1e-14 { return None; }
        m.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..3 {
            let f = m[r][col] / m[col][col];
            for c in col..3 { m[r][c] -= f * m[col][c]; }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; 3];
    for i in (0..3).rev() {
        let mut s = b[i];
        for j in i + 1..3 { s -= m[i][j] * x[j]; }
        x[i] = s / m[i][i];
    }
    Some(x)
}

/// Ramer–Douglas–Peucker on an open polyline; returns the indices kept (ascending).
pub fn rdp_open(pts: &[V2], eps: f64) -> Vec<usize> {
    let n = pts.len();
    if n <= 2 { return (0..n).collect(); }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];
    while let Some((a, b)) = stack.pop() {
        if b - a < 2 { continue; }
        let (mut worst, mut wd) = (0usize, eps);
        for i in a + 1..b {
            let d = point_segment_dist(pts[i], pts[a], pts[b]);
            if d > wd { wd = d; worst = i; }
        }
        if worst != 0 {
            keep[worst] = true;
            stack.push((a, worst));
            stack.push((worst, b));
        }
    }
    (0..n).filter(|&i| keep[i]).collect()
}

/// Thin a closed ring: Douglas-Peucker from two anchors (the first point and the one farthest
/// from it), then drop vertices that are left collinear (an anchor in the middle of a straight
/// run). Returns the thinned ring and its true worst distance from the original ring.
pub fn simplify_ring(ring: &[V2], eps: f64) -> (Vec<V2>, f64) {
    let n = ring.len();
    if n <= 4 { return (ring.to_vec(), 0.0); }
    let far = (1..n).max_by(|&i, &j| dist2(ring[0], ring[i]).partial_cmp(&dist2(ring[0], ring[j])).unwrap()).unwrap();
    let first: Vec<V2> = ring[..=far].to_vec();
    let mut second: Vec<V2> = ring[far..].to_vec();
    second.push(ring[0]);
    let mut out: Vec<V2> = Vec::new();
    for part in [first, second] {
        let keep = rdp_open(&part, eps);
        for &k in &keep[..keep.len() - 1] { out.push(part[k]); }
    }
    // collinear cleanup, closed ring
    let tight = 0.05 * eps;
    let mut i = 0;
    while out.len() > 3 && i < out.len() {
        let m = out.len();
        let (a, p, b) = (out[(i + m - 1) % m], out[i], out[(i + 1) % m]);
        if point_segment_dist(p, a, b) <= tight { out.remove(i); i = i.saturating_sub(1); } else { i += 1; }
    }
    if out.len() < 3 { return (ring.to_vec(), 0.0); }
    let err = max_dist_to_ring(ring, &out);
    (out, err)
}

/// Largest distance from a point of `a` to the closed outline `b`.
pub fn max_dist_to_ring(a: &[V2], b: &[V2]) -> f64 {
    a.iter().map(|&p| (0..b.len()).map(|i| point_segment_dist(p, b[i], b[(i + 1) % b.len()])).fold(f64::INFINITY, f64::min))
        .fold(0.0, f64::max)
}
