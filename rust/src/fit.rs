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

// ------------------------------------------------------------------ lines and arcs (method M5)

#[derive(Clone, Copy, Debug)]
pub enum Piece {
    Line { a: V2, b: V2 },
    /// Three-point arc: start, a point on it, end.
    Arc { a: V2, m: V2, b: V2 },
}

/// Circle through three points: (centre, radius), None if collinear.
pub fn circumcircle(a: V2, b: V2, c: V2) -> Option<(V2, f64)> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() < 1e-12 { return None; }
    let (a2, b2, c2) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
    let ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d;
    let uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d;
    Some(([ux, uy], dist([ux, uy], a)))
}

/// A run of points as one three-point arc, if it is one within `tol`.
fn try_arc(pts: &[V2], tol: f64) -> Option<Piece> {
    let n = pts.len();
    if n < 4 { return None; }
    let (a, b) = (pts[0], pts[n - 1]);
    let fit = fit_circle(pts)?;
    let c = [fit.cx, fit.cy];
    let ang = |p: V2| (p[1] - c[1]).atan2(p[0] - c[0]);
    // angular progress around the fitted centre: one direction, allowing back-steps of noise size
    let wrap = |d: f64| { let mut d = d; while d > std::f64::consts::PI { d -= std::f64::consts::TAU; } while d < -std::f64::consts::PI { d += std::f64::consts::TAU; } d };
    let steps: Vec<f64> = pts.windows(2).map(|w| wrap(ang(w[1]) - ang(w[0]))).collect();
    let sweep: f64 = steps.iter().sum();
    let noise = 3.0 * tol / fit.r;
    if steps.iter().any(|&d| d * sweep.signum() < -noise) { return None; }
    let (aa, ab) = (ang(a), ang(b));
    if sweep.abs() > 340f64.to_radians() || sweep.abs() < 1e-6 { return None; }
    let am = aa + sweep / 2.0;
    let m = [c[0] + fit.r * am.cos(), c[1] + fit.r * am.sin()];
    let (cc, r) = circumcircle(a, m, b)?; // the circle Fusion will draw through the three points
    if r < 3.0 * tol || r > 1e5 { return None; }
    let worst = pts.iter().map(|&p| (dist(p, cc) - r).abs()).fold(0.0, f64::max);
    if worst > tol { return None; }
    // The STL surface between two listed points is the straight edge joining them, so the arc must
    // also follow those edges: a long straight wall must not pass for a huge-radius arc.
    for w in pts.windows(2) {
        let l = dist(w[0], w[1]);
        if l <= 4.0 * tol { continue; }
        let k = ((l / (4.0 * tol)).ceil() as usize).min(16);
        for i in 1..k {
            let t = i as f64 / k as f64;
            let q = [w[0][0] + t * (w[1][0] - w[0][0]), w[0][1] + t * (w[1][1] - w[0][1])];
            if (dist(q, cc) - r).abs() > tol { return None; }
        }
    }
    Some(Piece::Arc { a, m, b })
}

fn fit_run(pts: &[V2], tol: f64, depth: usize, out: &mut Vec<(Piece, Vec<V2>)>) {
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    if pts.len() <= 2 { out.push((Piece::Line { a, b }, pts.to_vec())); return; }
    let (mut k, mut dmax) = (0usize, 0.0f64);
    for (i, &p) in pts.iter().enumerate() {
        let d = point_segment_dist(p, a, b);
        if d > dmax { dmax = d; k = i; }
    }
    if dmax <= tol { out.push((Piece::Line { a, b }, pts.to_vec())); return; }
    if let Some(arc) = try_arc(pts, tol) { out.push((arc, pts.to_vec())); return; }
    if depth > 24 || k == 0 || k == pts.len() - 1 { out.push((Piece::Line { a, b }, pts.to_vec())); return; }
    fit_run(&pts[..=k], tol, depth + 1, out);
    fit_run(&pts[k..], tol, depth + 1, out);
}

/// Largest end index in `lo..=hi` with `ok(end)`, given `ok(lo)`: gallop out, then bisect.
fn max_end(lo: usize, hi: usize, ok: impl Fn(usize) -> bool) -> usize {
    let (mut good, mut step) = (lo, 1usize);
    let mut bad = hi + 1;
    while good + step <= hi {
        if ok(good + step) { good += step; step *= 2; } else { bad = good + step; break; }
    }
    while bad - good > 1 {
        let mid = (good + bad) / 2;
        if ok(mid) { good = mid } else { bad = mid }
    }
    good
}

/// Greedy longest-piece-first segmentation of an open run: from each start, extend a line and an
/// arc as far as each stays within `tol`, keep the one that reaches further, continue from there.
fn greedy_run(pts: &[V2], tol: f64, out: &mut Vec<(Piece, Vec<V2>)>) {
    let last = pts.len() - 1;
    let mut i = 0;
    while i < last {
        let line_end = max_end(i + 1, last, |e| pts[i..=e].iter().all(|&p| point_segment_dist(p, pts[i], pts[e]) <= tol));
        let arc_end = if last >= i + 3 { max_end(i + 3, last, |e| try_arc(&pts[i..=e], tol).is_some()) } else { i };
        let use_arc = arc_end >= i + 3 && try_arc(&pts[i..=arc_end], tol).is_some() && arc_end > line_end + 1;
        if use_arc {
            out.push((try_arc(&pts[i..=arc_end], tol).unwrap(), pts[i..=arc_end].to_vec()));
            i = arc_end;
        } else {
            out.push((Piece::Line { a: pts[i], b: pts[line_end] }, pts[i..=line_end].to_vec()));
            i = line_end;
        }
    }
}

/// Dense polyline approximating a piece, for error measurement.
fn piece_polyline(p: &Piece) -> Vec<V2> {
    match *p {
        Piece::Line { a, b } => vec![a, b],
        Piece::Arc { a, m, b } => match circumcircle(a, m, b) {
            None => vec![a, b],
            Some((c, r)) => {
                let ang = |p: V2| (p[1] - c[1]).atan2(p[0] - c[0]);
                let (aa, am, ab) = (ang(a), ang(m), ang(b));
                let norm = |x: f64| x.rem_euclid(std::f64::consts::TAU);
                // direction a -> b through m
                let ccw = norm(am - aa) < norm(ab - aa);
                let sweep = if ccw { norm(ab - aa) } else { -norm(aa - ab) };
                let steps = ((sweep.abs() / 0.01).ceil() as usize).max(2);
                (0..=steps).map(|i| { let t = aa + sweep * i as f64 / steps as f64; [c[0] + r * t.cos(), c[1] + r * t.sin()] }).collect()
            }
        },
    }
}

/// Drop consecutive points closer than `min_d` (crossings at a shared vertex repeat it).
fn dedupe_ring(ring: &[V2], min_d: f64) -> Vec<V2> {
    let mut out: Vec<V2> = Vec::with_capacity(ring.len());
    for &p in ring {
        if out.last().map_or(true, |&q| dist(p, q) >= min_d) { out.push(p); }
    }
    while out.len() > 1 && dist(out[0], *out.last().unwrap()) < min_d { out.pop(); }
    out
}

/// Corner vertices: where the direction changes by more than 25 degrees, measured over a short
/// baseline (so micro-noise cannot fake or hide a corner), one vertex per corner.
pub fn find_corners(ring: &[V2], baseline: f64) -> Vec<usize> {
    let n = ring.len();
    let seg: Vec<f64> = (0..n).map(|i| dist(ring[i], ring[(i + 1) % n])).collect();
    let walk = |i: usize, fwd: bool| -> usize {
        let (mut j, mut acc) = (i, 0.0);
        for _ in 0..n / 2 {
            let nj = if fwd { (j + 1) % n } else { (j + n - 1) % n };
            acc += if fwd { seg[j] } else { seg[nj] };
            j = nj;
            if acc >= baseline { break; }
        }
        j
    };
    let angle: Vec<f64> = (0..n).map(|i| {
        let (b, f) = (walk(i, false), walk(i, true));
        let (e1, e2) = ([ring[i][0] - ring[b][0], ring[i][1] - ring[b][1]], [ring[f][0] - ring[i][0], ring[f][1] - ring[i][1]]);
        let (l1, l2) = ((e1[0] * e1[0] + e1[1] * e1[1]).sqrt(), (e2[0] * e2[0] + e2[1] * e2[1]).sqrt());
        if l1 < 1e-12 || l2 < 1e-12 { return 0.0; }
        ((e1[0] * e2[0] + e1[1] * e2[1]) / (l1 * l2)).clamp(-1.0, 1.0).acos()
    }).collect();
    let thr = 25f64.to_radians();
    let mut out = Vec::new();
    for i in 0..n {
        if angle[i] <= thr { continue; }
        // keep only the sharpest vertex within one baseline of travel
        let (mut best, mut d) = (true, 0.0);
        for dir in [true, false] {
            let (mut j, mut acc) = (i, 0.0);
            for _ in 0..n / 2 {
                let nj = if dir { (j + 1) % n } else { (j + n - 1) % n };
                d = if dir { seg[j] } else { seg[nj] };
                acc += d;
                if acc > baseline { break; }
                j = nj;
                if angle[j] > angle[i] || (angle[j] == angle[i] && j < i) { best = false; }
            }
        }
        if best { out.push(i); }
    }
    out
}

/// Split a closed ring at its corners and fit each smooth run as lines and three-point arcs.
/// Returns the pieces (closed, in order) and the worst distance from the original ring to them.
pub fn segment_ring(original: &[V2], tol: f64) -> (Vec<Piece>, f64) {
    let ring = dedupe_ring(original, 1e-4);
    let ring = &ring[..];
    let n = ring.len();
    let corners = find_corners(ring, (0.05f64).max(10.0 * tol));
    let mut parts: Vec<(Piece, Vec<V2>)> = Vec::new();
    if corners.is_empty() {
        // smooth ring: one cyclic run from the first point (the merge below joins across the seam)
        let mut run: Vec<V2> = ring.to_vec();
        run.push(ring[0]);
        greedy_run(&run, tol, &mut parts);
    } else {
        for w in 0..corners.len() {
            let (s, e) = (corners[w], corners[(w + 1) % corners.len()]);
            let len = if e > s { e - s } else { e + n - s };
            let run: Vec<V2> = (0..=len).map(|k| ring[(s + k) % n]).collect();
            greedy_run(&run, tol, &mut parts);
        }
    }
    // merge neighbours that are really one line or one arc
    let mut changed = true;
    while changed && parts.len() > 2 {
        changed = false;
        for i in 0..parts.len() {
            let j = (i + 1) % parts.len();
            if i == j { continue; }
            let same_kind = matches!((&parts[i].0, &parts[j].0), (Piece::Line { .. }, Piece::Line { .. }) | (Piece::Arc { .. }, Piece::Arc { .. }));
            if !same_kind { continue; }
            let mut joined = parts[i].1.clone();
            joined.extend_from_slice(&parts[j].1[1..]);
            let mut one = Vec::new();
            fit_run(&joined, tol, 24, &mut one); // depth 24: no further splitting
            if one.len() == 1 {
                let ok = match one[0].0 {
                    Piece::Line { a, b } => joined.iter().all(|&p| point_segment_dist(p, a, b) <= tol),
                    Piece::Arc { .. } => true,
                };
                if ok {
                    let merged = one.remove(0);
                    let (lo, hi) = (i.min(j), i.max(j));
                    if hi == lo + 1 { parts.splice(lo..=hi, [merged]); } else { parts.remove(hi); parts[lo] = merged; }
                    changed = true;
                    break;
                }
            }
        }
    }
    let pieces: Vec<Piece> = parts.into_iter().map(|p| p.0).collect();
    let dense: Vec<V2> = pieces.iter().flat_map(piece_polyline).collect();
    let mut err: f64 = 0.0;
    for &p in original {
        let d = (0..dense.len()).map(|i| point_segment_dist(p, dense[i], dense[(i + 1) % dense.len()])).fold(f64::INFINITY, f64::min);
        err = err.max(d);
    }
    (pieces, err)
}
