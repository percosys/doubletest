//! Fillets and chamfers on layer edges, read as offset-versus-depth profiles (method M6).

use crate::geom::*;
use crate::layers::Layer;
use crate::mesh::Mesh;
use crate::poly::*;
use crate::section::section_loops;
use crate::verify::Stack;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Family { ConvexTop, ConvexBottom, ConcaveAbove, ConcaveBelow }

impl Family {
    pub fn name(&self) -> &'static str {
        match self { Family::ConvexTop => "convex_top", Family::ConvexBottom => "convex_bottom",
                     Family::ConcaveAbove => "concave_above", Family::ConcaveBelow => "concave_below" }
    }
    pub fn convex(&self) -> bool { matches!(self, Family::ConvexTop | Family::ConvexBottom) }
    /// The treatment lives below the level (true) or above it.
    pub fn below(&self) -> bool { matches!(self, Family::ConvexTop | Family::ConcaveBelow) }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind { Chamfer, Fillet }

/// A cyclic run of vertices on one ring of one layer polygon.
#[derive(Clone, Debug)]
pub struct Run { pub poly: usize, pub ring: usize, pub start: usize, pub len: usize }

#[derive(Clone, Debug)]
pub struct Treatment {
    pub z: f64,
    pub family: Family,
    pub kind: Kind,
    pub size_xy: f64,
    pub size_z: f64,
    pub rms: f64,
    pub layer: usize,
    pub runs: Vec<Run>,
    /// The run polylines (for matching Fusion edges).
    pub curves: Vec<Vec<V2>>,
}

impl Treatment {
    /// Horizontal offset at depth d from the level.
    pub fn profile(&self, d: f64) -> f64 {
        match self.kind {
            Kind::Chamfer => self.size_xy * (1.0 - d / self.size_z).max(0.0),
            Kind::Fillet => fillet_offset(self.size_xy, d),
        }
    }
    pub fn band(&self) -> (f64, f64) {
        if self.family.below() { (self.z - self.size_z, self.z) } else { (self.z, self.z + self.size_z) }
    }
}

fn fillet_offset(r: f64, d: f64) -> f64 {
    if d >= r { 0.0 } else { r - (r * r - (r - d) * (r - d)).max(0.0).sqrt() }
}

// ------------------------------------------------------------------ geometry helpers
fn ring_boundary_dist(p: V2, polys: &[Poly]) -> f64 {
    let mut best = f64::INFINITY;
    for poly in polys {
        for ring in std::iter::once(&poly.outer).chain(poly.holes.iter()) {
            let n = ring.len();
            for i in 0..n { best = best.min(point_segment_dist(p, ring[i], ring[(i + 1) % n])); }
        }
    }
    best
}

fn inside(p: V2, polys: &[Poly]) -> bool { polys.iter().any(|q| q.contains(p)) }

#[derive(Clone, Copy, PartialEq)]
enum Class { Exposed, Inside, Coincident }

fn classify(p: V2, other: &[Poly], eps: f64) -> Class {
    if other.is_empty() { return Class::Exposed; }
    if ring_boundary_dist(p, other) <= eps { Class::Coincident }
    else if inside(p, other) { Class::Inside } else { Class::Exposed }
}

fn runs_of(classes: &[Class], want: Class) -> Vec<(usize, usize)> {
    let n = classes.len();
    if classes.iter().all(|&c| c == want) { return vec![(0, n)]; }
    let Some(first) = (0..n).find(|&i| classes[i] != want && classes[(i + 1) % n] == want) else { return vec![] };
    let mut out = Vec::new();
    let mut i = (first + 1) % n;
    let mut seen = 0;
    while seen < n {
        if classes[i] == want {
            let s = i;
            let mut l = 0;
            while classes[(s + l) % n] == want && seen < n { l += 1; seen += 1; }
            out.push((s, l));
            i = (s + l) % n;
        } else { i = (i + 1) % n; seen += 1; }
    }
    out
}

fn run_polyline(ring: &[V2], start: usize, len: usize) -> Vec<V2> {
    let n = ring.len();
    let mut pts: Vec<V2> = (0..len).map(|k| ring[(start + k) % n]).collect();
    if len == n { pts.push(ring[start]); }
    pts
}

/// Points along a polyline at roughly `h` spacing (at most `cap` of them).
fn resample(pts: &[V2], h: f64, cap: usize) -> Vec<V2> {
    let total: f64 = pts.windows(2).map(|w| dist(w[0], w[1])).sum();
    let n = ((total / h).ceil() as usize).clamp(2, cap);
    let mut out = Vec::with_capacity(n);
    let step = total / n as f64;
    let (mut seg, mut acc) = (0usize, 0.0f64);
    for k in 0..n {
        let target = step * (k as f64 + 0.5);
        while seg + 1 < pts.len() - 1 && acc + dist(pts[seg], pts[seg + 1]) < target {
            acc += dist(pts[seg], pts[seg + 1]);
            seg += 1;
        }
        let l = dist(pts[seg], pts[seg + 1]).max(1e-12);
        let t = ((target - acc) / l).clamp(0.0, 1.0);
        out.push([pts[seg][0] + t * (pts[seg + 1][0] - pts[seg][0]), pts[seg][1] + t * (pts[seg + 1][1] - pts[seg][1])]);
    }
    out
}

fn polyline_len(p: &[V2]) -> f64 { p.windows(2).map(|w| dist(w[0], w[1])).sum() }

// ------------------------------------------------------------------ fitting
fn rms_of(d: &[f64], o: &[f64], f: impl Fn(f64) -> f64) -> f64 {
    (d.iter().zip(o).map(|(&x, &y)| (f(x) - y).powi(2)).sum::<f64>() / d.len() as f64).sqrt()
}

fn golden(mut lo: f64, mut hi: f64, f: impl Fn(f64) -> f64) -> f64 {
    let g = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..40 {
        let (a, b) = (hi - g * (hi - lo), lo + g * (hi - lo));
        if f(a) < f(b) { hi = b } else { lo = a }
    }
    0.5 * (lo + hi)
}

/// Best chamfer or fillet for the profile. Returns (kind, size_xy, size_z, rms).
pub fn fit_profile(d: &[f64], o: &[f64]) -> (Kind, f64, f64, f64) {
    let lo = (d[0] * 0.5).ln();
    let hi = 100f64.ln();
    let grid: Vec<f64> = (0..240).map(|i| (lo + (hi - lo) * i as f64 / 239.0).exp()).collect();
    // chamfer: o = a * max(1 - d/b, 0); for fixed b the best a is linear least squares
    let cham = |b: f64| -> (f64, f64) {
        let m: Vec<f64> = d.iter().map(|&x| (1.0 - x / b).max(0.0)).collect();
        let (num, den) = (m.iter().zip(o).map(|(a, b)| a * b).sum::<f64>(), m.iter().map(|a| a * a).sum::<f64>());
        let a = if den > 0.0 { num / den } else { 0.0 };
        (a, rms_of(d, o, |x| a * (1.0 - x / b).max(0.0)))
    };
    let bi = (0..grid.len()).min_by(|&i, &j| cham(grid[i]).1.partial_cmp(&cham(grid[j]).1).unwrap()).unwrap();
    let (bl, bh) = (grid[bi.saturating_sub(1)].ln(), grid[(bi + 1).min(grid.len() - 1)].ln());
    let b = golden(bl, bh, |lb| cham(lb.exp()).1).exp();
    let (a, rc) = cham(b);
    let fil = |r: f64| rms_of(d, o, |x| fillet_offset(r, x));
    let ri = (0..grid.len()).min_by(|&i, &j| fil(grid[i]).partial_cmp(&fil(grid[j])).unwrap()).unwrap();
    let (rl, rh) = (grid[ri.saturating_sub(1)].ln(), grid[(ri + 1).min(grid.len() - 1)].ln());
    let r = golden(rl, rh, |lr| fil(lr.exp())).exp();
    let rf = fil(r);
    if rf < rc { (Kind::Fillet, r, r, rf) } else { (Kind::Chamfer, a, b, rc) }
}

// ------------------------------------------------------------------ detection
fn median(v: &mut Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

/// Detect chamfers and fillets along the exposed outlines of every layer interface.
pub fn detect(am: &Mesh, layers: &[Layer], tol: f64, max_edge: f64) -> Vec<Treatment> {
    let eps = 0.01;
    let mut cache: HashMap<i64, Vec<Poly>> = HashMap::new();
    let mut section = |z: f64| -> Vec<Poly> {
        cache.entry((z * 1e5).round() as i64).or_insert_with(|| regions_from_loops(section_loops(am, z))).clone()
    };
    let mut out: Vec<Treatment> = Vec::new();
    let none: Vec<Poly> = vec![];
    for k in -1..layers.len() as isize {
        let a = if k >= 0 { Some(&layers[k as usize]) } else { None };
        let b = layers.get((k + 1) as usize);
        let z = match a { Some(l) => l.z1, None => layers[0].z0 };
        // (layer index, polygons of that layer, the neighbour across the level, wanted classes)
        let mut jobs: Vec<(usize, &Layer, &[Poly], Family, Family)> = Vec::new();
        if let Some(l) = a { jobs.push((k as usize, l, b.map_or(&none[..], |x| &x.polys[..]), Family::ConvexTop, Family::ConcaveBelow)); }
        if let Some(l) = b { jobs.push(((k + 1) as usize, l, a.map_or(&none[..], |x| &x.polys[..]), Family::ConvexBottom, Family::ConcaveAbove)); }
        for (li, layer, other, exposed_fam, inside_fam) in jobs {
            let th = layer.z1 - layer.z0;
            let (dmin, dmax) = ((0.02f64).max(0.2 * tol), max_edge.min(0.45 * th));
            if dmax < 2.0 * dmin { continue; }
            let deltas: Vec<f64> = (0..9).map(|i| (dmin.ln() + (dmax.ln() - dmin.ln()) * i as f64 / 8.0).exp()).collect();
            for (pi, poly) in layer.polys.iter().enumerate() {
                for (ri, ring) in std::iter::once(&poly.outer).chain(poly.holes.iter()).enumerate() {
                    let classes: Vec<Class> = ring.iter().map(|&p| classify(p, other, eps)).collect();
                    for (want, fam) in [(Class::Exposed, exposed_fam), (Class::Inside, inside_fam)] {
                        for (start, len) in runs_of(&classes, want) {
                            let line = run_polyline(ring, start, len);
                            if polyline_len(&line) < 1.0 { continue; }
                            let pts = resample(&line, 0.5, 240);
                            let mut o: Vec<f64> = Vec::new();
                            for &d in &deltas {
                                let zs = if fam.below() { z - d } else { z + d };
                                let sec = section(zs);
                                let mut v: Vec<f64> = pts.iter().map(|&p| {
                                    let inn = inside(p, &sec);
                                    if fam.convex() == inn { 0.0 } else { ring_boundary_dist(p, &sec).min(max_edge) }
                                }).collect();
                                o.push(median(&mut v));
                            }
                            let omax = o.iter().cloned().fold(0.0, f64::max);
                            if omax < 0.5 * tol { continue; }
                            let (kind, sxy, sz, rms) = fit_profile(&deltas, &o);
                            if rms > (0.3 * tol).max(0.1 * omax) { continue; }
                            out.push(Treatment { z, family: fam, kind, size_xy: sxy, size_z: sz, rms, layer: li,
                                runs: vec![Run { poly: pi, ring: ri, start, len }], curves: vec![line] });
                        }
                    }
                }
            }
        }
    }
    group(out, tol)
}

/// Loops that came out the same (kind, level, size) become one treatment.
fn group(ts: Vec<Treatment>, tol: f64) -> Vec<Treatment> {
    let q = tol.max(0.05);
    let mut out: Vec<Treatment> = Vec::new();
    for t in ts {
        let key = |t: &Treatment| (((t.z * 1e3).round() as i64), t.family, t.kind == Kind::Fillet, t.layer,
                                   (t.size_xy / q).round() as i64, (t.size_z / q).round() as i64);
        if let Some(g) = out.iter_mut().find(|g| key(g) == key(&t)) {
            let (wa, wb) = (g.curves.iter().map(|c| polyline_len(c)).sum::<f64>(), polyline_len(&t.curves[0]));
            g.size_xy = (g.size_xy * wa + t.size_xy * wb) / (wa + wb);
            g.size_z = (g.size_z * wa + t.size_z * wb) / (wa + wb);
            g.rms = g.rms.max(t.rms);
            g.runs.extend(t.runs);
            g.curves.extend(t.curves);
        } else { out.push(t); }
    }
    out
}

// ------------------------------------------------------------------ verification geometry
/// Move the run's vertices along the mitred outward normal by `shift` (negative = into the material).
fn displace(ring: &mut Ring, run: &Run, shift: f64) {
    let src = ring.clone();
    let n = src.len();
    let right = |a: V2, b: V2| -> V2 {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = (dx * dx + dy * dy).sqrt().max(1e-12);
        [dy / l, -dx / l]
    };
    for k in 0..run.len.min(n) {
        let i = (run.start + k) % n;
        let (p, a, b) = (src[i], src[(i + n - 1) % n], src[(i + 1) % n]);
        let (n1, n2) = (right(a, p), right(p, b));
        let den = (1.0 + n1[0] * n2[0] + n1[1] * n2[1]).max(0.25);
        let m = [(n1[0] + n2[0]) / den, (n1[1] + n2[1]) / den];
        ring[i] = [p[0] + shift * m[0], p[1] + shift * m[1]];
    }
}

/// Heights (as depth d from the level, 0..size_z) at which a treatment's profile is sampled:
/// two for a chamfer (it is straight), evenly spaced in angle for a fillet.
fn depth_cuts(t: &Treatment) -> Vec<f64> {
    match t.kind {
        Kind::Chamfer => vec![0.0, t.size_z],
        Kind::Fillet => (0..=16).map(|k| t.size_z * (1.0 - (std::f64::consts::FRAC_PI_2 * k as f64 / 16.0).cos())).collect(),
    }
}

/// The layer as a stack of sections with its edge treatments applied (outline vertices of the
/// treated runs moved along their mitred normals). All sections share one ring structure.
pub fn layer_sections(layer: &Layer, k: usize, ts: &[Treatment]) -> Stack {
    let mine: Vec<&Treatment> = ts.iter().filter(|t| t.layer == k).collect();
    let mut cuts = vec![layer.z0, layer.z1];
    for t in &mine {
        let (lo, hi) = t.band();
        for d in depth_cuts(t) {
            let z = if t.family.below() { t.z - d } else { t.z + d };
            if z >= layer.z0 - 1e-9 && z <= layer.z1 + 1e-9 && lo <= hi { cuts.push(z.clamp(layer.z0, layer.z1)); }
        }
    }
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut zs: Vec<f64> = Vec::new();
    for c in cuts {
        match zs.last() {
            Some(&l) if c - l < 5e-4 => { if (c - layer.z1).abs() < 1e-9 { *zs.last_mut().unwrap() = c; } }
            _ => zs.push(c),
        }
    }
    let sections = zs.iter().map(|&z| {
        let mut polys = layer.polys.clone();
        for t in &mine {
            let (lo, hi) = t.band();
            if z < lo - 1e-9 || z > hi + 1e-9 { continue; }
            let o = t.profile((z - t.z).abs());
            let shift = if t.family.convex() { -o } else { o };
            for run in &t.runs {
                let poly = &mut polys[run.poly];
                if run.ring == 0 { displace(&mut poly.outer, run, shift) } else { displace(&mut poly.holes[run.ring - 1], run, shift) }
            }
        }
        polys
    }).collect();
    Stack { z: zs, sections }
}
