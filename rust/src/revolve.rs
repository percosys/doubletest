//! Turned parts as one revolve (method M7).

use crate::fit::fit_circle;
use crate::geom::*;
use crate::layers::ring_to_loop;
use crate::mesh::Mesh;
use crate::plan::Region;
use crate::poly::*;
use crate::section::section_loops;

/// Axis position (x, y) in the aligned frame if every section square to Z is made of concentric
/// circles; None for any part that is not a surface of revolution about Z.
pub fn turned_axis(am: &Mesh, tol: f64) -> Option<(f64, f64)> {
    let zmax = am.bounds().1[2];
    let samples = 41;
    let mut centres: Vec<(V2, f64)> = Vec::new();
    for i in 0..samples {
        let z = zmax * (i as f64 + 0.5) / samples as f64;
        let loops = section_loops(am, z);
        if loops.is_empty() { continue; }
        for lp in &loops {
            let (lo, hi) = lp.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| {
                ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])])
            });
            if (hi[0] - lo[0]).max(hi[1] - lo[1]) < 4.0 * tol { continue; } // a cone or sphere tip
            if lp.len() < 12 { return None; }
            let c = fit_circle(lp)?;
            if c.rms > 0.005 * c.r + 0.2 * tol || c.gap > 60f64.to_radians() { return None; }
            centres.push(([c.cx, c.cy], c.r));
        }
    }
    if centres.len() < 3 { return None; }
    let w: f64 = centres.iter().map(|c| c.1).sum();
    let mean = [centres.iter().map(|c| c.0[0] * c.1).sum::<f64>() / w, centres.iter().map(|c| c.0[1] * c.1).sum::<f64>() / w];
    for (c, r) in &centres {
        if dist(*c, mean) > 0.25 * tol + 0.002 * r { return None; }
    }
    Some((mean[0], mean[1]))
}

/// The half section through the axis, in (axial, radial) coordinates with radial >= 0.
pub fn half_profile(am: &Mesh, axis_xy: (f64, f64)) -> Vec<Poly> {
    // permute (x, y, z) -> (z, x - cx, y - cy): the plane y = cy becomes z' = 0, orientation kept
    let v: Vec<V3> = am.v.iter().map(|p| [p[2], p[0] - axis_xy.0, p[1] - axis_xy.1]).collect();
    let perm = Mesh { v, f: am.f.clone() };
    let polys = regions_from_loops(section_loops(&perm, 0.0));
    let (lo, hi) = perm.bounds();
    let half = vec![Poly { outer: vec![[lo[0] - 1.0, 0.0], [hi[0] + 1.0, 0.0], [hi[0] + 1.0, hi[1].max(1.0) + 1.0], [lo[0] - 1.0, hi[1].max(1.0) + 1.0]], holes: vec![] }];
    intersection(&polys, &half)
}

pub struct Revolve {
    /// Profile regions in sketch coordinates (radial = x, axial = y), for the CAD side.
    pub regions: Vec<Region>,
    /// The same profile in (axial, radial) coordinates, for verification.
    pub polys: Vec<Poly>,
    pub axis_xy: (f64, f64),
    pub simp: f64,
}

/// Profile regions as lines/arcs/circles (same fitting as outlines).
pub fn build_profile(am: &Mesh, axis_xy: (f64, f64), eps: f64) -> Option<Revolve> {
    let polys = half_profile(am, axis_xy);
    if polys.is_empty() { return None; }
    let r5 = |x: f64| (x * 1e5).round() / 1e5;
    let mut regions = Vec::new();
    let mut simp: f64 = 0.0;
    let swap = |r: &Ring| -> Ring { r.iter().map(|v| [v[1], v[0]]).collect() };
    let sketch: Vec<Poly> = polys.iter().map(|p| Poly { outer: swap(&p.outer), holes: p.holes.iter().map(|h| swap(h)).collect() }).collect();
    for p in &sketch {
        let (outer, _, s0) = ring_to_loop(&p.outer, eps);
        simp = simp.max(s0);
        let mut holes = Vec::new();
        for h in &p.holes {
            let (l, _, s) = ring_to_loop(h, eps);
            simp = simp.max(s);
            holes.push(l);
        }
        let c = p.centroid();
        regions.push(Region { outer, holes, area: (p.area() * 1e4).round() / 1e4, centroid: [r5(c[0]), r5(c[1])] });
    }
    Some(Revolve { regions, polys, axis_xy, simp })
}

/// The solid of revolution of the profile polygons (axial, radial) as a triangle soup in the aligned
/// frame (axis through `axis_xy`, parallel to Z), plus its volume (Pappus: 2 pi * area * centroid radius).
pub fn revolve_surface(polys: &[Poly], axis_xy: (f64, f64), tol: f64) -> (Mesh, f64) {
    let rmax = polys.iter().flat_map(|p| p.outer.iter()).fold(1.0f64, |m, v| m.max(v[1]));
    // chord sag of the revolve itself stays within 10 % of the tolerance
    let n = ((std::f64::consts::PI / (1.0 - 0.1 * tol / rmax).clamp(-1.0, 1.0).acos().max(1e-3)).ceil() as usize).clamp(90, 1440);
    let mut m = Mesh::default();
    let mut vol = 0.0;
    for p in polys {
        vol += 2.0 * std::f64::consts::PI * p.area() * p.centroid()[1];
        for ring in std::iter::once(&p.outer).chain(p.holes.iter()) {
            let k = ring.len();
            for i in 0..k {
                let (a, b) = (ring[i], ring[(i + 1) % k]);
                if a[1].abs() < 1e-9 && b[1].abs() < 1e-9 { continue; } // on the axis
                let base = m.v.len() as u32;
                let pt = |q: V2, j: usize| { let t = std::f64::consts::TAU * j as f64 / n as f64; [axis_xy.0 + q[1] * t.cos(), axis_xy.1 + q[1] * t.sin(), q[0]] };
                for j in 0..=n { m.v.push(pt(a, j)); m.v.push(pt(b, j)); }
                for j in 0..n {
                    let i0 = base + 2 * j as u32;
                    m.f.push([i0, i0 + 1, i0 + 3]);
                    m.f.push([i0, i0 + 3, i0 + 2]);
                }
            }
        }
    }
    (m, vol)
}
