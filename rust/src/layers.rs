//! Layers (slabs) along an axis (methods M3, M4, M5-circles).

use crate::fit::{fit_circle, simplify_ring};
use crate::geom::*;
use crate::mesh::Mesh;
use crate::plan::{Loop, Region};
use crate::poly::*;
use crate::section::section_loops;

pub const MAX_LAYERS: usize = 40;

#[derive(Clone, Debug)]
pub struct Layer {
    pub z0: f64,
    pub z1: f64,
    pub regions: Vec<Region>,
    /// Raw section polygons, used by verification.
    pub polys: Vec<Poly>,
}

fn r5(x: f64) -> f64 { (x * 1e5).round() / 1e5 }

/// A closed ring as a circle if it is one, else a thinned polyline.
/// Returns (loop, circle_fit_error, simplification_error).
pub fn ring_to_loop(ring: &[V2], eps: f64) -> (Loop, f64, f64) {
    if ring.len() >= 16 {
        if let Some(c) = fit_circle(ring) {
            if c.rms <= 0.002 * c.r + 1e-4 && c.gap <= 60f64.to_radians() {
                return (Loop::Circle { cx: r5(c.cx), cy: r5(c.cy), r: r5(c.r) }, c.rms, 0.0);
            }
        }
    }
    let (s, err) = simplify_ring(ring, eps);
    (Loop::Polyline { pts: s.iter().map(|p| [r5(p[0]), r5(p[1])]).collect() }, 0.0, err)
}

pub fn section_polys(am: &Mesh, z: f64) -> Vec<Poly> {
    regions_from_loops(section_loops(am, z))
}

fn levels(am: &Mesh, merge: f64) -> Vec<f64> {
    let zmax = am.bounds().1[2];
    let mut lv = vec![0.0, zmax];
    for i in 0..am.f.len() {
        let (n, _) = am.normal_area(i);
        if n[2].abs() > 0.9995 {
            let t = am.tri(i);
            lv.push((t[0][2] + t[1][2] + t[2][2]) / 3.0);
        }
    }
    lv.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut out = vec![lv[0]];
    for &z in &lv[1..] {
        if z - *out.last().unwrap() > merge { out.push(z); }
    }
    if zmax - *out.last().unwrap() > merge { out.push(zmax); } else { *out.last_mut().unwrap() = zmax; }
    out
}

fn same(a: &Layer, b: &Layer) -> bool {
    let (ua, ub) = (total_area(&a.polys), total_area(&b.polys));
    if (ua - ub).abs() > 1e-6 * ua.max(1.0) { return false; }
    total_area(&xor(&a.polys, &b.polys)) <= 1e-6 * ua.max(1.0)
}

/// Build layers for an aligned mesh (Z is the build axis, minimum at 0).
/// Returns (layers, max circle-fit error, max simplification error).
pub fn build_layers(am: &Mesh, eps: f64) -> Result<(Vec<Layer>, f64, f64), String> {
    let diag = am.diag();
    let lv = levels(am, (1e-3f64).max(1e-4 * diag));
    if lv.len() - 1 > MAX_LAYERS { return Err(format!("{} height levels (limit {})", lv.len() - 1, MAX_LAYERS)); }
    let mut layers: Vec<Layer> = Vec::new();
    let (mut worst, mut worst_simp) = (0.0f64, 0.0f64);
    for w in lv.windows(2) {
        let (z0, z1) = (w[0], w[1]);
        let polys = section_polys(am, 0.5 * (z0 + z1));
        let mut regions = Vec::new();
        for p in &polys {
            let (outer, e0, s0) = ring_to_loop(&p.outer, eps);
            worst = worst.max(e0);
            worst_simp = worst_simp.max(s0);
            let mut holes = Vec::new();
            for h in &p.holes {
                let (l, e, s) = ring_to_loop(h, eps);
                worst = worst.max(e);
                worst_simp = worst_simp.max(s);
                holes.push(l);
            }
            let c = p.centroid();
            regions.push(Region { outer, holes, area: (p.area() * 1e4).round() / 1e4, centroid: [r5(c[0]), r5(c[1])] });
        }
        let layer = Layer { z0, z1, regions, polys };
        if let Some(last) = layers.last_mut() {
            if same(last, &layer) { last.z1 = z1; continue; }
        }
        layers.push(layer);
    }
    layers.retain(|l| !l.regions.is_empty());
    Ok((layers, worst, worst_simp))
}

/// Merge layers thinner than `min_t` into the most similar neighbour; identical neighbours fuse.
pub fn merge_slivers(layers: &[Layer], min_t: f64) -> Vec<Layer> {
    let mut ls: Vec<Layer> = layers.to_vec();
    while ls.len() > 1 {
        let thin = (0..ls.len()).filter(|&i| ls[i].z1 - ls[i].z0 < min_t).min_by(|&a, &b| {
            (ls[a].z1 - ls[a].z0).partial_cmp(&(ls[b].z1 - ls[b].z0)).unwrap()
        });
        let Some(i) = thin else { break };
        let nbrs: Vec<usize> = [i.wrapping_sub(1), i + 1].into_iter().filter(|&j| j < ls.len()).collect();
        let j = *nbrs.iter().min_by(|&&a, &&b| {
            let ca = total_area(&xor(&ls[i].polys, &ls[a].polys));
            let cb = total_area(&xor(&ls[i].polys, &ls[b].polys));
            ca.partial_cmp(&cb).unwrap()
        }).unwrap();
        let (z0, z1) = (ls[i].z0.min(ls[j].z0), ls[i].z1.max(ls[j].z1));
        ls[j].z0 = z0;
        ls[j].z1 = z1;
        ls.remove(i);
    }
    let mut out: Vec<Layer> = Vec::new();
    for l in ls {
        match out.last_mut() {
            Some(last) if same(last, &l) => last.z1 = l.z1,
            _ => out.push(l),
        }
    }
    out
}
