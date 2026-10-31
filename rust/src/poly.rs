//! 2D polygons with holes, even-odd nesting of loops, and boolean operations.

use crate::geom::*;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

pub type Ring = Vec<V2>;

#[derive(Clone, Debug)]
pub struct Poly {
    pub outer: Ring,
    pub holes: Vec<Ring>,
}

pub fn ring_area_signed(r: &[V2]) -> f64 {
    let n = r.len();
    let mut s = 0.0;
    for i in 0..n {
        let (a, b) = (r[i], r[(i + 1) % n]);
        s += a[0] * b[1] - b[0] * a[1];
    }
    s / 2.0
}

/// Centroid of a ring (area-weighted); falls back to the vertex mean for a degenerate ring.
pub fn ring_centroid(r: &[V2]) -> (V2, f64) {
    let a = ring_area_signed(r);
    let n = r.len();
    if a.abs() < 1e-12 {
        let m = r.iter().fold([0.0, 0.0], |s, p| [s[0] + p[0], s[1] + p[1]]);
        return ([m[0] / n as f64, m[1] / n as f64], 0.0);
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let (p, q) = (r[i], r[(i + 1) % n]);
        let w = p[0] * q[1] - q[0] * p[1];
        cx += (p[0] + q[0]) * w;
        cy += (p[1] + q[1]) * w;
    }
    ([cx / (6.0 * a), cy / (6.0 * a)], a)
}

pub fn point_in_ring(p: V2, r: &[V2]) -> bool {
    let mut inside = false;
    let n = r.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (r[i], r[j]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

impl Poly {
    pub fn area(&self) -> f64 {
        ring_area_signed(&self.outer).abs() - self.holes.iter().map(|h| ring_area_signed(h).abs()).sum::<f64>()
    }
    pub fn centroid(&self) -> V2 {
        let (c, a) = ring_centroid(&self.outer);
        let (mut cx, mut cy, mut at) = (c[0] * a.abs(), c[1] * a.abs(), a.abs());
        for h in &self.holes {
            let (hc, ha) = ring_centroid(h);
            cx -= hc[0] * ha.abs();
            cy -= hc[1] * ha.abs();
            at -= ha.abs();
        }
        if at.abs() < 1e-12 { c } else { [cx / at, cy / at] }
    }
    pub fn contains(&self, p: V2) -> bool {
        point_in_ring(p, &self.outer) && !self.holes.iter().any(|h| point_in_ring(p, h))
    }
}

pub fn total_area(ps: &[Poly]) -> f64 { ps.iter().map(|p| p.area()).sum() }

/// Counter-clockwise (outer) or clockwise (hole), starting at the lowest-then-leftmost vertex,
/// so the same part always yields the same plan.
pub fn normalise_ring(mut r: Ring, ccw: bool) -> Ring {
    if (ring_area_signed(&r) > 0.0) != ccw { r.reverse(); }
    let k = (0..r.len()).min_by(|&i, &j| r[i][0].partial_cmp(&r[j][0]).unwrap().then(r[i][1].partial_cmp(&r[j][1]).unwrap())).unwrap_or(0);
    r.rotate_left(k);
    r
}

/// Material regions from section loops by the even-odd rule: a loop nested inside an odd number
/// of others is a hole of the smallest enclosing material loop.
pub fn regions_from_loops(loops: Vec<Ring>) -> Vec<Poly> {
    let n = loops.len();
    let areas: Vec<f64> = loops.iter().map(|l| ring_area_signed(l).abs()).collect();
    // containers[i] = loops (larger than i) that contain loop i
    let containers: Vec<Vec<usize>> = (0..n)
        .map(|i| (0..n).filter(|&j| j != i && areas[j] > areas[i] && point_in_ring(loops[i][0], &loops[j])).collect())
        .collect();
    let mut polys: Vec<(usize, Poly)> = Vec::new();
    for i in 0..n {
        if containers[i].len() % 2 == 0 {
            polys.push((i, Poly { outer: loops[i].clone(), holes: vec![] }));
        }
    }
    for i in 0..n {
        if containers[i].len() % 2 == 1 {
            // parent: the smallest container (it has even depth)
            if let Some(&p) = containers[i].iter().min_by(|&&a, &&b| areas[a].partial_cmp(&areas[b]).unwrap()) {
                if let Some(entry) = polys.iter_mut().find(|(k, _)| *k == p) {
                    entry.1.holes.push(loops[i].clone());
                }
            }
        }
    }
    let mut out: Vec<Poly> = polys.into_iter().map(|(_, p)| p).filter(|p| p.area() > 1e-6).collect();
    for p in out.iter_mut() {
        p.outer = normalise_ring(std::mem::take(&mut p.outer), true);
        p.holes = std::mem::take(&mut p.holes).into_iter().map(|h| normalise_ring(h, false)).collect();
        p.holes.sort_by(|a, b| a[0][0].partial_cmp(&b[0][0]).unwrap().then(a[0][1].partial_cmp(&b[0][1]).unwrap()));
    }
    out.sort_by(|a, b| a.outer[0][0].partial_cmp(&b.outer[0][0]).unwrap().then(a.outer[0][1].partial_cmp(&b.outer[0][1]).unwrap()));
    out
}

type Shapes = Vec<Vec<Vec<[f64; 2]>>>;

fn to_shapes(ps: &[Poly]) -> Shapes {
    ps.iter().map(|p| { let mut s = vec![p.outer.clone()]; s.extend(p.holes.iter().cloned()); s }).collect()
}

fn from_shapes(s: Shapes) -> Vec<Poly> {
    s.into_iter()
        .filter(|sh| !sh.is_empty())
        .map(|mut sh| { let outer = sh.remove(0); Poly { outer, holes: sh } })
        .collect()
}

fn run(a: &[Poly], b: &[Poly], rule: OverlayRule) -> Vec<Poly> {
    let sa = to_shapes(a);
    let sb = to_shapes(b);
    from_shapes(sa.overlay(&sb, rule, FillRule::EvenOdd))
}

pub fn difference(a: &[Poly], b: &[Poly]) -> Vec<Poly> {
    if a.is_empty() { return vec![]; }
    if b.is_empty() { return a.to_vec(); }
    run(a, b, OverlayRule::Difference)
}

pub fn xor(a: &[Poly], b: &[Poly]) -> Vec<Poly> {
    if a.is_empty() { return b.to_vec(); }
    if b.is_empty() { return a.to_vec(); }
    run(a, b, OverlayRule::Xor)
}

/// Triangulate a polygon with holes (ear clipping): vertices and triangle indices.
pub fn triangulate(p: &Poly) -> (Vec<V2>, Vec<[usize; 3]>) {
    let mut flat: Vec<f64> = Vec::new();
    let mut verts: Vec<V2> = Vec::new();
    let mut holes: Vec<usize> = Vec::new();
    for v in &p.outer { flat.extend_from_slice(v); verts.push(*v); }
    for h in &p.holes {
        holes.push(verts.len());
        for v in h { flat.extend_from_slice(v); verts.push(*v); }
    }
    let idx = earcutr::earcut(&flat, &holes, 2).unwrap_or_default();
    (verts, idx.chunks_exact(3).map(|t| [t[0], t[1], t[2]]).collect())
}
