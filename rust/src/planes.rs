//! Dominant directions and the part frame (method M2).

use crate::geom::*;
use crate::mesh::Mesh;
use std::collections::HashMap;

fn find(p: &mut Vec<u32>, mut x: u32) -> u32 {
    while p[x as usize] != x {
        p[x as usize] = p[p[x as usize] as usize];
        x = p[x as usize];
    }
    x
}

/// Plane normals (sign folded so the largest component is positive) ranked by total flat area.
/// A flat region grows over neighbours whose normals differ by under ~1° and is kept if all its
/// vertices lie within 0.1 % of the diagonal of its mean plane.
pub fn dominant_axes(m: &Mesh, top: usize) -> Vec<V3> {
    let nf = m.f.len();
    let na: Vec<(V3, f64)> = (0..nf).map(|i| m.normal_area(i)).collect();
    let mut parent: Vec<u32> = (0..nf as u32).collect();
    let cos1 = (1.0f64).to_radians().cos();
    for (a, b) in m.adjacency() {
        if dot(na[a as usize].0, na[b as usize].0) >= cos1 {
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra != rb { parent[ra as usize] = rb; }
        }
    }
    let mut comps: HashMap<u32, Vec<u32>> = HashMap::new();
    for i in 0..nf as u32 { let r = find(&mut parent, i); comps.entry(r).or_default().push(i); }
    let tol = 0.001 * m.diag();
    let mut groups: Vec<(V3, f64)> = Vec::new();
    for faces in comps.values() {
        if faces.len() < 2 { continue; }
        let mut n = [0.0; 3];
        let mut area = 0.0;
        for &f in faces { let (fn_, a) = na[f as usize]; n = add(n, scale(fn_, a)); area += a; }
        if len(n) == 0.0 { continue; }
        let n = norm(n);
        let (mut dmin, mut dmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for &f in faces { for &vi in &m.f[f as usize] { let d = dot(m.v[vi as usize], n); dmin = dmin.min(d); dmax = dmax.max(d); } }
        if dmax - dmin > 2.0 * tol { continue; }
        let k = (0..3).max_by(|&i, &j| n[i].abs().partial_cmp(&n[j].abs()).unwrap()).unwrap();
        let n = if n[k] < 0.0 { scale(n, -1.0) } else { n };
        match groups.iter_mut().find(|g| dot(g.0, n).abs() >= 0.9998) {
            Some(g) => g.1 += area,
            None => groups.push((n, area)),
        }
    }
    groups.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    groups.into_iter().take(top).map(|g| refine_normal(&na, g.0)).collect()
}

/// Mean-shift the normal onto the faces that agree with it, with a window that shrinks from 1° to
/// 0.1°. Flat regions grow across near-tangent facets at the start of a fillet, which tilts a plain
/// area-weighted mean by a fraction of a degree (0.4 mm across a 100 mm face). A wide first window
/// lets the large true flats outvote those facets; the narrow last one excludes them.
fn refine_normal(na: &[(V3, f64)], mut n: V3) -> V3 {
    for deg in [1.0f64, 0.5, 0.25, 0.1] {
        let cos = deg.to_radians().cos();
        for _ in 0..3 {
            let mut acc = [0.0; 3];
            for (fn_, a) in na {
                let d = dot(*fn_, n);
                if d.abs() >= cos { acc = add(acc, scale(*fn_, a * d.signum())); }
            }
            if len(acc) == 0.0 { break; }
            n = norm(acc);
        }
    }
    let k = (0..3).max_by(|&i, &j| n[i].abs().partial_cmp(&n[j].abs()).unwrap()).unwrap();
    if n[k] < 0.0 { scale(n, -1.0) } else { n }
}

pub struct Frame {
    /// world = matrix * aligned (4x4, row-major, millimetres)
    pub matrix: [[f64; 4]; 4],
    pub aligned: Mesh,
}

/// Frame with Z along `z_axis`, X along the major in-plane direction, origin at the minimum corner.
pub fn make_frame(m: &Mesh, z_axis: V3) -> Frame {
    let z = norm(z_axis);
    let n = m.v.len() as f64;
    let mean = m.v.iter().fold([0.0; 3], |s, p| add(s, *p));
    let mean = scale(mean, 1.0 / n);
    // in-plane basis (u, w) perpendicular to z
    let helper = if z[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let u = norm(cross(z, helper));
    let w = cross(z, u);
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for p in &m.v {
        let d = sub(*p, mean);
        let (a, b) = (dot(d, u), dot(d, w));
        sxx += a * a; sxy += a * b; syy += b * b;
    }
    let th = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let x = add(scale(u, th.cos()), scale(w, th.sin()));
    let y = cross(z, x);
    let rows = [x, y, z];
    let mut aligned: Vec<V3> = m.v.iter().map(|p| [dot(rows[0], *p), dot(rows[1], *p), dot(rows[2], *p)]).collect();
    let mut mins = [f64::INFINITY; 3];
    for p in &aligned { for k in 0..3 { mins[k] = mins[k].min(p[k]); } }
    for p in aligned.iter_mut() { *p = sub(*p, mins); }
    // world = R^T (aligned + mins)
    let mut mat = IDENTITY4;
    for r in 0..3 {
        for c in 0..3 { mat[r][c] = rows[c][r]; }
        mat[r][3] = (0..3).map(|c| rows[c][r] * mins[c]).sum();
    }
    Frame { matrix: mat, aligned: Mesh { v: aligned, f: m.f.clone() } }
}
