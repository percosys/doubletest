//! Helpers to generate test parts (also used by the integration tests).

use crate::geom::*;
use crate::mesh::{clean, Mesh};
use crate::poly::{Poly, Ring};
use crate::verify::{surface_from_slabs, Slab};

pub fn circle(cx: f64, cy: f64, r: f64, n: usize) -> Ring {
    (0..n).map(|i| { let a = std::f64::consts::TAU * i as f64 / n as f64; [cx + r * a.cos(), cy + r * a.sin()] }).collect()
}

pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Ring { vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]] }

pub fn poly(outer: Ring, holes: Vec<Ring>) -> Poly { Poly { outer, holes } }

/// A closed, welded solid from stacked slabs (z0, z1, polygons).
pub fn slab_mesh(slabs: &[(f64, f64, Vec<Poly>)]) -> Mesh {
    let s: Vec<Slab> = slabs.iter().map(|(a, b, p)| Slab { z0: *a, z1: *b, polys: p }).collect();
    clean(&surface_from_slabs(&s).0, 1e-6)
}

pub fn uv_sphere(r: f64, n_lat: usize, n_lon: usize) -> Mesh {
    let mut m = Mesh::default();
    for i in 0..=n_lat {
        let th = std::f64::consts::PI * i as f64 / n_lat as f64;
        for j in 0..n_lon {
            let ph = std::f64::consts::TAU * j as f64 / n_lon as f64;
            m.v.push([r * th.sin() * ph.cos(), r * th.sin() * ph.sin(), r * th.cos()]);
        }
    }
    for i in 0..n_lat {
        for j in 0..n_lon {
            let (a, b) = ((i * n_lon + j) as u32, (i * n_lon + (j + 1) % n_lon) as u32);
            let (c, d) = (a + n_lon as u32, b + n_lon as u32);
            m.f.push([a, b, d]);
            m.f.push([a, d, c]);
        }
    }
    clean(&m, 1e-9)
}

/// Rotate about `axis` by `angle` (radians), then translate.
pub fn transform(m: &Mesh, axis: V3, angle: f64, t: V3) -> Mesh {
    let k = norm(axis);
    let (s, c) = angle.sin_cos();
    let mut out = m.clone();
    for p in out.v.iter_mut() {
        let r = add(add(scale(*p, c), scale(cross(k, *p), s)), scale(k, dot(k, *p) * (1.0 - c)));
        *p = add(r, t);
    }
    out
}

pub fn write_stl(m: &Mesh, path: &str) {
    let mut b = vec![0u8; 80];
    b.extend_from_slice(&(m.f.len() as u32).to_le_bytes());
    for i in 0..m.f.len() {
        b.extend_from_slice(&[0u8; 12]);
        for v in m.tri(i) { for k in 0..3 { b.extend_from_slice(&(v[k] as f32).to_le_bytes()); } }
        b.extend_from_slice(&[0u8; 2]);
    }
    std::fs::write(path, b).unwrap();
}

/// Loft between stacked sections that share one ring structure (ring 0 outer, then holes,
/// each with the same vertex count in every section): slanted or curved side walls,
/// flat end caps. Used to make chamfered, filleted and countersunk test parts.
pub fn loft(sections: &[(f64, Vec<Ring>)]) -> Mesh {
    use crate::poly::{ring_area_signed, triangulate};
    let mut m = Mesh::default();
    let orient = |r: &Ring, outer: bool| -> Ring {
        let mut r = r.clone();
        if (ring_area_signed(&r) > 0.0) != outer { r.reverse(); }
        r
    };
    // orient every ring the same way in every section (outer CCW, holes CW), keeping vertex order
    let secs: Vec<(f64, Vec<Ring>)> = sections.iter().map(|(z, rings)| {
        (*z, rings.iter().enumerate().map(|(i, r)| orient(r, i == 0)).collect())
    }).collect();
    for w in secs.windows(2) {
        let (z0, r0) = (&w[0].0, &w[0].1);
        let (z1, r1) = (&w[1].0, &w[1].1);
        for (ra, rb) in r0.iter().zip(r1.iter()) {
            let n = ra.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let base = m.v.len() as u32;
                m.v.extend_from_slice(&[[ra[i][0], ra[i][1], *z0], [ra[j][0], ra[j][1], *z0],
                                        [rb[j][0], rb[j][1], *z1], [rb[i][0], rb[i][1], *z1]]);
                m.f.push([base, base + 1, base + 2]);
                m.f.push([base, base + 2, base + 3]);
            }
        }
    }
    for (idx, up) in [(0usize, false), (secs.len() - 1, true)] {
        let (z, rings) = &secs[idx];
        let p = Poly { outer: rings[0].clone(), holes: rings[1..].to_vec() };
        let (v2, tris) = triangulate(&p);
        let base = m.v.len() as u32;
        for v in &v2 { m.v.push([v[0], v[1], *z]); }
        for t in tris {
            let ccw = ring_area_signed(&[v2[t[0]], v2[t[1]], v2[t[2]]]) > 0.0;
            let (i, j, k) = (base + t[0] as u32, base + t[1] as u32, base + t[2] as u32);
            if ccw == up { m.f.push([i, j, k]) } else { m.f.push([i, k, j]) }
        }
    }
    clean(&m, 1e-6)
}

/// Rectangle inset by `o` on every side (a mitred offset), as a ring.
pub fn inset_rect(x0: f64, y0: f64, x1: f64, y1: f64, o: f64) -> Ring { rect(x0 + o, y0 + o, x1 - o, y1 - o) }

/// Solid of revolution about Z of a closed (radius, z) profile polygon (it may touch the axis).
pub fn revolve_solid(profile: &[(f64, f64)], n: usize) -> Mesh {
    let mut m = Mesh::default();
    let k = profile.len();
    for i in 0..k {
        let (a, b) = (profile[i], profile[(i + 1) % k]);
        if a.0.abs() < 1e-12 && b.0.abs() < 1e-12 { continue; }
        let base = m.v.len() as u32;
        for j in 0..=n {
            let t = std::f64::consts::TAU * j as f64 / n as f64;
            m.v.push([a.0 * t.cos(), a.0 * t.sin(), a.1]);
            m.v.push([b.0 * t.cos(), b.0 * t.sin(), b.1]);
        }
        for j in 0..n {
            let i0 = base + 2 * j as u32;
            m.f.push([i0, i0 + 1, i0 + 3]);
            m.f.push([i0, i0 + 3, i0 + 2]);
        }
    }
    clean(&m, 1e-6)
}
