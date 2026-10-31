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
