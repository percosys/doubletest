//! Verification: exterior of stacked slabs without a boolean union, and two-sided distance (M9).

use crate::bvh::Bvh;
use crate::geom::*;
use crate::mesh::Mesh;
use crate::plan::DevOut;
use crate::poly::*;
use rayon::prelude::*;

/// A slab: constant cross-section between z0 and z1.
pub struct Slab<'a> {
    pub z0: f64,
    pub z1: f64,
    pub polys: &'a [Poly],
}

/// Exterior surface of stacked slabs as a triangle soup, plus the exact volume.
/// Walls are exterior by construction (a slab's boundary is the solid's boundary in its height
/// range); between slabs the exterior is the part of one slab not covered by its neighbour.
pub fn surface_from_slabs(slabs: &[Slab]) -> (Mesh, f64) {
    let mut m = Mesh::default();
    let mut vol = 0.0;
    // `up`: the cap faces +z (counter-clockwise seen from above), else -z.
    let mut cap = |m: &mut Mesh, polys: &[Poly], z: f64, up: bool| {
        for p in polys {
            if p.area() < 1e-9 { continue; }
            let (v2, tris) = triangulate(p);
            let base = m.v.len() as u32;
            for v in &v2 { m.v.push([v[0], v[1], z]); }
            for t in tris {
                let a2 = ring_area_signed(&[v2[t[0]], v2[t[1]], v2[t[2]]]);
                let ccw = a2 > 0.0;
                let (i, j, k) = (base + t[0] as u32, base + t[1] as u32, base + t[2] as u32);
                if ccw == up { m.f.push([i, j, k]) } else { m.f.push([i, k, j]) }
            }
        }
    };
    let mut prev: Option<(&[Poly], f64)> = None;
    for s in slabs {
        if s.z1 - s.z0 <= 1e-6 { continue; }
        vol += total_area(s.polys) * (s.z1 - s.z0);
        for p in s.polys {
            // outer rings counter-clockwise, holes clockwise: outward-facing walls
            let rings = std::iter::once((&p.outer, true)).chain(p.holes.iter().map(|h| (h, false)));
            for (ring, outer) in rings {
                let mut ring = ring.clone();
                if (ring_area_signed(&ring) > 0.0) != outer { ring.reverse(); }
                let n = ring.len();
                for i in 0..n {
                    let (a, b) = (ring[i], ring[(i + 1) % n]);
                    let base = m.v.len() as u32;
                    m.v.extend_from_slice(&[[a[0], a[1], s.z0], [b[0], b[1], s.z0], [b[0], b[1], s.z1], [a[0], a[1], s.z1]]);
                    m.f.push([base, base + 1, base + 2]);
                    m.f.push([base, base + 2, base + 3]);
                }
            }
        }
        match prev {
            Some((pp, ptop)) if (ptop - s.z0).abs() <= 1e-6 => {
                cap(&mut m, &difference(pp, s.polys), s.z0, true);
                cap(&mut m, &difference(s.polys, pp), s.z0, false);
            }
            Some((pp, ptop)) => { cap(&mut m, pp, ptop, true); cap(&mut m, s.polys, s.z0, false); }
            None => cap(&mut m, s.polys, s.z0, false),
        }
        prev = Some((s.polys, s.z1));
    }
    if let Some((pp, ptop)) = prev { cap(&mut m, pp, ptop, true); }
    (m, vol)
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

/// Area-weighted random points on the surface.
pub fn sample_surface(m: &Mesh, n: usize, seed: u64) -> Vec<V3> {
    let mut cum = Vec::with_capacity(m.f.len());
    let mut total = 0.0;
    for i in 0..m.f.len() { total += m.normal_area(i).1; cum.push(total); }
    let mut rng = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1);
    (0..n).map(|_| {
        let r = rng.next() * total;
        let i = cum.partition_point(|&c| c < r).min(m.f.len() - 1);
        let [a, b, c] = m.tri(i);
        let (u, v) = (rng.next(), rng.next());
        let (su, w) = (u.sqrt(), v);
        add(add(scale(a, 1.0 - su), scale(b, su * (1.0 - w))), scale(c, su * w))
    }).collect()
}

pub fn tri_soup(m: &Mesh) -> Vec<[V3; 3]> { (0..m.f.len()).map(|i| m.tri(i)).collect() }

/// Two-sided surface distance and volume difference. `rebuilt_volume` is required for a soup.
pub fn deviation(original: &Mesh, rebuilt: &Mesh, rebuilt_volume: f64, n: usize) -> DevOut {
    let (bo, br) = (Bvh::new(tri_soup(original)), Bvh::new(tri_soup(rebuilt)));
    let po = sample_surface(original, n, 1);
    let pr = sample_surface(rebuilt, n, 2);
    let mut d: Vec<f64> = po.par_iter().map(|p| br.distance(*p)).collect();
    d.extend(pr.par_iter().map(|p| bo.distance(*p)).collect::<Vec<_>>());
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = d.iter().sum::<f64>() / d.len() as f64;
    let p99 = d[((d.len() as f64 * 0.99) as usize).min(d.len() - 1)];
    let v0 = original.volume();
    DevOut {
        max_mm: *d.last().unwrap(), mean_mm: mean, p99_mm: p99,
        volume_diff_pct: 100.0 * (rebuilt_volume - v0).abs() / v0.max(1e-9), samples: d.len(),
    }
}
