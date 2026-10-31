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

/// A layer as a stack of sections (ascending z). Every section has the same ring structure, so
/// the side walls between consecutive sections are lofted quads (vertical for a plain layer,
/// slanted or curved where an edge treatment moves the outline).
pub struct Stack {
    pub z: Vec<f64>,
    pub sections: Vec<Vec<Poly>>,
}

fn mid_polys(a: &[Poly], b: &[Poly]) -> Vec<Poly> {
    let avg = |r: &Ring, q: &Ring| -> Ring { r.iter().zip(q).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0]).collect() };
    a.iter().zip(b).map(|(p, q)| Poly {
        outer: avg(&p.outer, &q.outer),
        holes: p.holes.iter().zip(&q.holes).map(|(h, g)| avg(h, g)).collect(),
    }).collect()
}

/// Exterior surface of stacked layers as a triangle soup, plus the volume.
/// Walls are exterior by construction (a section's boundary is the solid's boundary at that
/// height); between layers the exterior is the part of one end section not covered by the other.
/// No boolean union is used: a union of many thin coplanar slabs leaves stray internal shells.
pub fn surface_from_stacks(stacks: &[Stack]) -> (Mesh, f64) {
    let mut m = Mesh::default();
    let mut vol = 0.0;
    // `up`: the cap faces +z (counter-clockwise seen from above), else -z.
    let cap = |m: &mut Mesh, polys: &[Poly], z: f64, up: bool| {
        for p in polys {
            if p.area() < 1e-9 { continue; }
            let (v2, tris) = triangulate(p);
            let base = m.v.len() as u32;
            for v in &v2 { m.v.push([v[0], v[1], z]); }
            for t in tris {
                let ccw = ring_area_signed(&[v2[t[0]], v2[t[1]], v2[t[2]]]) > 0.0;
                let (i, j, k) = (base + t[0] as u32, base + t[1] as u32, base + t[2] as u32);
                if ccw == up { m.f.push([i, j, k]) } else { m.f.push([i, k, j]) }
            }
        }
    };
    let mut prev: Option<(&[Poly], f64)> = None;
    for st in stacks {
        if st.z.len() < 2 || st.z[st.z.len() - 1] - st.z[0] <= 1e-6 { continue; }
        for j in 0..st.z.len() - 1 {
            let (z0, z1) = (st.z[j], st.z[j + 1]);
            if z1 - z0 <= 1e-9 { continue; }
            let (a, b) = (&st.sections[j], &st.sections[j + 1]);
            let mid = mid_polys(a, b);
            vol += (z1 - z0) / 6.0 * (total_area(a) + 4.0 * total_area(&mid) + total_area(b));
            for (pa, pb) in a.iter().zip(b.iter()) {
                // outer rings counter-clockwise, holes clockwise: outward-facing walls
                let rings_a = std::iter::once(&pa.outer).chain(pa.holes.iter());
                let rings_b = std::iter::once(&pb.outer).chain(pb.holes.iter());
                for (idx, (ra, rb)) in rings_a.zip(rings_b).enumerate() {
                    let (mut ra, mut rb) = (ra.clone(), rb.clone());
                    if (ring_area_signed(&ra) > 0.0) != (idx == 0) { ra.reverse(); rb.reverse(); }
                    let n = ra.len();
                    for i in 0..n {
                        let k = (i + 1) % n;
                        let base = m.v.len() as u32;
                        m.v.extend_from_slice(&[[ra[i][0], ra[i][1], z0], [ra[k][0], ra[k][1], z0],
                                                [rb[k][0], rb[k][1], z1], [rb[i][0], rb[i][1], z1]]);
                        m.f.push([base, base + 1, base + 2]);
                        m.f.push([base, base + 2, base + 3]);
                    }
                }
            }
        }
        let (first, last) = (&st.sections[0], &st.sections[st.sections.len() - 1]);
        let (zf, zl) = (st.z[0], st.z[st.z.len() - 1]);
        match prev {
            Some((pp, ptop)) if (ptop - zf).abs() <= 1e-6 => {
                cap(&mut m, &difference(pp, first), zf, true);
                cap(&mut m, &difference(first, pp), zf, false);
            }
            Some((pp, ptop)) => { cap(&mut m, pp, ptop, true); cap(&mut m, first, zf, false); }
            None => cap(&mut m, first, zf, false),
        }
        prev = Some((last, zl));
    }
    if let Some((pp, ptop)) = prev { cap(&mut m, pp, ptop, true); }
    (m, vol)
}

/// Exterior surface of constant-section slabs (a stack per slab).
pub fn surface_from_slabs(slabs: &[Slab]) -> (Mesh, f64) {
    let stacks: Vec<Stack> = slabs.iter().filter(|s| s.z1 - s.z0 > 1e-6)
        .map(|s| Stack { z: vec![s.z0, s.z1], sections: vec![s.polys.to_vec(), s.polys.to_vec()] }).collect();
    surface_from_stacks(&stacks)
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
