//! Triangle mesh: STL reading, cleaning and basic queries (method M1).

use crate::geom::*;
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub v: Vec<V3>,
    pub f: Vec<[u32; 3]>,
}

impl Mesh {
    pub fn tri(&self, i: usize) -> [V3; 3] {
        let t = self.f[i];
        [self.v[t[0] as usize], self.v[t[1] as usize], self.v[t[2] as usize]]
    }
    /// (unit normal, area) of a triangle; zero normal for a degenerate one.
    pub fn normal_area(&self, i: usize) -> (V3, f64) {
        let [a, b, c] = self.tri(i);
        let n = cross(sub(b, a), sub(c, a));
        let l = len(n);
        if l == 0.0 { ([0.0; 3], 0.0) } else { (scale(n, 1.0 / l), 0.5 * l) }
    }
    pub fn bounds(&self) -> (V3, V3) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in &self.v {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        (lo, hi)
    }
    pub fn extents(&self) -> V3 { let (lo, hi) = self.bounds(); sub(hi, lo) }
    pub fn diag(&self) -> f64 { len(self.extents()) }
    /// Absolute enclosed volume (sum of signed tetrahedra).
    pub fn volume(&self) -> f64 {
        let mut s = 0.0;
        for i in 0..self.f.len() {
            let [a, b, c] = self.tri(i);
            s += dot(a, cross(b, c));
        }
        (s / 6.0).abs()
    }
    pub fn area(&self) -> f64 { (0..self.f.len()).map(|i| self.normal_area(i).1).sum() }

    /// Closed: every undirected edge is shared by exactly two triangles.
    pub fn is_closed(&self) -> bool {
        let mut cnt: HashMap<(u32, u32), u32> = HashMap::new();
        for t in &self.f {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *cnt.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        cnt.values().all(|&c| c == 2)
    }

    /// Pairs of triangles that share an edge.
    pub fn adjacency(&self) -> Vec<(u32, u32)> {
        let mut first: HashMap<(u32, u32), u32> = HashMap::new();
        let mut out = Vec::new();
        for (i, t) in self.f.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                match first.get(&key) {
                    Some(&j) => out.push((j, i as u32)),
                    None => { first.insert(key, i as u32); }
                }
            }
        }
        out
    }
}

pub fn read_stl(path: &str) -> Result<Mesh, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    parse_stl(&bytes)
}

pub fn parse_stl(bytes: &[u8]) -> Result<Mesh, String> {
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if bytes.len() == 84 + 50 * n {
            let mut m = Mesh::default();
            m.v.reserve(3 * n);
            m.f.reserve(n);
            let rd = |o: usize| f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]) as f64;
            for i in 0..n {
                let base = 84 + 50 * i + 12;
                for k in 0..3 {
                    let o = base + 12 * k;
                    m.v.push([rd(o), rd(o + 4), rd(o + 8)]);
                }
                let b = (3 * i) as u32;
                m.f.push([b, b + 1, b + 2]);
            }
            return Ok(m);
        }
    }
    let text = std::str::from_utf8(bytes).map_err(|_| "not a binary or ASCII STL".to_string())?;
    let mut m = Mesh::default();
    let mut cur: Vec<V3> = Vec::new();
    let mut it = text.split_whitespace();
    while let Some(tok) = it.next() {
        if tok == "vertex" {
            let mut p = [0.0; 3];
            for k in 0..3 {
                p[k] = it.next().ok_or("truncated ASCII STL")?.parse::<f64>().map_err(|e| e.to_string())?;
            }
            cur.push(p);
            if cur.len() == 3 {
                let b = m.v.len() as u32;
                m.v.extend(cur.drain(..));
                m.f.push([b, b + 1, b + 2]);
            }
        }
    }
    if m.f.is_empty() { Err("no triangles found".into()) } else { Ok(m) }
}

/// Weld vertices closer than `tol`, drop degenerate triangles.
pub fn clean(m: &Mesh, tol: f64) -> Mesh {
    let mut map: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut out = Mesh::default();
    let mut remap = vec![0u32; m.v.len()];
    for (i, p) in m.v.iter().enumerate() {
        let key = ((p[0] / tol).round() as i64, (p[1] / tol).round() as i64, (p[2] / tol).round() as i64);
        remap[i] = *map.entry(key).or_insert_with(|| { out.v.push(*p); (out.v.len() - 1) as u32 });
    }
    for t in &m.f {
        let f = [remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]];
        if f[0] == f[1] || f[1] == f[2] || f[0] == f[2] { continue; }
        let (a, b, c) = (out.v[f[0] as usize], out.v[f[1] as usize], out.v[f[2] as usize]);
        if len(cross(sub(b, a), sub(c, a))) < 1e-14 { continue; }
        out.f.push(f);
    }
    out
}
