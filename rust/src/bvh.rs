//! Triangle bounding-box tree with exact point-to-triangle distance (method M9).

use crate::geom::*;

#[derive(Clone)]
struct Node {
    lo: V3,
    hi: V3,
    left: u32,
    right: u32,
    start: u32,
    count: u32,
}

pub struct Bvh {
    nodes: Vec<Node>,
    tris: Vec<[V3; 3]>,
}

fn box_dist2(p: V3, lo: V3, hi: V3) -> f64 {
    let mut d = 0.0;
    for k in 0..3 {
        let e = (lo[k] - p[k]).max(0.0).max(p[k] - hi[k]);
        d += e * e;
    }
    d
}

/// Closest point on triangle abc to p (region test, after Ericson).
pub fn closest_on_triangle(p: V3, a: V3, b: V3, c: V3) -> V3 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 { return a; }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 { return b; }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add(a, scale(ab, d1 / (d1 - d3)));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 { return c; }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add(a, scale(ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return add(b, scale(sub(c, b), w));
    }
    let denom = 1.0 / (va + vb + vc);
    add(a, add(scale(ab, vb * denom), scale(ac, vc * denom)))
}

impl Bvh {
    pub fn new(tris: Vec<[V3; 3]>) -> Bvh {
        let mut idx: Vec<u32> = (0..tris.len() as u32).collect();
        let cent: Vec<V3> = tris.iter().map(|t| scale(add(add(t[0], t[1]), t[2]), 1.0 / 3.0)).collect();
        let mut nodes = Vec::with_capacity(tris.len() / 2 + 1);
        let n = idx.len();
        if n > 0 { Self::build(&tris, &cent, &mut idx, 0, n, &mut nodes); }
        let ordered: Vec<[V3; 3]> = idx.iter().map(|&i| tris[i as usize]).collect();
        Bvh { nodes, tris: ordered }
    }

    fn build(tris: &[[V3; 3]], cent: &[V3], idx: &mut [u32], s: usize, e: usize, nodes: &mut Vec<Node>) -> u32 {
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for &i in &idx[s..e] {
            for v in &tris[i as usize] {
                for k in 0..3 { lo[k] = lo[k].min(v[k]); hi[k] = hi[k].max(v[k]); }
            }
        }
        let me = nodes.len() as u32;
        nodes.push(Node { lo, hi, left: 0, right: 0, start: s as u32, count: (e - s) as u32 });
        if e - s <= 4 { return me; }
        let ext = sub(hi, lo);
        let axis = if ext[0] >= ext[1] && ext[0] >= ext[2] { 0 } else if ext[1] >= ext[2] { 1 } else { 2 };
        let mid = (s + e) / 2;
        idx[s..e].select_nth_unstable_by(mid - s, |&a, &b| cent[a as usize][axis].partial_cmp(&cent[b as usize][axis]).unwrap());
        let l = Self::build(tris, cent, idx, s, mid, nodes);
        let r = Self::build(tris, cent, idx, mid, e, nodes);
        let nd = &mut nodes[me as usize];
        nd.left = l;
        nd.right = r;
        nd.count = 0;
        me
    }

    /// Distance from p to the nearest triangle (infinity for an empty tree).
    pub fn distance(&self, p: V3) -> f64 {
        if self.nodes.is_empty() { return f64::INFINITY; }
        let mut best = f64::INFINITY;
        let mut stack: Vec<u32> = vec![0];
        while let Some(i) = stack.pop() {
            let n = &self.nodes[i as usize];
            if box_dist2(p, n.lo, n.hi) >= best { continue; }
            if n.count > 0 {
                for t in &self.tris[n.start as usize..(n.start + n.count) as usize] {
                    let q = closest_on_triangle(p, t[0], t[1], t[2]);
                    let d = sub(p, q);
                    best = best.min(dot(d, d));
                }
            } else {
                let (l, r) = (n.left, n.right);
                let (dl, dr) = (box_dist2(p, self.nodes[l as usize].lo, self.nodes[l as usize].hi),
                                box_dist2(p, self.nodes[r as usize].lo, self.nodes[r as usize].hi));
                // visit the nearer child first (pushed last)
                if dl < dr { stack.push(r); stack.push(l); } else { stack.push(l); stack.push(r); }
            }
        }
        best.sqrt()
    }
}
