//! Plane section of a triangle mesh into closed loops (method M4, first half).

use crate::geom::*;
use crate::mesh::Mesh;
use std::collections::HashMap;

type Key = (u32, u32);

/// Closed loops where the plane z = `z` cuts the mesh. A vertex exactly on the plane counts as
/// below it (strict "above" test), so every crossing is consistent between neighbours and the
/// loops always close. Each crossing is identified by its mesh edge, so no float matching is needed.
pub fn section_loops(m: &Mesh, z: f64) -> Vec<Vec<V2>> {
    let mut pts: HashMap<Key, V2> = HashMap::new();
    let mut nbr: HashMap<Key, [Option<Key>; 2]> = HashMap::new();
    for t in &m.f {
        let above = [m.v[t[0] as usize][2] > z, m.v[t[1] as usize][2] > z, m.v[t[2] as usize][2] > z];
        if above[0] == above[1] && above[1] == above[2] { continue; }
        let mut crossing: Vec<Key> = Vec::with_capacity(2);
        for k in 0..3 {
            let (i, j) = (t[k], t[(k + 1) % 3]);
            if above[k] != above[(k + 1) % 3] {
                let key = (i.min(j), i.max(j));
                pts.entry(key).or_insert_with(|| {
                    let (a, b) = (m.v[key.0 as usize], m.v[key.1 as usize]);
                    let s = (z - a[2]) / (b[2] - a[2]);
                    [a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s]
                });
                crossing.push(key);
            }
        }
        if crossing.len() == 2 {
            let (a, b) = (crossing[0], crossing[1]);
            let e = nbr.entry(a).or_insert([None, None]);
            if e[0].is_none() { e[0] = Some(b) } else { e[1] = Some(b) }
            let e = nbr.entry(b).or_insert([None, None]);
            if e[0].is_none() { e[0] = Some(a) } else { e[1] = Some(a) }
        }
    }
    let mut visited: HashMap<Key, ()> = HashMap::new();
    let mut loops = Vec::new();
    let mut keys: Vec<Key> = nbr.keys().copied().collect();
    keys.sort();
    for start in keys {
        if visited.contains_key(&start) { continue; }
        let mut ring: Vec<V2> = Vec::new();
        let mut prev: Option<Key> = None;
        let mut cur = start;
        let mut closed = false;
        loop {
            visited.insert(cur, ());
            ring.push(pts[&cur]);
            let n = nbr[&cur];
            let next = match prev {
                None => n[0],
                Some(p) => if n[0] == Some(p) { n[1] } else { n[0] },
            };
            let Some(next) = next else { break };
            if next == start { closed = ring.len() >= 3; break; }
            if visited.contains_key(&next) { break; }
            prev = Some(cur);
            cur = next;
        }
        if closed { loops.push(ring); }
    }
    loops
}
