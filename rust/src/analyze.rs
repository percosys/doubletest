//! STL -> build plan: deterministic 2.5D rebuild with verification (methods M1-M4, M6, M9).

use crate::geom::*;
use crate::layers::*;
use crate::mesh::{clean, read_stl, Mesh};
use crate::plan::*;
use crate::planes::{dominant_axes, make_frame, Frame};
use crate::treatments::{detect, layer_sections, Treatment};
use crate::verify::{deviation, surface_from_stacks, Stack};
use std::time::Instant;

fn trace(t0: Instant, msg: &str) {
    if std::env::var("STL2CAD_TRACE").is_ok() { eprintln!("[stl2cad {:6.2}s] {msg}", t0.elapsed().as_secs_f64()); }
}

struct Cand {
    layers: Vec<Layer>,
    /// Deviation before any edge treatment.
    sharp: DevOut,
    dev: DevOut,
    ts: Vec<Treatment>,
}

struct Sharp {
    frame: Frame,
    fit_err: f64,
    simp: f64,
    cands: Vec<Cand>,
    chosen: usize,
}

impl Sharp {
    fn best(&self) -> &Cand { &self.cands[self.chosen] }
    fn passes(&self, tol: f64) -> bool { self.best().dev.max_mm + self.simp <= tol && self.fit_err <= tol }
}

fn verify_layers(am: &Mesh, layers: &[Layer], ts: &[Treatment], _tol: f64) -> DevOut {
    let stacks: Vec<Stack> = layers.iter().enumerate().map(|(k, l)| layer_sections(l, k, ts)).collect();
    let t = Instant::now();
    let (m, vol) = surface_from_stacks(&stacks);
    let t_surface = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let d = deviation(am, &m, vol, 8000);
    if std::env::var("STL2CAD_TRACE").is_ok() {
        eprintln!("[verify] {} layers -> {} triangles: surface {:.2}s, distances {:.2}s",
                  stacks.len(), m.f.len(), t_surface, t.elapsed().as_secs_f64());
    }
    d
}

/// Layers along one axis. The merged-sliver variant is preferred unless the unmerged one is
/// much better (a few percent difference is sampling noise).
fn sharp(mesh: &Mesh, axis: [f64; 3], tol: f64) -> Result<Sharp, String> {
    let frame = make_frame(mesh, axis);
    let (layers0, fit_err, simp) = build_layers(&frame.aligned, 0.1 * tol)?;
    let merged = merge_slivers(&layers0, tol);
    let mut variants = vec![merged];
    if variants[0].len() != layers0.len() { variants.push(layers0); }
    let mut cands: Vec<Cand> = Vec::new();
    let mut chosen = 0;
    for layers in variants {
        let dev = verify_layers(&frame.aligned, &layers, &[], tol);
        let done = dev.max_mm + simp <= tol;
        cands.push(Cand { layers, sharp: dev.clone(), dev, ts: vec![] });
        let n = cands.len() - 1;
        if n > 0 && cands[n].dev.max_mm < 0.5 * cands[chosen].dev.max_mm { chosen = n; }
        if done { break; }
    }
    Ok(Sharp { frame, fit_err, simp, cands, chosen })
}

/// Try to close the remaining gap with chamfers/fillets, on the chosen variant first.
fn treat(s: &mut Sharp, tol: f64, max_edge: f64, t0: Instant) {
    let mut order: Vec<usize> = vec![s.chosen];
    order.extend((0..s.cands.len()).filter(|&i| i != s.chosen));
    let mut winner: Option<usize> = None;
    for i in order {
        let ts = detect(&s.frame.aligned, &s.cands[i].layers, tol, max_edge);
        trace(t0, &format!("detected {} treatments on variant {i}", ts.len()));
        if ts.is_empty() { continue; }
        let dev = verify_layers(&s.frame.aligned, &s.cands[i].layers, &ts, tol);
        trace(t0, &format!("verified variant {i} with treatments: max {:.4} mm", dev.max_mm));
        if dev.max_mm < s.cands[i].dev.max_mm {
            s.cands[i].dev = dev;
            s.cands[i].ts = ts;
        }
        if s.cands[i].dev.max_mm + s.simp <= tol { winner = Some(i); break; }
    }
    s.chosen = winner.unwrap_or_else(|| {
        (0..s.cands.len()).min_by(|&a, &b| s.cands[a].dev.max_mm.partial_cmp(&s.cands[b].dev.max_mm).unwrap()).unwrap()
    });
}

fn curve_dist(p: V2, curves: &[Vec<V2>]) -> f64 {
    curves.iter().flat_map(|c| c.windows(2).map(move |w| point_segment_dist(p, w[0], w[1]))).fold(f64::INFINITY, f64::min)
}

/// Outline edges of a layer that lie on the treatment's curves (for Fusion edge picking).
fn edge_descriptors(t: &Treatment, layer: &LayerOut, thr: f64) -> Vec<serde_json::Value> {
    let z = (t.z * 1e5).round() / 1e5;
    let mut out = Vec::new();
    for r in &layer.regions {
        for lp in std::iter::once(&r.outer).chain(r.holes.iter()) {
            match lp {
                Loop::Circle { cx, cy, r } => {
                    let ok = (0..16).all(|i| {
                        let a = std::f64::consts::TAU * i as f64 / 16.0;
                        curve_dist([cx + r * a.cos(), cy + r * a.sin()], &t.curves) < thr
                    });
                    if ok { out.push(serde_json::json!({"type": "circle", "cx": cx, "cy": cy, "r": r, "z": z})); }
                }
                Loop::Polyline { pts } => {
                    for (i, p0) in pts.iter().enumerate() {
                        let p1 = pts[(i + 1) % pts.len()];
                        let mid = [(p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0];
                        if curve_dist(mid, &t.curves) < thr && curve_dist(*p0, &t.curves) < thr {
                            out.push(serde_json::json!({"type": "segment", "p0": p0, "p1": p1, "z": z}));
                        }
                    }
                }
            }
        }
    }
    out
}

pub fn analyze(path: &str, tol: f64) -> Result<Plan, String> { analyze_with(path, tol, 6.0) }

pub fn analyze_with(path: &str, tol: f64, max_edge: f64) -> Result<Plan, String> {
    let t0 = Instant::now();
    let mesh = clean(&read_stl(path)?, 1e-5);
    let mut plan = Plan::new(tol);
    let ext = mesh.extents();
    let ext_r: Vec<f64> = ext.iter().map(|e| (e * 1e3).round() / 1e3).collect();
    plan.stats = serde_json::json!({
        "faces": mesh.f.len(), "watertight": mesh.is_closed(), "extents_mm": ext_r,
        "volume_mm3": (mesh.volume() * 10.0).round() / 10.0,
    });
    if !mesh.is_closed() {
        plan.notes.push("mesh is not watertight; 2.5D rebuild skipped".into());
        return Ok(finish(plan, t0));
    }
    let mut best: Option<Sharp> = None;
    let mut first: Option<Sharp> = None;
    let mut found_first = false;
    for axis in dominant_axes(&mesh, 3) {
        let label = format!("axis [{:.3}, {:.3}, {:.3}]", axis[0], axis[1], axis[2]);
        match sharp(&mesh, axis, tol) {
            Err(e) => plan.notes.push(format!("{label}: {e}")),
            Ok(s) => {
                plan.notes.push(format!("{label}: {} layers, max deviation {:.4} mm", s.best().layers.len(), s.best().dev.max_mm));
                let done = s.passes(tol);
                // keep a copy of the main axis result for the treatment search
                let is_first = !found_first;
                found_first = true;
                if is_first { first = Some(clone_sharp(&s)); }
                if best.as_ref().map_or(true, |b| s.best().dev.max_mm < b.best().dev.max_mm) { best = Some(s); }
                if done { break; }
            }
        }
    }
    // Edge treatments are searched on one axis, and only if needed. Sharp deviation does not
    // predict recoverability, so keep the main axis unless another is at least twice as accurate.
    if let Some(b) = best.as_mut() {
        if !b.passes(tol) {
            if let Some(f) = first {
                if b.best().dev.max_mm > 0.5 * f.best().dev.max_mm { *b = f; }
            }
            trace(t0, "sharp stage done");
            treat(b, tol, max_edge, t0);
            plan.notes.push(format!("edge treatments: {} found, max deviation now {:.4} mm",
                                    b.best().ts.len(), b.best().dev.max_mm));
        }
    }
    match best {
        None => plan.notes.push("no 2.5D interpretation found; falling back to mesh conversion".into()),
        Some(b) => {
            let c = b.best();
            plan.deviation = Some(c.dev.clone());
            plan.deviation_sharp = Some(c.sharp.clone());
            plan.circle_fit_error_mm = (b.fit_err * 1e5).round() / 1e5;
            plan.simplify_error_mm = (b.simp * 1e5).round() / 1e5;
            if b.passes(tol) {
                plan.strategy = "extruded_layers".into();
                plan.frame = Some(b.frame.matrix);
                plan.layers = c.layers.iter().map(|l| LayerOut { z0: l.z0, z1: l.z1, regions: l.regions.clone() }).collect();
                let thr = (0.02f64).max(0.2 * tol);
                for t in &c.ts {
                    let edges = edge_descriptors(t, &plan.layers[t.layer], thr);
                    if edges.is_empty() {
                        plan.review.push(format!("{} at z={:.2} could not be tied to outline edges; skipped",
                                                 if t.kind == crate::treatments::Kind::Fillet { "fillet" } else { "chamfer" }, t.z));
                    }
                    plan.treatments.push(serde_json::json!({
                        "kind": if t.kind == crate::treatments::Kind::Fillet { "fillet" } else { "chamfer" },
                        "family": t.family.name(), "z": (t.z * 1e5).round() / 1e5,
                        "size_xy": (t.size_xy * 1e4).round() / 1e4, "size_z": (t.size_z * 1e4).round() / 1e4,
                        "fit_rms_mm": (t.rms * 1e5).round() / 1e5, "layer": t.layer, "edges": edges }));
                }
            } else {
                plan.notes.push(format!(
                    "best 2.5D rebuild is off by up to {:.3} mm (tolerance {} mm); falling back to mesh conversion",
                    c.dev.max_mm + b.simp, tol));
            }
        }
    }
    Ok(finish(plan, t0))
}

fn clone_sharp(s: &Sharp) -> Sharp {
    Sharp {
        frame: Frame { matrix: s.frame.matrix, aligned: s.frame.aligned.clone() },
        fit_err: s.fit_err, simp: s.simp, chosen: s.chosen,
        cands: s.cands.iter().map(|c| Cand { layers: c.layers.clone(), sharp: c.sharp.clone(), dev: c.dev.clone(), ts: c.ts.clone() }).collect(),
    }
}

fn finish(mut plan: Plan, t0: Instant) -> Plan {
    if let Some(o) = plan.stats.as_object_mut() {
        o.insert("seconds".into(), serde_json::json!((t0.elapsed().as_secs_f64() * 100.0).round() / 100.0));
    }
    plan
}
