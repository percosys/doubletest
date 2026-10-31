//! STL -> build plan: deterministic 2.5D rebuild with verification (methods M1-M4, M9).

use crate::layers::*;
use crate::mesh::{clean, read_stl, Mesh};
use crate::plan::*;
use crate::planes::{dominant_axes, make_frame, Frame};
use crate::verify::{deviation, surface_from_slabs, Slab};
use std::time::Instant;

struct Sharp {
    frame: Frame,
    layers: Vec<Layer>,
    fit_err: f64,
    simp: f64,
    dev: DevOut,
}

fn verify_layers(am: &Mesh, layers: &[Layer]) -> DevOut {
    let slabs: Vec<Slab> = layers.iter().map(|l| Slab { z0: l.z0, z1: l.z1, polys: &l.polys }).collect();
    let (m, vol) = surface_from_slabs(&slabs);
    deviation(am, &m, vol, 8000)
}

fn passes(s: &Sharp, tol: f64) -> bool { s.dev.max_mm + s.simp <= tol && s.fit_err <= tol }

/// Layers along one axis. The merged-sliver variant is preferred unless the unmerged one is
/// much better (a few percent difference is sampling noise).
fn sharp(mesh: &Mesh, axis: [f64; 3], tol: f64) -> Result<Sharp, String> {
    let frame = make_frame(mesh, axis);
    let (layers0, fit_err, simp) = build_layers(&frame.aligned, 0.1 * tol)?;
    let merged = merge_slivers(&layers0, tol);
    let mut variants = vec![merged];
    if variants[0].len() != layers0.len() { variants.push(layers0); }
    let mut best: Option<(Vec<Layer>, DevOut)> = None;
    for layers in variants {
        let dev = verify_layers(&frame.aligned, &layers);
        let done = dev.max_mm + simp <= tol;
        let better = match &best { None => true, Some((_, b)) => dev.max_mm < 0.5 * b.max_mm };
        if better { best = Some((layers, dev)); }
        if done { break; }
    }
    let (layers, dev) = best.unwrap();
    Ok(Sharp { frame, layers, fit_err, simp, dev })
}

pub fn analyze(path: &str, tol: f64) -> Result<Plan, String> {
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
    for axis in dominant_axes(&mesh, 3) {
        let label = format!("axis [{:.3}, {:.3}, {:.3}]", axis[0], axis[1], axis[2]);
        match sharp(&mesh, axis, tol) {
            Err(e) => plan.notes.push(format!("{label}: {e}")),
            Ok(s) => {
                plan.notes.push(format!("{label}: {} layers, max deviation {:.4} mm", s.layers.len(), s.dev.max_mm));
                let done = passes(&s, tol);
                if best.as_ref().map_or(true, |b| s.dev.max_mm < b.dev.max_mm) { best = Some(s); }
                if done { break; }
            }
        }
    }
    match best {
        None => plan.notes.push("no 2.5D interpretation found; falling back to mesh conversion".into()),
        Some(b) => {
            plan.deviation = Some(b.dev.clone());
            plan.deviation_sharp = Some(b.dev.clone());
            plan.circle_fit_error_mm = (b.fit_err * 1e5).round() / 1e5;
            plan.simplify_error_mm = (b.simp * 1e5).round() / 1e5;
            if passes(&b, tol) {
                plan.strategy = "extruded_layers".into();
                plan.frame = Some(b.frame.matrix);
                plan.layers = b.layers.iter().map(|l| LayerOut { z0: l.z0, z1: l.z1, regions: l.regions.clone() }).collect();
            } else {
                plan.notes.push(format!(
                    "best 2.5D rebuild is off by up to {:.3} mm (tolerance {} mm); falling back to mesh conversion",
                    b.dev.max_mm + b.simp, tol));
            }
        }
    }
    Ok(finish(plan, t0))
}

fn finish(mut plan: Plan, t0: Instant) -> Plan {
    if let Some(o) = plan.stats.as_object_mut() {
        o.insert("seconds".into(), serde_json::json!((t0.elapsed().as_secs_f64() * 100.0).round() / 100.0));
    }
    plan
}
