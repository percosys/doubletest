use stl2cad::analyze::analyze;
use stl2cad::bvh::Bvh;
use stl2cad::fit::fit_circle;
use stl2cad::mesh::{clean, parse_stl};
use stl2cad::plan::{Loop, Plan};
use stl2cad::section::section_loops;
use stl2cad::testutil::*;

fn tmp(name: &str) -> String {
    let d = std::env::temp_dir().join(format!("stl2cad_rs_{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d.join(name).to_string_lossy().into_owned()
}

fn hole_diameters(p: &Plan) -> Vec<f64> {
    let mut v: Vec<f64> = p.layers.iter().flat_map(|l| l.regions.iter()).flat_map(|r| r.holes.iter())
        .filter_map(|h| if let Loop::Circle { r, .. } = h { Some((2.0 * r * 100.0).round() / 100.0) } else { None }).collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v
}

fn plate() -> stl2cad::mesh::Mesh {
    slab_mesh(&[(0.0, 10.0, vec![poly(rect(0.0, 0.0, 60.0, 40.0),
        vec![circle(15.0, 20.0, 1.7, 64), circle(45.0, 20.0, 2.75, 64)])])])
}

#[test]
fn stl_roundtrip_and_closed() {
    let m = plate();
    assert!(m.is_closed());
    let p = tmp("rt.stl");
    write_stl(&m, &p);
    let back = clean(&parse_stl(&std::fs::read(&p).unwrap()).unwrap(), 1e-5);
    assert!((back.volume() - m.volume()).abs() < 1e-2 * m.volume() * 1e-2);
    assert!((m.volume() - (60.0 * 40.0 - std::f64::consts::PI * (1.7f64.powi(2) + 2.75f64.powi(2))) * 10.0).abs() < 5.0);
}

#[test]
fn section_of_a_plate_is_outline_plus_two_holes() {
    let loops = section_loops(&plate(), 5.0);
    assert_eq!(loops.len(), 3);
}

#[test]
fn circle_fit_recovers_centre_and_radius() {
    let c = fit_circle(&circle(3.0, -2.0, 7.5, 40)).unwrap();
    assert!((c.cx - 3.0).abs() < 1e-9 && (c.cy + 2.0).abs() < 1e-9 && (c.r - 7.5).abs() < 1e-9 && c.rms < 1e-9);
}

#[test]
fn bvh_distance_matches_brute_force() {
    let m = plate();
    let tris: Vec<_> = (0..m.f.len()).map(|i| m.tri(i)).collect();
    let b = Bvh::new(tris.clone());
    for p in [[30.0, 20.0, 5.0], [-5.0, 20.0, 5.0], [15.0, 20.0, 12.0], [61.0, 41.0, -2.0]] {
        let brute = tris.iter().map(|t| {
            let q = stl2cad::bvh::closest_on_triangle(p, t[0], t[1], t[2]);
            ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
        }).fold(f64::INFINITY, f64::min);
        assert!((b.distance(p) - brute).abs() < 1e-9);
    }
    assert!((b.distance([-5.0, 20.0, 5.0]) - 5.0).abs() < 1e-9);
}

#[test]
fn plate_with_holes_is_one_exact_layer() {
    let p = tmp("plate.stl");
    write_stl(&plate(), &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert_eq!(plan.layers.len(), 1);
    assert!((plan.layers[0].z1 - plan.layers[0].z0 - 10.0).abs() < 1e-3);
    assert_eq!(hole_diameters(&plan), vec![3.4, 5.5]);
    assert!(plan.deviation.as_ref().unwrap().max_mm < 0.01);
    assert_eq!(plan.llm_calls, 0);
}

#[test]
fn tilted_plate_is_still_exact() {
    let m = transform(&plate(), [0.3, 0.7, 0.2], 0.9, [12.0, -5.0, 3.0]);
    let p = tmp("tilt.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert_eq!(plan.layers.len(), 1);
    assert_eq!(hole_diameters(&plan), vec![3.4, 5.5]);
    let f = plan.frame.unwrap();
    // rotation part is orthonormal
    for i in 0..3 { for j in 0..3 {
        let d: f64 = (0..3).map(|k| f[k][i] * f[k][j]).sum();
        assert!((d - if i == j { 1.0 } else { 0.0 }).abs() < 1e-6);
    } }
}

#[test]
fn stepped_part_with_pocket_has_three_layers() {
    let m = slab_mesh(&[
        (0.0, 4.0, vec![poly(rect(0.0, 0.0, 60.0, 40.0), vec![rect(20.0, 15.0, 40.0, 25.0)])]),
        (4.0, 5.0, vec![poly(rect(0.0, 0.0, 60.0, 40.0), vec![])]),
        (5.0, 10.0, vec![poly(rect(15.0, 10.0, 45.0, 30.0), vec![])]),
    ]);
    let p = tmp("step.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert_eq!(plan.layers.len(), 3);
    assert!(plan.deviation.unwrap().max_mm < 0.01);
}

#[test]
fn l_bracket_outline_is_a_six_point_polyline() {
    let l = vec![[0.0, 0.0], [50.0, 0.0], [50.0, 10.0], [10.0, 10.0], [10.0, 40.0], [0.0, 40.0]];
    let p = tmp("l.stl");
    write_stl(&slab_mesh(&[(0.0, 8.0, vec![poly(l, vec![])])]), &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    match &plan.layers[0].regions[0].outer {
        Loop::Polyline { pts } => assert_eq!(pts.len(), 6),
        other => panic!("expected polyline, got {other:?}"),
    }
}

#[test]
fn matching_sliver_flange_is_merged_away() {
    let m = slab_mesh(&[
        (0.0, 2.5, vec![poly(rect(0.0, 0.0, 40.0, 40.0), vec![])]),
        (2.5, 2.52, vec![poly(rect(-0.01, -0.01, 40.01, 40.01), vec![])]),
        (2.52, 5.0, vec![poly(rect(0.0, 0.0, 40.0, 40.0), vec![])]),
    ]);
    let p = tmp("sliver.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert_eq!(plan.layers.len(), 1);
}

#[test]
fn thin_internal_cavity_is_not_merged_away() {
    // A 0.03 mm cavity inside a 5 mm plate: merging that layer would fill it in, which
    // verification must refuse (the cavity walls would be ~2.5 mm from the filled solid).
    let m = slab_mesh(&[
        (0.0, 2.485, vec![poly(rect(0.0, 0.0, 50.0, 30.0), vec![])]),
        (2.485, 2.515, vec![poly(rect(0.0, 0.0, 50.0, 30.0), vec![rect(10.0, 8.0, 40.0, 22.0)])]),
        (2.515, 5.0, vec![poly(rect(0.0, 0.0, 50.0, 30.0), vec![])]),
    ]);
    assert!(m.is_closed());
    let p = tmp("cavity.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert_eq!(plan.layers.len(), 3);
    assert!(plan.deviation.unwrap().max_mm < 0.05);
}

#[test]
fn sphere_falls_back_to_mesh_conversion() {
    let p = tmp("sphere.stl");
    write_stl(&uv_sphere(20.0, 40, 80), &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "mesh_convert");
    assert!(plan.layers.is_empty());
    assert!(plan.notes.iter().any(|n| n.contains("falling back") || n.contains("no 2.5D")));
}

#[test]
fn open_mesh_is_not_rebuilt() {
    let mut m = plate();
    m.f.truncate(m.f.len() - 2);
    let p = tmp("open.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "mesh_convert");
    assert!(plan.notes[0].contains("watertight"));
}

// ---------------------------------------------------------------- edge treatments
fn treatments_of<'a>(p: &'a Plan, kind: &str) -> Vec<&'a serde_json::Value> {
    p.treatments.iter().filter(|t| t["kind"] == kind).collect()
}

#[test]
fn exact_parts_get_no_treatments() {
    let p = tmp("exact.stl");
    write_stl(&plate(), &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert!(plan.treatments.is_empty());
}

#[test]
fn chamfered_plate_edge_is_recovered() {
    let (w, d, h, c) = (60.0, 40.0, 10.0, 1.5);
    let m = loft(&[
        (0.0, vec![rect(0.0, 0.0, w, d)]),
        (h - c, vec![rect(0.0, 0.0, w, d)]),
        (h, vec![inset_rect(0.0, 0.0, w, d, c)]),
    ]);
    let p = tmp("chamfer.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert!(plan.deviation_sharp.as_ref().unwrap().max_mm > 0.8, "a sharp box is ~1.06 mm off");
    assert!(plan.deviation.as_ref().unwrap().max_mm + plan.simplify_error_mm <= 0.05);
    let ch = treatments_of(&plan, "chamfer");
    assert_eq!(ch.len(), 1, "{:?}", plan.treatments);
    assert!((ch[0]["size_xy"].as_f64().unwrap() - c).abs() < 0.05 && (ch[0]["size_z"].as_f64().unwrap() - c).abs() < 0.05);
    assert_eq!(ch[0]["family"], "convex_top");
    assert_eq!(ch[0]["edges"].as_array().unwrap().len(), 4);
}

#[test]
fn filleted_plate_edge_is_recovered_as_a_fillet() {
    let (w, d, h, r) = (60.0, 40.0, 10.0, 2.0);
    let mut secs = vec![(0.0, vec![rect(0.0, 0.0, w, d)]), (h - r, vec![rect(0.0, 0.0, w, d)])];
    for k in 1..=24 {
        let th = std::f64::consts::FRAC_PI_2 * k as f64 / 24.0;
        secs.push((h - r + r * th.sin(), vec![inset_rect(0.0, 0.0, w, d, r * (1.0 - th.cos()))]));
    }
    let p = tmp("fillet.stl");
    write_stl(&loft(&secs), &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    let f = treatments_of(&plan, "fillet");
    assert_eq!(f.len(), 1, "{:?}", plan.treatments);
    assert!((f[0]["size_xy"].as_f64().unwrap() - r).abs() < 0.1);
    assert!(plan.deviation.as_ref().unwrap().max_mm + plan.simplify_error_mm <= 0.05);
}

#[test]
fn hole_countersinks_and_outline_chamfer_are_fitted_separately() {
    // 8 mm plate, 1.5 mm outline chamfer on top, two 45-degree countersinks (bore r 2.25 -> r 4.0)
    let (w, d, c) = (60.0, 40.0, 1.5);
    let holes = |r: f64| vec![circle(15.0, 20.0, r, 64), circle(45.0, 20.0, r, 64)];
    let sec = |inset: f64, r: f64| { let mut v = vec![inset_rect(0.0, 0.0, w, d, inset)]; v.extend(holes(r)); v };
    let m = loft(&[(0.0, sec(0.0, 2.25)), (6.25, sec(0.0, 2.25)), (6.5, sec(0.0, 2.5)), (8.0, sec(c, 4.0))]);
    assert!(m.is_closed());
    let p = tmp("countersink.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    let ch = treatments_of(&plan, "chamfer");
    let near = |t: &&serde_json::Value, v: f64| (t["size_xy"].as_f64().unwrap() - v).abs() < 0.1;
    let outline = ch.iter().find(|t| near(t, 1.5)).expect("outline chamfer");
    let sinks = ch.iter().find(|t| near(t, 1.75)).expect("countersink chamfer");
    assert_eq!(outline["edges"].as_array().unwrap().len(), 4);
    // the two identical countersinks share one treatment with two circle edges
    let e = sinks["edges"].as_array().unwrap();
    assert_eq!(e.len(), 2);
    assert!(e.iter().all(|x| x["type"] == "circle"));
    assert!(plan.deviation.as_ref().unwrap().max_mm + plan.simplify_error_mm <= 0.05);
}

// ---------------------------------------------------------------- exact lines and arcs
use stl2cad::fit::{segment_ring, Piece};
use stl2cad::geom::*;

/// Rounded rectangle, counter-clockwise, `n` points per quarter circle.
fn rounded_rect(w: f64, d: f64, r: f64, n: usize) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    let corners = [(w - r, r, -90.0), (w - r, d - r, 0.0), (r, d - r, 90.0), (r, r, 180.0)];
    for (cx, cy, start) in corners {
        for k in 0..=n {
            let a = (start + 90.0 * k as f64 / n as f64).to_radians();
            out.push([cx + r * a.cos(), cy + r * a.sin()]);
        }
    }
    out
}

fn counts(pieces: &[Piece]) -> (usize, usize) {
    (pieces.iter().filter(|p| matches!(p, Piece::Line { .. })).count(),
     pieces.iter().filter(|p| matches!(p, Piece::Arc { .. })).count())
}

#[test]
fn rounded_rectangle_is_four_lines_and_four_arcs() {
    let ring = rounded_rect(60.0, 40.0, 5.0, 12);
    let (pieces, err) = segment_ring(&ring, 0.005);
    assert_eq!(counts(&pieces), (4, 4), "{pieces:?}");
    assert!(err < 0.005);
    for p in &pieces {
        if let Piece::Arc { a, m, b } = p {
            let (_, r) = stl2cad::fit::circumcircle(*a, *m, *b).unwrap();
            assert!((r - 5.0).abs() < 0.01, "radius {r}");
        }
    }
}

#[test]
fn stadium_slot_is_two_lines_and_two_half_circle_arcs() {
    let r = 4.0;
    let mut ring = Vec::new();
    for k in 0..=24 { let a = (-90.0 + 180.0 * k as f64 / 24.0).to_radians(); ring.push([30.0 + r * a.cos(), r + r * a.sin()]); }
    for k in 0..=24 { let a = (90.0 + 180.0 * k as f64 / 24.0).to_radians(); ring.push([10.0 + r * a.cos(), r + r * a.sin()]); }
    let (pieces, err) = segment_ring(&ring, 0.005);
    assert_eq!(counts(&pieces), (2, 2), "{pieces:?}");
    assert!(err < 0.005);
}

#[test]
fn polygon_stays_lines_only() {
    let l = vec![[0.0, 0.0], [50.0, 0.0], [50.0, 10.0], [10.0, 10.0], [10.0, 40.0], [0.0, 40.0]];
    let (pieces, _) = segment_ring(&l, 0.005);
    assert_eq!(counts(&pieces), (6, 0));
}

#[test]
fn rounded_plate_outline_becomes_a_path_with_arcs() {
    // 2.5 degree steps: chord sag 0.0012 mm, inside the arc budget (10 % of the tolerance)
    let m = slab_mesh(&[(0.0, 6.0, vec![poly(rounded_rect(60.0, 40.0, 5.0, 36), vec![])])]);
    let p = tmp("rounded.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    match &plan.layers[0].regions[0].outer {
        Loop::Path { pieces } => assert_eq!(pieces.len(), 8),
        other => panic!("expected a path of 8 pieces, got {other:?}"),
    }
    assert!(plan.deviation.as_ref().unwrap().max_mm + plan.simplify_error_mm <= 0.05);
}

#[test]
fn coarse_tessellation_stays_a_polyline_within_budget() {
    // 7.5 degree steps: chord sag 0.011 mm exceeds the 0.005 mm arc budget, so no arcs are claimed
    let m = slab_mesh(&[(0.0, 6.0, vec![poly(rounded_rect(60.0, 40.0, 5.0, 12), vec![])])]);
    let p = tmp("rounded_coarse.stl");
    write_stl(&m, &p);
    let plan = analyze(&p, 0.05).unwrap();
    assert_eq!(plan.strategy, "extruded_layers", "{:?}", plan.notes);
    assert!(matches!(plan.layers[0].regions[0].outer, Loop::Polyline { .. }));
}
