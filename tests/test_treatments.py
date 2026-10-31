import numpy as np
import pytest
import trimesh

from stl2cad.convert import analyze
from stl2cad.layers import build_layers, merge_slivers
from test_convert import save


def rect(w, d, z):
    return [(sx * w / 2, sy * d / 2, z) for sx in (-1, 1) for sy in (-1, 1)]


def chamfered_plate(c=1.5, w=60, d=40, h=10):
    pts = rect(w, d, 0) + rect(w, d, h - c) + rect(w - 2 * c, d - 2 * c, h)
    return trimesh.convex.convex_hull(np.array(pts))


def post_with_fillet_and_chamfer(R=8.0, H=30.0, f=2.0, ch=1.0):
    a = np.linspace(0, np.pi / 2, 24)
    prof = [(0, 0)] + [(R + f - f * np.sin(t), f - f * np.cos(t)) for t in a] + [(R, H - ch), (R - ch, H), (0, H)]
    post = trimesh.creation.revolve(np.array(prof), angle=2 * np.pi, sections=128)
    plate = trimesh.creation.box((60, 60, 6)); plate.apply_translation([0, 0, -3])
    return plate.union(post, engine="manifold")


def test_chamfered_plate_is_recovered(tmp_path):
    plan = analyze(save(chamfered_plate(), tmp_path), tol=0.05)
    assert plan.strategy == "extruded_layers"
    assert plan.deviation_sharp["max_mm"] > 0.8          # sharp box alone is ~1.06 mm off
    assert plan.deviation["max_mm"] + plan.simplify_error_mm <= 0.05
    ch = [t for t in plan.treatments if t["kind"] == "chamfer"]
    assert ch and abs(ch[0]["size_xy"] - 1.5) < 0.1 and abs(ch[0]["size_z"] - 1.5) < 0.1
    assert ch[0]["family"] == "convex_top" and len(ch[0]["edges"]) == 4   # four outline segments


def test_post_base_fillet_and_top_chamfer(tmp_path):
    plan = analyze(save(post_with_fillet_and_chamfer(), tmp_path), tol=0.05)
    assert plan.strategy == "extruded_layers", plan.notes
    kinds = {(t["family"], t["kind"]) for t in plan.treatments}
    assert ("concave_above", "fillet") in kinds
    fil = next(t for t in plan.treatments if t["kind"] == "fillet")
    assert abs(fil["size_xy"] - 2.0) < 0.15
    assert any(t["edges"] and t["edges"][0]["type"] == "circle" for t in plan.treatments)
    assert plan.deviation["max_mm"] + plan.simplify_error_mm <= 0.05


def test_treatments_never_added_to_already_exact_parts(tmp_path):
    plan = analyze(save(trimesh.creation.box((40, 30, 8)), tmp_path))
    assert plan.strategy == "extruded_layers" and plan.treatments == []


def test_sliver_layer_with_matching_outline_is_merged(tmp_path):
    base = trimesh.creation.box((40, 40, 5))
    flange = trimesh.creation.box((40.02, 40.02, 0.02)); flange.apply_translation([0, 0, 0.0])
    m = base.union(flange, engine="manifold")
    plan = analyze(save(m, tmp_path), tol=0.05)
    assert plan.strategy == "extruded_layers" and len(plan.layers) == 1


def test_thin_foil_is_merged_but_thin_slit_is_not(tmp_path):
    # A 0.02 mm foil is only 0.02 mm off in surface distance, so merging it is fine.
    base = trimesh.creation.box((50, 30, 5))
    foil = trimesh.creation.box((49, 29, 0.02)); foil.apply_translation([0, 0, 2.51])
    plan = analyze(save(base.union(foil, engine="manifold"), tmp_path), tol=0.05)
    assert plan.strategy == "extruded_layers" and len(plan.layers) == 1
    # A 0.03 mm slit open at one edge is a deep cavity: merging would fill it in.
    slit = trimesh.creation.box((35, 30, 0.03)); slit.apply_translation([10, 0, 0])
    plan = analyze(save(base.difference(slit, engine="manifold"), tmp_path, "slit.stl"), tol=0.05)
    assert plan.strategy == "extruded_layers" and len(plan.layers) == 3
    assert plan.deviation["max_mm"] < 0.05


def test_merge_slivers_unit():
    m = trimesh.creation.box((40, 40, 5))
    layers, _, _ = build_layers(m.copy().apply_translation([20, 20, 2.5]) or m, 0.01)
    assert len(layers) == 1 and merge_slivers(layers, 0.05)[0].z1 == layers[0].z1


def test_hole_countersinks_and_outline_chamfer_are_fitted_separately(tmp_path):
    plate = chamfered_plate(c=1.5, w=60, d=40, h=8)
    cut = None
    for x in (-15, 15):
        cyl = trimesh.creation.cylinder(radius=2.25, height=30, sections=64); cyl.apply_translation([x, 0, 4])
        prof = np.array([(0, 6.25), (2.25, 6.25), (4.0, 8.0), (4.0, 8.5), (0, 8.5)])  # 45 deg countersink
        cs = trimesh.creation.revolve(prof, angle=2 * np.pi, sections=64); cs.apply_translation([x, 0, 0])
        plate = plate.difference(cyl, engine="manifold").difference(cs, engine="manifold")
    plan = analyze(save(plate, tmp_path), tol=0.05)
    assert plan.strategy == "extruded_layers", plan.notes
    sizes = sorted(round(t["size_xy"], 1) for t in plan.treatments if t["kind"] == "chamfer")
    assert any(abs(s - 1.5) < 0.15 for s in sizes) and any(abs(s - 1.75) < 0.15 for s in sizes), sizes
    # the two identical countersinks share one treatment (one Fusion feature)
    cs_t = [t for t in plan.treatments if abs(t["size_xy"] - 1.75) < 0.15]
    assert len(cs_t) == 1 and len(cs_t[0]["edges"]) == 2
    assert plan.deviation["max_mm"] + plan.simplify_error_mm <= 0.05
