import json

import numpy as np
import pytest
import shapely.geometry as sg
import trimesh

from stl2cad.classifier import RuleTransport
from stl2cad.convert import analyze
from stl2cad.worker import main as worker_main


def holes_plate(size=(60, 40, 10), holes=((15, 20, 3.4), (45, 20, 5.5))):
    m = trimesh.creation.box(size)
    for x, y, d in holes:
        c = trimesh.creation.cylinder(radius=d / 2, height=size[2] * 3, sections=64)
        c.apply_translation([x - size[0] / 2, y - size[1] / 2, 0])
        m = m.difference(c, engine="manifold")
    return m


def save(m, tmp_path, name="p.stl"):
    p = tmp_path / name
    m.export(p)
    return str(p)


def holes_of(plan):
    return sorted(round(2 * h["r"], 2) for L in plan.layers for r in L["regions"]
                  for h in r["holes"] if h["type"] == "circle")


def test_plate_with_holes_single_layer_no_llm(tmp_path):
    plan = analyze(save(holes_plate(), tmp_path))
    assert plan.strategy == "extruded_layers" and plan.llm_calls == 0
    assert len(plan.layers) == 1
    L = plan.layers[0]
    assert abs((L["z1"] - L["z0"]) - 10) < 1e-3
    assert holes_of(plan) == [3.4, 5.5]
    assert plan.deviation["max_mm"] < 0.01


def test_tilted_plate_frame_is_orthonormal_and_still_exact(tmp_path):
    m = holes_plate()
    rot = trimesh.transformations.random_rotation_matrix(np.random.default_rng(3).random(3))
    m.apply_transform(rot)
    plan = analyze(save(m, tmp_path))
    assert plan.strategy == "extruded_layers" and len(plan.layers) == 1
    R = np.array(plan.frame)[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6) and np.linalg.det(R) > 0
    assert holes_of(plan) == [3.4, 5.5]


def test_stepped_part_with_pocket_has_layers(tmp_path):
    base = trimesh.creation.box((60, 40, 10))
    boss = trimesh.creation.box((30, 20, 5)); boss.apply_translation([0, 0, 7.5])
    pocket = trimesh.creation.box((20, 10, 4)); pocket.apply_translation([0, 0, -4])
    m = base.union(boss, engine="manifold").difference(pocket, engine="manifold")
    plan = analyze(save(m, tmp_path))
    assert plan.strategy == "extruded_layers"
    assert len(plan.layers) == 3  # pocketed floor, solid middle, boss
    assert plan.deviation["max_mm"] < 0.01 and plan.deviation["volume_diff_pct"] < 0.1


def test_l_bracket_outline_is_polyline(tmp_path):
    poly = sg.Polygon([(0, 0), (50, 0), (50, 10), (10, 10), (10, 40), (0, 40)])
    m = trimesh.creation.extrude_polygon(poly, 8)
    plan = analyze(save(m, tmp_path))
    assert plan.strategy == "extruded_layers" and len(plan.layers) == 1
    outer = plan.layers[0]["regions"][0]["outer"]
    assert outer["type"] == "polyline" and len(outer["pts"]) == 6


def test_blob_falls_back_to_mesh_conversion(tmp_path):
    plan = analyze(save(trimesh.creation.icosphere(subdivisions=3, radius=20), tmp_path))
    assert plan.strategy == "mesh_convert" and not plan.layers
    assert any("falling back" in n for n in plan.notes)


def test_snap_only_with_transport_and_within_tolerance(tmp_path):
    p = save(holes_plate(holes=((15, 20, 3.37), (45, 20, 5.5))), tmp_path)
    plan = analyze(p, snap=True, transport=RuleTransport())
    assert plan.llm_calls == 1
    assert {x["to_mm"] for x in plan.snapped} == {3.4, 5.5}
    assert 3.4 in holes_of(plan)
    # no transport: holes stay as measured and the reason is recorded
    plan = analyze(p, snap=True, transport=None)
    assert plan.llm_calls == 0 and 3.37 in holes_of(plan) and plan.review


def test_worker_writes_plan(tmp_path, capsys):
    p = save(holes_plate(), tmp_path)
    out = tmp_path / "plan.json"
    assert worker_main([p, "--out", str(out)]) == 0
    data = json.loads(out.read_text())
    assert data["version"] == 1 and data["units"] == "mm" and data["strategy"] == "extruded_layers"
    assert "apikey" not in out.read_text().lower()
