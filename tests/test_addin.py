import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest
import trimesh

ADDIN = Path(__file__).resolve().parents[1] / "fusion_addin" / "STL2CAD"
sys.path.insert(0, str(ADDIN))
import plan_build  # noqa: E402
import stl_io  # noqa: E402

from stl2cad.convert import analyze  # noqa: E402
from test_convert import holes_plate, save  # noqa: E402


def test_stl_writer_roundtrip_with_cm_to_mm_scale(tmp_path):
    m = trimesh.creation.box((6, 4, 1))  # "cm"
    n = stl_io.write_binary_stl(str(tmp_path / "a.stl"), m.vertices.ravel().tolist(),
                                m.faces.ravel().tolist(), scale=10.0)
    back = trimesh.load(str(tmp_path / "a.stl"))
    assert n == 12 and np.allclose(sorted(back.extents), [10, 40, 60], atol=1e-3)
    assert abs(back.volume - 24000) < 1


def test_match_profiles_picks_ring_not_hole_disc():
    expected = [(10.0, 1.0, 1.0)]
    actual = [(2.0, 1.0, 1.0), (10.0, 1.0, 1.0)]  # hole disc first, ring second
    assert plan_build.match_profiles(expected, actual) == [1]
    with pytest.raises(LookupError):
        plan_build.match_profiles([(10.0, 1.0, 1.0)], [(2.0, 1.0, 1.0)])
    # a profile is never used for two regions
    with pytest.raises(LookupError):
        plan_build.match_profiles([(10, 1, 1), (10, 1, 1)], [(10, 1, 1)])


def _fake_fusion(plan, calls):
    """Fusion stand-in. Each sketch exposes the material profiles plus a decoy per hole."""
    core, fusion, root = MagicMock(), MagicMock(), MagicMock()
    sketches = iter(range(len(plan["layers"])))

    def add_sketch(_plane):
        L = plan["layers"][next(sketches)]
        profs = []
        for r in L["regions"]:
            profs.append(("material", r["area"] / 100, r["centroid"][0] / 10, r["centroid"][1] / 10))
            for h in r["holes"]:  # the disc inside each hole is also a profile
                profs.append(("decoy", 0.01, r["centroid"][0] / 10 + 5, r["centroid"][1] / 10))
        sk = MagicMock()
        items = []
        for kind, a, x, y in profs:
            p = MagicMock(); p.kind = kind
            p.areaProperties.return_value = MagicMock(area=a, centroid=MagicMock(x=x, y=y))
            items.append(p)
        sk.profiles.count = len(items)
        sk.profiles.item.side_effect = lambda k: items[k]
        return sk

    root.sketches.add.side_effect = add_sketch
    collections = []

    def new_collection():
        c = MagicMock(); c.items = []
        c.add.side_effect = lambda o: c.items.append(o)
        collections.append(c)
        return c

    core.ObjectCollection.create.side_effect = new_collection
    ext = root.features.extrudeFeatures
    ext.createInput.side_effect = lambda coll, op: calls.append((list(coll.items), op)) or MagicMock()
    ext.add.return_value.bodies.count = 1
    return core, fusion, root


def test_builder_extrudes_only_material_profiles_per_layer(tmp_path):
    plan = json.loads(analyze(save(holes_plate(), tmp_path)).to_json())
    assert plan["strategy"] == "extruded_layers"
    calls = []
    core, fusion, root = _fake_fusion(plan, calls)
    bodies, notes = plan_build.build_plan(core, fusion, root, plan)
    assert len(calls) == len(plan["layers"]) == 1
    items, op = calls[0]
    assert [p.kind for p in items] == ["material"]           # decoy hole discs excluded
    assert op is fusion.FeatureOperations.NewBodyFeatureOperation
    assert len(bodies) == 1
    # identity-ish frame for an axis-aligned plate may still translate; move only if non-identity
    moved = root.features.moveFeatures.add.called
    assert moved == (not plan_build._is_identity(plan["frame"]))


def test_builder_joins_later_layers():
    base = trimesh.creation.box((60, 40, 10))
    boss = trimesh.creation.box((30, 20, 5)); boss.apply_translation([0, 0, 7.5])
    import tempfile
    d = Path(tempfile.mkdtemp())
    plan = json.loads(analyze(save(base.union(boss, engine="manifold"), d)).to_json())
    assert len(plan["layers"]) == 2
    calls = []
    core, fusion, root = _fake_fusion(plan, calls)
    plan_build.build_plan(core, fusion, root, plan)
    ops = [op for _, op in calls]
    assert ops == [fusion.FeatureOperations.NewBodyFeatureOperation,
                   fusion.FeatureOperations.JoinFeatureOperation]


def test_installer_copies_addin(tmp_path):
    import subprocess
    dest = tmp_path / "AddIns" / "STL2CAD"
    root = Path(__file__).resolve().parents[1]
    subprocess.check_call([sys.executable, str(root / "scripts" / "install_addin.py"),
                           "--dest", str(dest), "--no-venv"])
    assert (dest / "STL2CAD.py").exists() and (dest / "STL2CAD.manifest").exists()
    assert json.loads((dest / "STL2CAD.manifest").read_text())["type"] == "addin"


def test_match_edge_circle_and_segment_either_direction():
    c = {"type": "circle", "cx": 10, "cy": 5, "r": 8, "z": 6}
    assert plan_build.match_edge(c, {"type": "circle", "c": (1.0, 0.5, 0.6), "r": 0.8})
    assert not plan_build.match_edge(c, {"type": "circle", "c": (1.0, 0.5, 0.6), "r": 0.9})
    assert not plan_build.match_edge(c, {"type": "circle", "c": (1.0, 0.5, 0.0), "r": 0.8})  # wrong z
    s = {"type": "segment", "p0": [0, 0], "p1": [30, 0], "z": 10}
    fwd = {"type": "segment", "p0": (0, 0, 1.0), "p1": (3.0, 0, 1.0)}
    rev = {"type": "segment", "p0": (3.0, 0, 1.0), "p1": (0, 0, 1.0)}
    assert plan_build.match_edge(s, fwd) and plan_build.match_edge(s, rev)
    assert not plan_build.match_edge(s, {"type": "circle", "c": (0, 0, 1.0), "r": 1})


def test_apply_treatments_picks_edges_and_makes_fillet_and_chamfer(monkeypatch):
    plan = {"treatments": [
        {"kind": "fillet", "z": 6.0, "size_xy": 2.0, "size_z": 2.0,
         "edges": [{"type": "circle", "cx": 0, "cy": 0, "r": 8, "z": 6.0}]},
        {"kind": "chamfer", "z": 30.0, "size_xy": 1.0, "size_z": 1.0,
         "edges": [{"type": "circle", "cx": 0, "cy": 0, "r": 7, "z": 30.0},
                   {"type": "circle", "cx": 99, "cy": 0, "r": 7, "z": 30.0}]}]}  # second edge absent
    edges = {"base": {"type": "circle", "c": (0, 0, 0.6), "r": 0.8},
             "top": {"type": "circle", "c": (0, 0, 3.0), "r": 0.7},
             "other": {"type": "circle", "c": (0, 0, 1.5), "r": 0.4}}
    monkeypatch.setattr(plan_build, "_edge_info", lambda core, e: edges[e])
    core, fusion, root = MagicMock(), MagicMock(), MagicMock()
    body = MagicMock(); body.edges = list(edges)

    class Coll:
        def __init__(self): self.items = []
        def add(self, o): self.items.append(o)
        @property
        def count(self): return len(self.items)

    made = []
    core.ObjectCollection.create.side_effect = lambda: made.append(Coll()) or made[-1]
    notes = plan_build.apply_treatments(core, fusion, root, [body], plan)
    assert [c.items for c in made] == [["base"], ["top"]]  # right edge per treatment, absent one skipped
    assert root.features.filletFeatures.add.call_count == 1
    assert root.features.chamferFeatures.add.call_count == 1
    assert any("not found" in n for n in notes)  # the absent edge is reported, not silently dropped
