import json

import numpy as np
import pytest
import trimesh

from stl2cad import mesh as meshmod
from stl2cad.classifier import Answer, RuleTransport
from stl2cad.cli import run
from stl2cad.decisions import decide_hole, verdict
from stl2cad.emit_cadquery import emit
from stl2cad.features import Hole, extract
from stl2cad.segment import segment


def plate(holes=((15, 20, 3.4), (45, 20, 5.5)), size=(60, 40, 10)):
    m = trimesh.creation.box(size)
    for x, y, d in holes:
        c = trimesh.creation.cylinder(radius=d / 2, height=size[2] * 3, sections=64)
        c.apply_translation([x - size[0] / 2, y - size[1] / 2, 0])
        m = m.difference(c, engine="manifold")
    return m


@pytest.fixture
def stl(tmp_path):
    p = tmp_path / "plate.stl"
    plate().export(p)
    return str(p)


def test_segmentation_finds_planes_and_holes(stl):
    m, _ = meshmod.align(meshmod.load_clean(stl))
    seg = segment(m)
    assert len(seg.planes) >= 6
    holes = [c for c in seg.cylinders if c.concave and c.coverage_deg > 300]
    assert sorted(round(2 * c.radius, 1) for c in holes) == [3.4, 5.5]
    assert seg.unexplained_area_fraction < 0.01
    f = extract(m, seg)
    assert len(f.holes) == 2 and not f.bosses
    assert np.allclose(sorted(f.extents), [10, 40, 60], atol=0.05)


def test_boss_is_not_a_hole():
    m = trimesh.creation.box((40, 40, 5)).union(
        trimesh.creation.cylinder(radius=5, height=15, sections=64), engine="manifold")
    m, _ = meshmod.align(m)
    f = extract(m, segment(m))
    # The 15 mm cylinder protrudes 5 mm from each face of the 5 mm plate.
    assert [b.diameter for b in f.bosses] == [10.0, 10.0] and not f.holes


def test_verdict_bands_and_unsettled_row():
    mk = lambda p: Answer("noul", {"true": p, "false": 1 - p})
    assert verdict(mk(0.9)) == "yes" and verdict(mk(0.1)) == "no" and verdict(mk(0.5)) == "unsettled"


def test_hole_decisions_snap_and_flag():
    h = Hole("hole_1", [0, 0, 0], [0, 0, 1], 3.37, 10)
    noul = lambda p: Answer("noul", {"true": p, "false": 1 - p})
    purpose = Answer("choice", {"screw_clearance": 0.9, "tap_pilot": 0.1})
    ans = {"hole_1.intentional": noul(0.95), "hole_1.fastener": noul(0.9), "hole_1.purpose": purpose}
    d = decide_hole(h, ans)
    assert (d.action, d.diameter, d.standard) == ("include", 3.4, "M3 clearance")
    ans["hole_1.intentional"] = noul(0.5)
    assert decide_hole(h, ans).action == "review"
    ans["hole_1.intentional"] = noul(0.05)
    assert decide_hole(h, ans).action == "drop"


def test_end_to_end_offline(stl, tmp_path):
    res = run(stl, tmp_path / "out", RuleTransport())
    r = res["recipe"]
    assert r.part_type == "flat_plate_with_holes"
    assert sorted(h["diameter"] for h in r.holes) == [3.4, 5.5]
    assert (tmp_path / "out" / "request_state.json").exists()
    assert json.loads((tmp_path / "out" / "recipe.json").read_text())["units"] == "mm"
    code = emit(r)
    compile(code, "model.py", "exec")
    assert code.count("makeCylinder") == 2
