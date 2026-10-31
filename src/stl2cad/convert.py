"""STL -> build plan. Deterministic first; JEV only for the optional snap step."""

from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np

from . import mesh as meshmod
from .classifier import JevError, Transport
from .decisions import decide_hole
from .features import Features, Hole
from shapely.geometry import Point

from .layers import Frame, Layer, build_layers, candidate_axes, make_frame, merge_slivers
from .treatments import Treatment, detect_treatments, layer_slabs
from .questions import build_state, hole_questions
from .segment import segment
from .layers import _union
from .verify import deviation, surface_from_slabs

PLAN_VERSION = 1
_T0 = time.time()


def _trace(msg: str) -> None:
    if os.environ.get("STL2CAD_TRACE"):
        print(f"[stl2cad {time.time() - _T0:6.1f}s] {msg}", file=sys.stderr, flush=True)


@dataclass
class Plan:
    strategy: str  # "extruded_layers" | "mesh_convert"
    tolerance_mm: float
    frame: list[list[float]] | None = None
    layers: list[dict] = field(default_factory=list)
    deviation: dict | None = None
    deviation_sharp: dict | None = None  # layers only, before edge treatments
    treatments: list[dict] = field(default_factory=list)
    circle_fit_error_mm: float = 0.0
    simplify_error_mm: float = 0.0
    review: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    snapped: list[dict] = field(default_factory=list)
    llm_calls: int = 0
    stats: dict = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps({"version": PLAN_VERSION, "units": "mm", **self.__dict__}, indent=1)


def _layer_dict(L: Layer) -> dict:
    return {"z0": round(L.z0, 5), "z1": round(L.z1, 5),
            "regions": [{"outer": r.outer, "holes": r.holes, "area": round(r.area, 4),
                         "centroid": [round(c, 5) for c in r.centroid]} for r in L.regions]}


def _verify(am, layers: list[Layer], ts: list[Treatment], tol: float):
    """Deviation of the layered model (with edge treatments, if any) from the mesh."""
    slabs = []
    for k, L in enumerate(layers):
        slabs += layer_slabs(L, k, ts, step=0.25 * tol) if ts else [(L.z0, L.z1, _union(L))]
    mesh, vol = surface_from_slabs(slabs)
    return deviation(am, mesh, rebuilt_volume=vol)


def _edge_descriptors(t: Treatment, plan_layer: dict, eps: float) -> list[dict]:
    """Outline edges of the layer that lie on the treatment's curve (for Fusion edge picking)."""
    thr = max(0.02, 2 * eps)
    out = []
    for r in plan_layer["regions"]:
        for loop in [r["outer"]] + r["holes"]:
            if loop["type"] == "circle":
                c, rad = (loop["cx"], loop["cy"]), loop["r"]
                pts = [Point(c[0] + rad * np.cos(a), c[1] + rad * np.sin(a)) for a in np.linspace(0, 6.2832, 16)]
                if all(t.curve.distance(p) < thr for p in pts):
                    out.append({"type": "circle", "cx": c[0], "cy": c[1], "r": rad, "z": round(t.z, 5)})
            else:
                P = loop["pts"]
                for p0, p1 in zip(P, P[1:] + P[:1]):
                    mid = Point((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
                    if t.curve.distance(mid) < thr and t.curve.distance(Point(*p0)) < thr:
                        out.append({"type": "segment", "p0": p0, "p1": p1, "z": round(t.z, 5)})
    return out


def _sharp(raw, axis, tol: float):
    """Layers for one axis (no edge treatments), verified against the mesh."""
    frame: Frame = make_frame(raw, axis)
    am, eps = frame.aligned_mesh, 0.2 * tol
    layers0, fit_err, simp_err = build_layers(am, eps)
    _trace(f"axis {np.round(axis, 3).tolist()}: {len(layers0)} layers")
    merged = merge_slivers(layers0, tol)
    variants = [merged] + ([layers0] if len(merged) != len(layers0) else [])
    best = None
    for layers in variants:
        dev = _verify(am, layers, [], tol)
        _trace(f"  {len(layers)} layers: max deviation {dev.max_mm:.4f} mm")
        res = {"frame": frame, "layers": layers, "fit_err": fit_err, "eps": eps, "simp": simp_err,
               "dev": dev, "dev_sharp": dev, "treatments": []}
        # Prefer the merged (cleaner) variant unless the unmerged one is much better: sharp
        # deviations that differ by a few percent are sampling noise, while a merge that
        # fills in a real cavity is off by orders of magnitude.
        if best is None or dev.max_mm < 0.5 * best["dev"].max_mm:
            best = res
        if dev.max_mm + simp_err <= tol:
            break
    return best


def _with_treatments(res: dict, tol: float, max_edge: float) -> dict:
    """Try to close the remaining gap with chamfers/fillets on the layer edges."""
    am = res["frame"].aligned_mesh
    ts = detect_treatments(am, res["layers"], tol, max_edge)
    _trace(f"  {len(ts)} edge treatments detected")
    if ts:
        dev_t = _verify(am, res["layers"], ts, tol)
        _trace(f"  with treatments: max deviation {dev_t.max_mm:.4f} mm")
        if dev_t.max_mm < res["dev"].max_mm:
            return {**res, "dev": dev_t, "treatments": ts}
    return res


def analyze(stl_path: str, tol: float = 0.05, snap: bool = False,
            transport: Transport | None = None, snap_tol: float = 0.1,
            max_edge: float = 6.0) -> Plan:
    t0 = time.time()
    raw = meshmod.load_clean(stl_path)
    plan = Plan("mesh_convert", tol)
    plan.stats = {"faces": int(len(raw.faces)), "watertight": bool(raw.is_watertight),
                  "extents_mm": [round(float(e), 3) for e in raw.extents],
                  "volume_mm3": round(float(abs(raw.volume)), 1)}
    if not raw.is_watertight:
        plan.notes.append("mesh is not watertight; 2.5D rebuild skipped")
        plan.stats["seconds"] = round(time.time() - t0, 1)
        return plan

    seg = segment(raw)
    plan.stats["unexplained_area_pct"] = round(100 * seg.unexplained_area_fraction, 1)
    best = first = None
    tried = []
    for axis in candidate_axes(seg):  # ranked by flat area: the first is the main extrusion axis
        try:
            res = _sharp(raw, axis, tol)
        except ValueError as e:
            tried.append(f"axis {np.round(axis, 3).tolist()}: {e}")
            continue
        dev = res["dev"]
        tried.append(f"axis {np.round(axis, 3).tolist()}: {len(res['layers'])} layers, "
                     f"max deviation {dev.max_mm:.4f} mm")
        first = first or res
        if best is None or dev.max_mm < best["dev"].max_mm:
            best = res
        if dev.max_mm + res["simp"] <= tol and res["fit_err"] <= tol:
            break
    # Edge treatments are searched on one axis only, and only if needed. Sharp deviation
    # does not predict recoverability (a chamfered plate is ~equally off along every axis),
    # so keep the main axis unless another is at least twice as accurate.
    if best is not None and best["dev"].max_mm + best["simp"] > tol:
        if first is not None and best is not first and best["dev"].max_mm > 0.5 * first["dev"].max_mm:
            best = first
        best = _with_treatments(best, tol, max_edge)
        tried.append(f"edge treatments: {len(best['treatments'])} found, "
                     f"max deviation now {best['dev'].max_mm:.4f} mm")

    plan.notes += tried
    if best is not None:
        dev, eps = best["dev"], best["eps"]
        bound = dev.max_mm + best["simp"]
        plan.deviation = dev.to_dict()
        plan.deviation_sharp = best["dev_sharp"].to_dict()
        plan.circle_fit_error_mm = round(best["fit_err"], 5)
        plan.simplify_error_mm = round(best["simp"], 5)
        if bound <= tol and best["fit_err"] <= tol:
            plan.strategy = "extruded_layers"
            plan.frame = best["frame"].matrix.tolist()
            plan.layers = [_layer_dict(L) for L in best["layers"]]
            for t in best["treatments"]:
                edges = _edge_descriptors(t, plan.layers[t.layer], eps)
                plan.treatments.append({
                    "kind": t.kind, "family": t.family, "z": round(t.z, 5),
                    "size_xy": round(t.size_xy, 4), "size_z": round(t.size_z, 4),
                    "fit_rms_mm": round(t.rms, 5), "layer": t.layer, "edges": edges})
                if not edges:
                    plan.review.append(f"{t.kind} at z={t.z:.2f} could not be tied to outline edges; skipped")
            if snap:
                _snap_holes(plan, best["frame"], transport, snap_tol)
        else:
            plan.notes.append(
                f"best 2.5D rebuild is off by up to {bound:.3f} mm (tolerance {tol} mm); "
                "falling back to mesh conversion")
    else:
        plan.notes.append("no 2.5D interpretation found; falling back to mesh conversion")
    plan.stats["seconds"] = round(time.time() - t0, 1)
    return plan


def _snap_holes(plan: Plan, frame: Frame, transport: Transport | None, snap_tol: float) -> None:
    """Optional design-intent cleanup: snap circular holes to standard screw sizes.

    JEV judges meaning (is it a screw hole, clearance or tap drill); code does the
    arithmetic and applies a change only if it moves the radius by <= snap_tol.
    """
    if transport is None:
        plan.review.append("snap requested but no JEV transport available; holes left as measured")
        return
    groups: dict[tuple, list[dict]] = {}
    for L in plan.layers:
        for r in L["regions"]:
            for h in r["holes"]:
                if h["type"] == "circle":
                    groups.setdefault((round(h["cx"], 2), round(h["cy"], 2), round(h["r"], 3)), []).append(h)
    if not groups:
        return
    holes = [Hole(f"hole_{i + 1}", [k[0], k[1], 0.0], [0, 0, 1], round(2 * k[2], 4), 0.0)
             for i, k in enumerate(groups)]
    feats = Features(extents=[round(float(e), 4) for e in frame.aligned_mesh.extents],
                     volume=float(abs(frame.aligned_mesh.volume)), holes=holes)
    qs = [q for h in holes for q in hole_questions(h.id)]
    try:
        answers = transport.ask(build_state(feats), qs)
    except JevError as e:
        plan.review.append(f"snap skipped: JEV call failed ({e})")  # never treated as an answer
        return
    plan.llm_calls += 1
    for h, loops in zip(holes, groups.values()):
        d = decide_hole(h, answers)
        if d.action == "review":
            plan.review.append(f"{h.id} (d={h.diameter} mm): {d.reason}")
        if d.standard and abs(d.diameter - h.diameter) / 2 <= snap_tol and d.diameter != h.diameter:
            for loop in loops:
                loop["r"] = round(d.diameter / 2, 5)
            plan.snapped.append({"from_mm": h.diameter, "to_mm": d.diameter, "standard": d.standard,
                                 "at": [loops[0]["cx"], loops[0]["cy"]]})
