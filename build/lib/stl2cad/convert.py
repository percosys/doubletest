"""STL -> build plan. Deterministic first; JEV only for the optional snap step."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field

import numpy as np

from . import mesh as meshmod
from .classifier import JevError, Transport
from .decisions import decide_hole
from .features import Features, Hole
from .layers import Frame, Layer, build_layers, candidate_axes, make_frame
from .questions import build_state, hole_questions
from .segment import segment
from .verify import Deviation, deviation, layers_to_mesh

PLAN_VERSION = 1


@dataclass
class Plan:
    strategy: str  # "extruded_layers" | "mesh_convert"
    tolerance_mm: float
    frame: list[list[float]] | None = None
    layers: list[dict] = field(default_factory=list)
    deviation: dict | None = None
    circle_fit_error_mm: float = 0.0
    simplify_eps_mm: float = 0.0
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


def _attempt(raw, axis, tol: float):
    frame: Frame = make_frame(raw, axis)
    eps = 0.2 * tol
    layers, fit_err = build_layers(frame.aligned_mesh, eps)
    dev = deviation(frame.aligned_mesh, layers_to_mesh(layers))
    return frame, layers, fit_err, eps, dev


def analyze(stl_path: str, tol: float = 0.05, snap: bool = False,
            transport: Transport | None = None, snap_tol: float = 0.1) -> Plan:
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
    best = None
    tried = []
    for axis in candidate_axes(seg):
        try:
            frame, layers, fit_err, eps, dev = _attempt(raw, axis, tol)
        except ValueError as e:
            tried.append(f"axis {np.round(axis, 3).tolist()}: {e}")
            continue
        bound = dev.max_mm + eps
        tried.append(f"axis {np.round(axis, 3).tolist()}: {len(layers)} layers, "
                     f"max deviation {dev.max_mm:.4f} mm")
        if best is None or bound < best[0]:
            best = (bound, frame, layers, fit_err, eps, dev)
        if bound <= tol and fit_err <= tol:
            break

    plan.notes += tried
    if best is not None:
        bound, frame, layers, fit_err, eps, dev = best
        plan.deviation = dev.to_dict()
        plan.circle_fit_error_mm = round(fit_err, 5)
        plan.simplify_eps_mm = round(eps, 5)
        if bound <= tol and fit_err <= tol:
            plan.strategy = "extruded_layers"
            plan.frame = frame.matrix.tolist()
            plan.layers = [_layer_dict(L) for L in layers]
            if snap:
                _snap_holes(plan, frame, transport, snap_tol)
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
