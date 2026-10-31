"""Deterministic 2.5D rebuild: cut the mesh into layers along one axis.

A part that is an extrusion, or a stack of extrusions (plates, brackets, steps,
pockets, bosses, through and blind holes), has a constant cross-section between
consecutive flat faces. Slicing at the middle of each interval recovers each
layer's outline exactly, including holes. No LLM and no feature guessing.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh
from shapely.geometry import LinearRing, Polygon

from .segment import Segmentation

MAX_LAYERS = 40


@dataclass
class Region:
    """One connected piece of material in a layer: outer loop minus holes."""

    outer: dict
    holes: list[dict]
    area: float
    centroid: list[float]


@dataclass
class Layer:
    z0: float
    z1: float
    regions: list[Region]
    # Raw section polygons (not serialised); used only to verify the rebuild.
    raw: list[Polygon] = field(default_factory=list, repr=False)


@dataclass
class Frame:
    """Aligned frame: ``world = rot_world_from_aligned @ aligned + shift``."""

    matrix: np.ndarray  # 4x4 world-from-aligned
    aligned_mesh: trimesh.Trimesh


# ---------------------------------------------------------------- axis & frame
def candidate_axes(seg: Segmentation, top: int = 3, cos_tol: float = 0.9998) -> list[np.ndarray]:
    """Plane normals ranked by total flat area (sign-insensitive)."""
    groups: list[list] = []  # [unit normal, area]
    for p in seg.planes:
        n = p.normal.copy()
        if n[np.argmax(np.abs(n))] < 0:
            n = -n
        for g in groups:
            if abs(float(g[0] @ n)) >= cos_tol:
                g[1] += p.area
                break
        else:
            groups.append([n, p.area])
    groups.sort(key=lambda g: -g[1])
    return [g[0] for g in groups[:top]]


def make_frame(mesh: trimesh.Trimesh, z_axis: np.ndarray) -> Frame:
    z = z_axis / np.linalg.norm(z_axis)
    v = mesh.vertices - mesh.vertices.mean(axis=0)
    flat = v - np.outer(v @ z, z)
    w, vec = np.linalg.eigh(flat.T @ flat)
    x = vec[:, -1] - (vec[:, -1] @ z) * z  # major in-plane direction
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])  # aligned = R @ world - mins
    aligned = mesh.vertices @ R.T
    mins = aligned.min(axis=0)
    M = np.eye(4)
    M[:3, :3] = R.T
    M[:3, 3] = R.T @ mins
    am = trimesh.Trimesh(aligned - mins, mesh.faces.copy(), process=False)
    return Frame(M, am)


# ---------------------------------------------------------------- loops
def _fit_circle(pts: np.ndarray):
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    sol, *_ = np.linalg.lstsq(A, x**2 + y**2, rcond=None)
    a, b, c = sol
    r2 = c + a * a + b * b
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    rms = float(np.sqrt(np.mean((np.hypot(x - a, y - b) - r) ** 2)))
    ang = np.sort(np.arctan2(y - b, x - a))
    gap = float(np.max(np.diff(np.r_[ang, ang[0] + 2 * np.pi])))
    return float(a), float(b), r, rms, gap


def ring_to_loop(coords, eps: float) -> tuple[dict, float]:
    """Closed ring -> circle if it is one, else a simplified polyline.

    Returns (loop, circle_fit_error); the error is 0 for polylines.
    """
    pts = np.asarray(coords)[:-1, :2]
    if len(pts) >= 16:
        fit = _fit_circle(pts)
        if fit is not None:
            cx, cy, r, rms, gap = fit
            if rms <= 0.002 * r + 1e-4 and gap <= np.radians(60):
                return {"type": "circle", "cx": round(cx, 5), "cy": round(cy, 5),
                        "r": round(r, 5)}, rms
    simp = np.asarray(LinearRing(pts).simplify(eps, preserve_topology=False).coords)[:-1]
    if len(simp) < 3:
        simp = pts
    return {"type": "polyline", "pts": [[round(float(a), 5), round(float(b), 5)] for a, b in simp]}, 0.0


def section_polygons(mesh: trimesh.Trimesh, z: float) -> list[Polygon]:
    """Material at height z (even-odd over all section loops)."""
    sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if sec is None:
        return []
    mat = None
    for p in sec.discrete:
        p = np.asarray(p)
        if len(p) < 4 or not np.allclose(p[0], p[-1]):
            continue
        poly = Polygon(p[:, :2])
        if not poly.is_valid:
            poly = poly.buffer(0)
        mat = poly if mat is None else mat.symmetric_difference(poly)
    if mat is None or mat.is_empty:
        return []
    polys = list(mat.geoms) if hasattr(mat, "geoms") else [mat]
    return [p for p in polys if p.geom_type == "Polygon" and p.area > 1e-6]


def _levels(am: trimesh.Trimesh, merge: float) -> list[float]:
    flat = np.abs(am.face_normals[:, 2]) > 0.9995
    zs = np.sort(am.triangles_center[flat, 2])
    lv = [0.0, float(am.bounds[1, 2])]
    for z in zs:
        lv.append(float(z))
    lv.sort()
    out = [lv[0]]
    for z in lv[1:]:
        if z - out[-1] > merge:
            out.append(z)
    if am.bounds[1, 2] - out[-1] > merge:
        out.append(float(am.bounds[1, 2]))
    else:
        out[-1] = float(am.bounds[1, 2])
    return out


def build_layers(am: trimesh.Trimesh, simplify_eps: float) -> tuple[list[Layer], float]:
    """Layers for an aligned mesh. Raises ValueError if the part has too many levels.

    Returns (layers, max_circle_fit_error).
    """
    diag = float(np.linalg.norm(am.extents))
    levels = _levels(am, merge=max(1e-3, 1e-4 * diag))
    if len(levels) - 1 > MAX_LAYERS:
        raise ValueError(f"{len(levels) - 1} height levels (limit {MAX_LAYERS})")
    layers: list[Layer] = []
    worst = 0.0
    for z0, z1 in zip(levels[:-1], levels[1:]):
        polys = section_polygons(am, (z0 + z1) / 2)
        regions = []
        for poly in polys:
            outer, e0 = ring_to_loop(poly.exterior.coords, simplify_eps)
            holes, errs = [], [e0]
            for ring in poly.interiors:
                loop, e = ring_to_loop(ring.coords, simplify_eps)
                holes.append(loop)
                errs.append(e)
            worst = max(worst, *errs)
            c = poly.centroid
            regions.append(Region(outer, holes, float(poly.area), [float(c.x), float(c.y)]))
        layer = Layer(z0, z1, regions, polys)
        if layers and _same(layers[-1], layer):
            layers[-1].z1 = z1  # identical cross-section: merge
            continue
        layers.append(layer)
    return [l for l in layers if l.regions], worst


def _same(a: Layer, b: Layer) -> bool:
    if len(a.raw) != len(b.raw):
        return False
    from shapely.ops import unary_union
    ua, ub = unary_union(a.raw), unary_union(b.raw)
    return ua.symmetric_difference(ub).area <= 1e-6 * max(ua.area, 1.0)
