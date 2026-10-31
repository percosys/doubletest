"""Check a rebuild against the input mesh (exact point-to-surface distances)."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
import trimesh

from .layers import Layer


@dataclass
class Deviation:
    max_mm: float
    mean_mm: float
    p99_mm: float
    volume_diff_pct: float
    samples: int

    def to_dict(self) -> dict:
        return {k: round(v, 5) if isinstance(v, float) else v for k, v in asdict(self).items()}


def layers_to_mesh(layers: list[Layer]) -> trimesh.Trimesh:
    """Rebuild the solid from the raw section polygons (same vertices as the STL)."""
    parts = []
    for L in layers:
        for poly in L.raw:
            m = trimesh.creation.extrude_polygon(poly, L.z1 - L.z0)
            m.apply_translation([0, 0, L.z0])
            parts.append(m)
    if not parts:
        raise ValueError("no layers")
    if len(parts) == 1:
        return parts[0]
    return trimesh.boolean.union(parts, engine="manifold")


def slabs_to_mesh(slabs) -> trimesh.Trimesh:
    """Solid from (z0, z1, shapely geometry) slabs."""
    parts = []
    for z0, z1, g in slabs:
        for poly in (g.geoms if hasattr(g, "geoms") else [g]):
            if poly.geom_type != "Polygon" or poly.area < 1e-9:
                continue
            m = trimesh.creation.extrude_polygon(poly, z1 - z0)
            m.apply_translation([0, 0, z0])
            parts.append(m)
    if not parts:
        raise ValueError("no slabs")
    return parts[0] if len(parts) == 1 else trimesh.boolean.union(parts, engine="manifold")


def surface_from_slabs(slabs) -> tuple[trimesh.Trimesh, float]:
    """Exterior surface of stacked slabs, built directly (no boolean union).

    Between interfaces the exterior is each slab's side walls; at each interface it is
    the part of one slab not covered by its neighbour (up- or down-facing ledges).
    Returns (triangle soup, volume). A boolean union of hundreds of thin coplanar slabs
    leaves stray internal shells, which made the verification wrong.
    """
    from shapely.geometry import Polygon

    slabs = [(a, b, g) for a, b, g in slabs if b - a > 1e-6 and not g.is_empty]
    slabs.sort(key=lambda s: s[0])
    verts: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    n = 0

    def add(v, f):
        nonlocal n
        verts.append(v)
        faces.append(f + n)
        n += len(v)

    def polys(g):
        return [p for p in (g.geoms if hasattr(g, "geoms") else [g])
                if isinstance(p, Polygon) and p.area > 1e-9]

    def cap(g, z):
        for p in polys(g):
            v2, f = trimesh.creation.triangulate_polygon(p, engine="earcut")
            add(np.c_[v2, np.full(len(v2), z)], np.asarray(f))

    volume = 0.0
    prev_g, prev_top = None, None
    for z0, z1, g in slabs:
        volume += g.area * (z1 - z0)
        for p in polys(g):  # side walls
            for ring in [p.exterior, *p.interiors]:
                c = np.asarray(ring.coords)
                a, b = c[:-1], c[1:]
                m = len(a)
                v = np.vstack([np.c_[a, np.full(m, z0)], np.c_[b, np.full(m, z0)],
                               np.c_[b, np.full(m, z1)], np.c_[a, np.full(m, z1)]])
                i = np.arange(m)
                f = np.vstack([np.c_[i, i + m, i + 2 * m], np.c_[i, i + 2 * m, i + 3 * m]])
                add(v, f)
        if prev_g is None or abs(prev_top - z0) > 1e-6:  # bottom of a stack
            if prev_g is not None:
                cap(prev_g, prev_top)
            cap(g, z0)
        else:  # interface with the slab below
            cap(prev_g.difference(g), z0)
            cap(g.difference(prev_g), z0)
        prev_g, prev_top = g, z1
    if prev_g is not None:
        cap(prev_g, prev_top)
    if not verts:
        raise ValueError("no slabs")
    return trimesh.Trimesh(np.vstack(verts), np.vstack(faces), process=False), float(volume)


def _distances(mesh: trimesh.Trimesh, pts: np.ndarray, chunk: int = 500) -> np.ndarray:
    """Exact point-to-surface distances, in chunks (trimesh builds candidate lists for
    all query points at once, which needs gigabytes on 200k-face meshes)."""
    return np.concatenate([trimesh.proximity.closest_point(mesh, pts[i:i + chunk])[1]
                           for i in range(0, len(pts), chunk)])


def surface_distances(original: trimesh.Trimesh, rebuilt: trimesh.Trimesh, n: int,
                      seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """(sample points on `original`, their distance to `rebuilt`)."""
    np.random.seed(seed)
    pts = original.sample(n)
    return pts, _distances(rebuilt, pts)


def deviation(original: trimesh.Trimesh, rebuilt: trimesh.Trimesh, n: int = 8000,
              seed: int = 0, rebuilt_volume: float | None = None) -> Deviation:
    """Two-sided surface distance between the meshes, plus volume difference.

    Pass ``rebuilt_volume`` when ``rebuilt`` is a triangle soup (surface_from_slabs).
    """
    np.random.seed(seed)  # trimesh.sample uses the global RNG
    d1 = _distances(rebuilt, original.sample(n))
    d2 = _distances(original, rebuilt.sample(n))
    d = np.r_[d1, d2]
    v0 = abs(original.volume)
    v1 = rebuilt_volume if rebuilt_volume is not None else abs(rebuilt.volume)
    return Deviation(float(d.max()), float(d.mean()), float(np.percentile(d, 99)),
                     100.0 * abs(v1 - v0) / max(v0, 1e-9), int(len(d)))
