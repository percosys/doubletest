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
              seed: int = 0) -> Deviation:
    """Two-sided surface distance between the meshes, plus volume difference."""
    np.random.seed(seed)  # trimesh.sample uses the global RNG
    d1 = _distances(rebuilt, original.sample(n))
    d2 = _distances(original, rebuilt.sample(n))
    d = np.r_[d1, d2]
    v0, v1 = abs(original.volume), abs(rebuilt.volume)
    return Deviation(float(d.max()), float(d.mean()), float(np.percentile(d, 99)),
                     100.0 * abs(v1 - v0) / max(v0, 1e-9), int(len(d)))
