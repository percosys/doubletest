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


def deviation(original: trimesh.Trimesh, rebuilt: trimesh.Trimesh, n: int = 8000,
              seed: int = 0) -> Deviation:
    """Two-sided surface distance between the meshes, plus volume difference."""
    np.random.seed(seed)  # trimesh.sample uses the global RNG
    d1 = trimesh.proximity.closest_point(rebuilt, original.sample(n))[1]
    d2 = trimesh.proximity.closest_point(original, rebuilt.sample(n))[1]
    d = np.r_[d1, d2]
    v0, v1 = abs(original.volume), abs(rebuilt.volume)
    return Deviation(float(d.max()), float(d.mean()), float(np.percentile(d, 99)),
                     100.0 * abs(v1 - v0) / max(v0, 1e-9), int(len(d)))
