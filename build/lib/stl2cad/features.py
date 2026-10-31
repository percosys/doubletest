"""Turn raw segments into candidate CAD features (measurements only, no judgment)."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh

from .segment import Segmentation

FULL_WRAP_DEG = 300.0


@dataclass
class Hole:
    id: str
    center: list[float]
    axis: list[float]
    diameter: float
    depth: float


@dataclass
class Boss:
    id: str
    center: list[float]
    axis: list[float]
    diameter: float
    height: float


@dataclass
class Arc:
    """Partial cylinder (fillet / slot end / rounded edge candidate)."""

    id: str
    center: list[float]
    axis: list[float]
    radius: float
    length: float
    coverage_deg: float
    concave: bool


@dataclass
class Facet:
    """Planar face that is not parallel to a principal axis (chamfer/draft candidate)."""

    id: str
    normal: list[float]
    area: float


@dataclass
class Features:
    extents: list[float]
    volume: float
    holes: list[Hole] = field(default_factory=list)
    bosses: list[Boss] = field(default_factory=list)
    arcs: list[Arc] = field(default_factory=list)
    facets: list[Facet] = field(default_factory=list)
    axis_aligned_plane_count: int = 0
    unexplained_area_fraction: float = 0.0


def _r(v) -> list[float]:
    return [round(float(x), 4) for x in v]


def extract(mesh: trimesh.Trimesh, seg: Segmentation, axis_tol: float = 0.995) -> Features:
    f = Features(
        extents=_r(mesh.extents),
        volume=round(float(abs(mesh.volume)), 3),
        unexplained_area_fraction=round(seg.unexplained_area_fraction, 4),
    )
    for p in seg.planes:
        if np.abs(p.normal).max() >= axis_tol:
            f.axis_aligned_plane_count += 1
        else:
            f.facets.append(Facet(f"facet_{len(f.facets) + 1}", _r(p.normal), round(p.area, 3)))

    for c in seg.cylinders:
        if c.coverage_deg >= FULL_WRAP_DEG:
            if c.concave:
                f.holes.append(Hole(f"hole_{len(f.holes) + 1}", _r(c.center), _r(c.axis),
                                    round(2 * c.radius, 4), round(c.length, 4)))
            else:
                f.bosses.append(Boss(f"boss_{len(f.bosses) + 1}", _r(c.center), _r(c.axis),
                                     round(2 * c.radius, 4), round(c.length, 4)))
        else:
            f.arcs.append(Arc(f"arc_{len(f.arcs) + 1}", _r(c.center), _r(c.axis),
                              round(c.radius, 4), round(c.length, 4),
                              round(c.coverage_deg, 1), c.concave))
    return f
