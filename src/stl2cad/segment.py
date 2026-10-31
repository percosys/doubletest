"""Primitive segmentation: planes and cylinders by region growing + least squares."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh


@dataclass
class PlaneSeg:
    faces: np.ndarray
    normal: np.ndarray
    d: float  # plane: normal . x = d
    area: float


@dataclass
class CylSeg:
    faces: np.ndarray
    axis: np.ndarray
    center: np.ndarray  # point on the axis, at mid-length
    radius: float
    length: float
    coverage_deg: float  # angular extent of the surface around the axis
    concave: bool  # True when the solid's normals point toward the axis (hole-like)
    area: float
    rms: float  # radial RMS residual / radius


@dataclass
class Segmentation:
    planes: list[PlaneSeg]
    cylinders: list[CylSeg]
    unexplained_faces: np.ndarray
    unexplained_area_fraction: float


def _components(n: int, edges: np.ndarray) -> list[np.ndarray]:
    if len(edges) == 0:
        return [np.array([i]) for i in range(n)]
    return [np.asarray(c) for c in trimesh.graph.connected_components(edges, nodes=np.arange(n))]


def _fit_plane(mesh: trimesh.Trimesh, faces: np.ndarray, tol: float) -> PlaneSeg | None:
    w = mesh.area_faces[faces]
    n = (mesh.face_normals[faces] * w[:, None]).sum(axis=0)
    norm = np.linalg.norm(n)
    if norm == 0:
        return None
    n /= norm
    verts = mesh.vertices[np.unique(mesh.faces[faces])]
    proj = verts @ n
    d = float(proj.mean())
    if np.abs(proj - d).max() > tol:
        return None
    return PlaneSeg(faces=faces, normal=n, d=d, area=float(w.sum()))


def _fit_cylinder(
    mesh: trimesh.Trimesh, faces: np.ndarray, fit_tol: float, max_radius: float
) -> CylSeg | None:
    N = mesh.face_normals[faces]
    w = mesh.area_faces[faces]
    M = (N * w[:, None]).T @ N / w.sum()
    evals, evecs = np.linalg.eigh(M)
    # Normals of a cylinder are perpendicular to its axis (smallest eigenvalue ~ 0)
    # and span a plane (second eigenvalue clearly > 0, otherwise it is a flat patch).
    if evals[0] > 0.02 or evals[1] < 5e-3:
        return None
    axis = evecs[:, 0]
    helper = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    P = mesh.vertices[np.unique(mesh.faces[faces])]
    x, y, t = P @ u, P @ v, P @ axis
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    sol, *_ = np.linalg.lstsq(A, x**2 + y**2, rcond=None)
    a, b, c = sol
    r2 = c + a * a + b * b
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    if r > max_radius:
        return None
    rms = float(np.sqrt(np.mean((np.hypot(x - a, y - b) - r) ** 2)) / r)
    if rms > fit_tol:
        return None

    ang = np.sort(np.arctan2(y - b, x - a))
    gaps = np.diff(np.r_[ang, ang[0] + 2 * np.pi])
    coverage = float(np.degrees(2 * np.pi - gaps.max()))

    t0, t1 = t.min(), t.max()
    center = a * u + b * v + axis * (t0 + t1) / 2
    cen = mesh.triangles_center[faces] - center
    radial = cen - np.outer(cen @ axis, axis)
    concave = bool(np.mean(np.einsum("ij,ij->i", N, radial)) < 0)
    return CylSeg(
        faces=faces, axis=axis, center=center, radius=r, length=float(t1 - t0),
        coverage_deg=coverage, concave=concave, area=float(w.sum()), rms=rms,
    )


def segment(
    mesh: trimesh.Trimesh,
    plane_tol_deg: float = 1.0,
    curve_tol_deg: float = 25.0,
    fit_tol: float = 0.02,
    min_cyl_faces: int = 6,
) -> Segmentation:
    diag = float(np.linalg.norm(mesh.extents))
    nfaces = len(mesh.faces)
    adj, ang = mesh.face_adjacency, mesh.face_adjacency_angles
    plane_tol = 0.001 * diag
    planes: list[PlaneSeg] = []
    cyls: list[CylSeg] = []
    claimed = np.zeros(nfaces, dtype=bool)

    # Pass 1: grow over smoothly joined faces and try each whole region as ONE
    # plane or ONE cylinder. (A tessellated cylinder wall is made of coplanar
    # triangle pairs, so a coplanar-first pass would wrongly claim each facet.)
    smooth = adj[ang <= np.radians(curve_tol_deg)]
    rest: list[np.ndarray] = []
    for comp in _components(nfaces, smooth):
        p = _fit_plane(mesh, comp, plane_tol) if len(comp) >= 2 else None
        if p is not None:
            planes.append(p)
            claimed[comp] = True
            continue
        cyl = _fit_cylinder(mesh, comp, fit_tol, max_radius=diag) if len(comp) >= min_cyl_faces else None
        if cyl is not None:
            cyls.append(cyl)
            claimed[comp] = True
        else:
            rest.append(comp)

    # Pass 2: regions that were neither (e.g. a flat face blended into a fillet):
    # split by coplanarity and keep the flat parts.
    flat = adj[ang <= np.radians(plane_tol_deg)]
    for comp in rest:
        inside = np.zeros(nfaces, dtype=bool)
        inside[comp] = True
        edges = flat[inside[flat[:, 0]] & inside[flat[:, 1]]]
        for sub in _components(nfaces, edges):
            if not inside[sub[0]] or len(sub) < 2:
                continue
            p = _fit_plane(mesh, sub, plane_tol)
            if p is not None:
                planes.append(p)
                claimed[sub] = True

    # Pass 3: cylinders among whatever is left.
    left = ~claimed
    ok = left[adj[:, 0]] & left[adj[:, 1]] & (ang <= np.radians(curve_tol_deg))
    for comp in _components(nfaces, adj[ok]):
        if not left[comp[0]] or len(comp) < min_cyl_faces:
            continue
        cyl = _fit_cylinder(mesh, comp, fit_tol, max_radius=diag)
        if cyl is not None:
            cyls.append(cyl)
            claimed[comp] = True

    unexplained = np.where(~claimed)[0]
    frac = float(mesh.area_faces[unexplained].sum() / mesh.area_faces.sum())
    return Segmentation(planes, cyls, unexplained, frac)
