"""Load, clean and align an STL."""

from __future__ import annotations

import numpy as np
import trimesh


def load_clean(path: str) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{path}: not a single triangle mesh")
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    trimesh.repair.fix_normals(mesh)
    return mesh


def align(mesh: trimesh.Trimesh) -> tuple[trimesh.Trimesh, np.ndarray]:
    """Move the mesh into its oriented bounding box frame (min corner at origin).

    Returns the aligned copy and the 4x4 transform that was applied. Units are
    assumed to be millimetres.
    """
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    aligned = mesh.copy()
    aligned.apply_transform(to_origin)
    # oriented_bounds centres the box on the origin; shift to the min corner.
    shift = np.eye(4)
    shift[:3, 3] = extents / 2.0
    aligned.apply_transform(shift)
    return aligned, shift @ to_origin
