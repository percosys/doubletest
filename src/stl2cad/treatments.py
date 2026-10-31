"""Recover chamfers and fillets on the edges of a layered model.

For every level between layers, measure how far the real surface pulls away from the
layer's sharp outline at several depths, giving an offset-versus-depth profile. A
chamfer is a straight profile, a fillet is a quarter circle. No LLM.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares
import os
import sys

import shapely
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from .layers import Layer, section_polygons, _union

def _trace(msg: str) -> None:
    if os.environ.get("STL2CAD_TRACE"):
        print(f"[treat] {msg}", file=sys.stderr, flush=True)


FAMILIES = ("convex_top", "convex_bottom", "concave_above", "concave_below")


@dataclass
class Treatment:
    z: float  # the level the edges lie on
    family: str
    kind: str  # "chamfer" | "fillet"
    size_xy: float  # chamfer: offset leg; fillet: radius
    size_z: float  # chamfer: height leg; fillet: radius
    rms: float
    layer: int  # index of the layer whose outline the edges belong to
    side: str  # "below" the level or "above" it
    curve: object = field(repr=False, default=None)  # shapely linework (not serialised)
    foot: object = field(repr=False, default=None)  # footprint limiting added material

    def profile(self, d: float) -> float:
        """Horizontal offset at depth d from the level."""
        if self.kind == "chamfer":
            return self.size_xy * max(1.0 - d / self.size_z, 0.0)
        r = self.size_xy
        return r - float(np.sqrt(max(r * r - (r - d) ** 2, 0.0))) if d < r else 0.0

    def band(self, layer: Layer) -> tuple[float, float]:
        return (self.z - self.size_z, self.z) if self.side == "below" else (self.z, self.z + self.size_z)

    def apply(self, g, o: float):
        if o <= 1e-9:
            return g
        ring = self.curve.buffer(o, cap_style=2)
        if self.family.startswith("convex"):
            return g.difference(ring)
        return g.union(ring.intersection(self.foot))


def _fit(deltas: np.ndarray, o: np.ndarray):
    """Best chamfer or fillet for the offset profile. Returns (kind, size_xy, size_z, rms)."""
    o0 = float(o[0])

    def rms(res):
        return float(np.sqrt(np.mean(res**2)))

    first_zero = deltas[np.argmax(o < 0.1 * o0)] if np.any(o < 0.1 * o0) else deltas[-1]
    ch = least_squares(lambda p: p[0] * np.clip(1 - deltas / p[1], 0, None) - o,
                       [o0 * 1.05, max(first_zero, deltas[0] * 1.5)],
                       bounds=([0, deltas[0] * 0.5], [np.inf, 100.0]))

    def fil(p):
        r = p[0]
        m = np.where(deltas < r, r - np.sqrt(np.maximum(r * r - (r - deltas) ** 2, 0)), 0.0)
        return m - o

    fi = least_squares(fil, [max(o0, first_zero)], bounds=([deltas[0] * 0.5], [100.0]))
    rc, rf = rms(ch.fun), rms(fi.fun)
    if rf < rc:
        return "fillet", float(fi.x[0]), float(fi.x[0]), rf
    return "chamfer", float(ch.x[0]), float(ch.x[1]), rc


def _solve_offset(area: float, curve, ref, removed: bool, foot, hi: float) -> float:
    """Offset o whose band along `curve` has the measured area.

    Area / length is biased on curved edges (about o^2 / 2R). A few fixed-point steps
    on the true band area remove the bias (the band area is close to linear in o).
    """
    if area <= 1e-9:
        return 0.0

    def band(o):
        ring = curve.buffer(o, cap_style=2, quad_segs=4)
        g = ring.intersection(ref) if removed else ring.intersection(foot).difference(ref)
        return g.area

    o = min(area / curve.length, hi)
    for _ in range(4):
        a = band(o)
        if a <= 1e-12:
            break
        o = min(o * area / a, hi)
    return o


MAX_STEPS = 60  # slabs per treatment band when verifying (bounds time and memory)
SIMPLIFY = 0.002  # mm; geometry used for measuring is thinned to this (eps must exceed it)


def _geom(layer: Layer):
    g = _union(layer)
    return g.simplify(SIMPLIFY, preserve_topology=True) if not g.is_empty else g


def detect_treatments(am, layers: list[Layer], tol: float, max_size: float = 6.0,
                      eps: float = 0.01) -> list[Treatment]:
    out: list[Treatment] = []
    cache: dict[float, object] = {}

    def section(z: float):
        key = round(z, 5)
        if key not in cache:
            polys = section_polygons(am, z)
            g = unary_union(polys) if polys else Polygon()
            cache[key] = g.simplify(SIMPLIFY, preserve_topology=True) if not g.is_empty else g
        return cache[key]

    for k in range(-1, len(layers)):  # boundary between layer k (A) and k+1 (B)
        A = _geom(layers[k]) if k >= 0 else None
        B = _geom(layers[k + 1]) if k + 1 < len(layers) else None
        z = layers[k].z1 if k >= 0 else layers[0].z0
        fams = []  # (family, curve, layer index, side, reference, removed?, footprint)
        if A is not None:
            ab = A.boundary
            fams.append(("convex_top", ab if B is None else ab.difference(B.buffer(eps)),
                         k, "below", A, True, None))
            if B is not None:
                fams.append(("concave_below", ab.intersection(B.buffer(-eps)), k, "below", A, False, B))
        if B is not None:
            bb = B.boundary
            fams.append(("convex_bottom", bb if A is None else bb.difference(A.buffer(eps)),
                         k + 1, "above", B, True, None))
            if A is not None:
                fams.append(("concave_above", bb.intersection(A.buffer(-eps)), k + 1, "above", B, False, A))
        for fam, curve_all, li, side, ref, removed, foot in fams:
            if curve_all.is_empty:
                continue
            th = layers[li].z1 - layers[li].z0
            dmin, dmax = max(0.02, 0.2 * tol), min(max_size, 0.45 * th)
            if dmax < 2 * dmin:
                continue
            deltas = np.geomspace(dmin, dmax, 9)
            # Fit each connected edge loop on its own: a hole's countersink must not be
            # averaged away by a much longer outline with a different profile.
            merged = shapely.line_merge(curve_all)
            pieces = [g for g in (merged.geoms if hasattr(merged, "geoms") else [merged])
                      if isinstance(g, LineString) and g.length >= 1.0]
            for piece in pieces:
                curve = piece.simplify(SIMPLIFY)
                near = curve.buffer(max_size, cap_style=2, quad_segs=4)
                o = []
                for d in deltas:
                    zs = z - d if side == "below" else z + d
                    sec = section(zs)
                    diff = ref.difference(sec) if removed else sec.difference(ref).intersection(foot)
                    o.append(_solve_offset(diff.intersection(near).area, curve, ref, removed, foot, max_size))
                o = np.array(o)
                _trace(f"z={z:.2f} {fam} len={piece.length:.1f} o={np.round(o, 3).tolist()}")
                if o.max() < 0.5 * tol:
                    continue
                kind, sxy, sz, rms = _fit(deltas, o)
                _trace(f"   fit {kind} xy={sxy:.3f} z={sz:.3f} rms={rms:.4f} "
                       f"(limit {max(0.3 * tol, 0.1 * o.max()):.4f})")
                if rms > max(0.3 * tol, 0.1 * o.max()):
                    continue  # profile is neither a clean chamfer nor a fillet
                out.append(Treatment(z, fam, kind, sxy, sz, rms, li, side, curve, foot))
    return _group(out, tol)


def _group(ts: list[Treatment], tol: float) -> list[Treatment]:
    """Loops that came out the same (kind, level, size) become one treatment / one Fusion feature."""
    q = max(0.05, tol)
    groups: dict[tuple, list[Treatment]] = {}
    for t in ts:
        key = (round(t.z, 3), t.family, t.kind, t.layer, round(t.size_xy / q), round(t.size_z / q))
        groups.setdefault(key, []).append(t)
    out = []
    for g in groups.values():
        t0 = g[0]
        if len(g) > 1:
            w = np.array([t.curve.length for t in g])
            t0 = Treatment(t0.z, t0.family, t0.kind, float(np.average([t.size_xy for t in g], weights=w)),
                           float(np.average([t.size_z for t in g], weights=w)), max(t.rms for t in g),
                           t0.layer, t0.side, unary_union([t.curve for t in g]), t0.foot)
        out.append(t0)
    return out
    return out


def layer_slabs(layer: Layer, k: int, treatments: list[Treatment], step: float):
    """(z0, z1, geometry) slabs of one layer with its edge treatments applied."""
    base = _union(layer)
    ts = [t for t in treatments if t.layer == k]
    cuts = {layer.z0, layer.z1}
    for t in ts:
        lo, hi = t.band(layer)
        lo, hi = max(lo, layer.z0), min(hi, layer.z1)
        n = min(max(10, int(np.ceil((hi - lo) / step))), MAX_STEPS)
        cuts.update(lo + (hi - lo) * j / n for j in range(n + 1))
    zs = sorted(cuts)
    out = []
    for a, b in zip(zs[:-1], zs[1:]):
        if b - a < 1e-9:
            continue
        zm, g = (a + b) / 2, base
        for t in ts:
            lo, hi = t.band(layer)
            if lo <= zm <= hi:
                g = t.apply(g, t.profile(abs(zm - t.z)))
        out.append((a, b, g.simplify(SIMPLIFY, preserve_topology=True)))
    return out
