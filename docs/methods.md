# stl2cad: method specification

This document describes *what each step computes*, in plain terms and standard mathematics. The
Rust core implements these methods. Nothing here is copied from another project's code.

## Provenance (read this first)

- The methods below come from textbook geometry and well-known algorithms: least-squares line and
  circle fitting (Kåsa), Ramer–Douglas–Peucker thinning, region growing on face normals,
  RANSAC-style primitive detection, section (plane slicing) of a triangle mesh, even-odd polygon
  filling, point-to-triangle distance with an axis-aligned bounding-box tree, and
  profile-versus-depth curve fitting.
- The idea of reading a part as "levels and slabs along an axis", of recovering fillets/chamfers
  from a section square to an edge, of reading a turned part from a section through its axis, and
  of reading cross-axis holes as revolved cuts is common practice in reverse engineering. During
  planning, the public descriptions of one open-source tool (ScanRuler, AGPL-3.0: its README,
  docstrings and file headers, plus a list of function names) were read. Its implementation code
  was **not** studied, and no code, identifiers, constants, file layout or data formats from it
  are used here.
- This was done by one agent, not by two separated teams, so it is not a formal clean room. If
  provenance matters for your use, have counsel review this section and the git history.
- Rule going forward: new methods are written from this document and standard references only.

## M1. Mesh preparation

Weld vertices that coincide within a small distance, drop zero-area triangles, orient consistently.
A mesh is *closed* if every undirected edge is shared by exactly two triangles. Volume is the sum
of signed tetrahedra against the origin. Non-closed meshes skip the layer method.

## M2. Candidate directions and frame

Group triangles into flat regions by growing over neighbours whose normals differ by less than
about one degree, and accept a region if every vertex lies within a small distance of its mean
plane. Rank plane normals (sign-insensitive) by total area. The top directions are candidate
extrusion axes. The part frame has Z along the axis, X along the major in-plane direction of the
vertices, origin at the minimum corner.

## M3. Layers (slabs) along an axis

Levels are the heights of triangles whose normals are within a few hundredths of a degree of the
axis, merged within 1e-4 of the part's diagonal, plus the bottom and top. Between two levels the
cross-section is constant for an extrusion, so each layer's section is taken at its middle height.
Layers thinner than the tolerance are merged into the neighbour with the smallest symmetric
difference; adjacent layers whose sections are equal become one. The merge is kept only if the
verification (M9) still passes, otherwise the unmerged layers are used.

## M4. Section loops and regions

Slice every triangle that crosses the plane, chain the segments into closed loops by matching end
points, and fill by the even-odd rule: nested loops alternate between material and hole.
Each region is an outer loop plus the holes inside it.

## M5. Loops to exact lines and arcs

Input: a closed polyline from M4 and a tolerance. Output: a loop of lines, three-point arcs and
full circles.

1. Whole circle: fit a circle algebraically (solve x²+y² = 2ax+2by+c in least squares); accept if
   the RMS radial residual is below 0.2 % of the radius, there are enough points, and no angular
   gap exceeds 60°.
2. Corners: mark points whose turning angle exceeds a threshold, after smoothing over a few
   neighbours, so runs between corners are smooth.
3. Each run is a line if a total-least-squares line fits within the tolerance, else an arc if a
   circle fit stays within the tolerance and the angles advance monotonically, else it is split at
   the point of greatest deviation and the halves are tried again (split-and-merge).
4. Neighbouring pieces are joined at their intersection (line-line), or at the foot point (line or
   arc that is tangent), so the loop closes exactly. Adjacent collinear lines and co-circular arcs
   are merged.

The worst distance from the original polyline to the pieces is recorded and counted in M9.

## M6. Fillets and chamfers by profile

For every edge curve of a layer outline at a level, and for several depths below or above the level,
measure how far the real surface is from the layer's sharp outline, **square to the curve**: from a
point on the curve step along the inward normal to where the section at that depth crosses it.
This gives an offset-versus-depth profile for the curve (taking the median over its points, with
the spread as an uncertainty). Fit two models by least squares: a chamfer, offset = a(1 − depth/b),
and a quarter-circle fillet of radius r, offset = r − √(r² − (r − depth)²). Keep the better one
if its RMS is small relative to the offset, otherwise report "unclassified". Fit each connected
curve on its own, then group curves with the same kind, level and size.

A concave corner (material added) is measured the same way with the sign reversed.
A chamfer's two distances are reported with the face they belong to (the lower or upper face)
so the CAD side can pick the right face.

## M7. Turned parts by revolve (implemented)

A part is *turned about Z* (in the frame of M2) if every section square to Z, sampled at 41 heights
away from the end caps, consists only of circles (RMS radial residual under 0.5 % of the radius
plus 0.2 x tolerance, no angular gap over 60 degrees) with one common centre (within a quarter of
the tolerance plus 0.2 % of the radius). Tiny sections near a cone or sphere tip are ignored. This
holds for cylinders, cones, tori and spheres alike, so one test covers all of them.

The profile is the cut by the plane through the axis, restricted to radius >= 0. The mesh is
re-indexed so that this plane becomes a height plane, the section is taken with M4, and the region is
intersected with the half-plane. The region (axial, radial) is swapped to sketch coordinates
(x = radius, y = axial) and fitted with M5: lines for cylinders, cones and planes, arcs for tori
and spheres, a full circle for a torus cross-section.

Verification revolves the raw profile polygon (angular step chosen so the revolve's own chord sag is
under 10 % of the tolerance) and compares it with M9; the volume is Pappus' 2 pi x area x centroid
radius. A turned part that passes is emitted as `strategy: "revolve"`; one that does not falls back
to the layer method. The sketch plane is XY, the revolve is about the sketch Y axis, and the plan's
frame maps that local frame to the mesh's pose.

Known limits: a plane through the axis misses the tessellation's vertex lines by a tiny amount, so
the profile carries sub-tolerance noise (a zigzag of ~0.0004 mm on a fine mesh). The fitter therefore
measures corners over a short baseline, drops near-duplicate points, and allows noise-sized
back-steps when it checks that an arc advances in one direction.

## M8. Holes across the main axis

After the layers, find concave cylinders (and cones) whose axes are not the main axis: fit
cylinders by least squares to the unexplained faces. For each axis, build its radius-versus-position
profile (steps for cylinders, slopes for countersinks), close it at a floor if a flat face lies
square to the axis inside the radius, and cut it as a revolve through the body.

## M9. Verification

Distance is symmetric: sample points on each surface proportional to triangle area and find the
closest point on the other surface (axis-aligned bounding-box tree, point-to-triangle distance by
region). Report the maximum, mean and 99th percentile, plus the volume difference. A rebuild is
accepted if the maximum plus the worst fitting/simplification error is within the tolerance.
Residual samples above the tolerance are clustered by single-linkage distance and reported with
their location, so a failure says *where* it failed.

The rebuilt solid is represented without a boolean union: its exterior is the side walls of every
slab plus the horizontal ledges where adjacent slabs differ (polygon differences, triangulated).

## M10. Design-value rounding

A measured value rounds to a design value if it is within a fraction of the tolerance of the
nearest step: lengths to 0.5 mm below 10 mm and to 1 mm above, radii to 0.5 mm, angles to 5°.
Fastener hole diameters round to the nearest clearance or tap-drill size from a table. A change is
applied only if it moves the geometry by less than the stated allowance, and is re-verified.
No language model is needed for either rule; a model may be asked only whether a hole is a fastener
hole (to choose which table).

## M11. CAD export or scan

If the plane regions fit with residual below 1e-4 of the diagonal and cylinder radii are stable
along their length, the mesh is a CAD export and fits are exact. Otherwise it is a scan: loosen the
tolerance (never finer than three times the measured noise: the RMS distance of region vertices
from their fitted primitive) and expect polylines or splines rather than exact curves.
