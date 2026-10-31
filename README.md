# stl2cad

Convert an STL into a CAD model, deterministically. Built as a Fusion add-in: insert an STL,
select the mesh body, click **Convert Mesh to Model**.

## How it works

1. **Deterministic 2.5D rebuild (no LLM).** Find the main extrusion axis, cut the mesh into layers
   at every flat-face height, and take each layer's exact cross-section (circles stay circles,
   everything else becomes a simplified polyline). Plates, brackets, steps, pockets, bosses and
   through/blind holes come out as sketches + extrudes (join). Layers thinner than the tolerance
   (tessellation noise, tiny edge rounds) are merged into their most similar neighbour, and the
   merge is kept only if verification still passes.
2. **Edge treatments (no LLM).** If the sharp layered model is out of tolerance, the gap along each
   layer outline is measured at several depths, giving an offset-versus-depth profile that is fitted
   as a chamfer (straight) or fillet (quarter circle), convex or concave. They are added as Fusion
   chamfer/fillet features on the matching edges and included in the verification model.
3. **Verify.** The rebuild is compared with the STL (exact two-sided surface distance + volume).
   It is accepted only if max deviation + simplification <= tolerance (default 0.05 mm).
4. **Fallback.** Otherwise the add-in falls back to Fusion's own mesh conversion, so you still
   get something faithful to the STL.
5. **Optional AI (off by default).** "Snap holes to standard sizes" asks JEV (TypeSafe) whether
   each hole is a clearance or tap-drill screw hole; code does the size arithmetic and only
   changes a radius by <= 0.1 mm. Geometry never depends on the AI, and a failed call is reported,
   not treated as an answer.

```
Fusion mesh body -> STL (mm) -> stl2cad.worker -> plan.json -> sketches + extrudes in Fusion
                                  (analysis + verification run in an external Python venv)
```

## Install the add-in (Windows/macOS)

```
python scripts/install_addin.py        # needs Python >= 3.10; creates a venv inside the add-in
```
Then in Fusion: Utilities > Add-Ins > Scripts and Add-Ins > Add-Ins > STL2CAD > Run. The button is in
Solid > Modify. For the optional snap step, set `TYPESAFE_API_KEY` or put `{"api_key": "..."}` in
`~/.stl2cad/config.json`.

## Use without Fusion

```
pip install -e '.[dev]'
python -m stl2cad.worker part.stl --out plan.json [--tol 0.05] [--snap]
pytest
```
(`stl2cad part.stl` is the older recipe/CadQuery path: stock + holes only.)

## Status

Verified here (18 tests, plus runs on synthetic parts and one real STL):
- Analysis, layer extraction, circle detection, tilted parts, stepped parts with pockets,
  non-rectangular outlines, verification, fallback decision, optional JEV snap, installer, units.
- Live JEV calls (jev-1.14.0) for the hole-intent questions.
- Real STL (3-arm grinder disc holder, 219k faces): nearly 2.5D (5 layers, mean deviation 0.08 mm,
  volume error 0.18%) but 2.8 mm worst-case, so it correctly falls back. Analysis took ~75 s.

**Not verified: the Fusion add-in itself.** `STL2CAD.py` and `plan_build.py` use the Fusion API
(mesh export, sketches, extrudes, Move feature, mesh conversion) as I understand it, but I could
not run Fusion or read the API reference from the build environment. Expect to fix some API
details on first run. The builder's logic (profile matching, layer order, join vs new body) is
tested against a mock. The mesh-conversion fallback call is the least certain; if it fails the
add-in tells you to use Mesh > Modify > Convert Mesh manually.

## Limits

- Exact only for 2.5D parts (everything extruded along one axis), plus chamfers/fillets whose profile
  is uniform along each edge run. Revolved/turned parts, sloped or curved faces, variable-radius
  blends, fillets larger than `--max-edge` (6 mm) and organic shapes fall back to mesh conversion.
- Edge treatments are searched on one axis only (the one with the most flat area, unless another is
  twice as accurate); an unusual part could have a better axis that is not tried.
- Arcs in outlines (e.g. rounded corners) become many short line segments, not true arcs.
- Circles are fitted to the STL's vertices; a coarse STL (few segments per circle) deviates from
  the true circle by up to its facet sagitta, which the deviation check does not count.
- Results are plain sketches and extrudes, not named features; no dimensions/constraints yet.
- Large meshes are slow (~1 min for 200k faces) and block Fusion while the worker runs.
- Not watertight meshes skip the 2.5D path.
